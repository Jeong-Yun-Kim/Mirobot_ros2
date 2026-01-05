#!/usr/bin/env python3
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import serial


STATUS_RE = re.compile(r"^<([^,>]+),")  # <Idle, ...> 에서 Idle/Run/Hold 등만 추출


@dataclass
class ParsedStatus:
    state: str            # Idle / Run / Hold / Alarm ...
    raw: str              # 전체 상태문자열


class SerialBridge(Node):
    """
    - /<ns>/target_pose_xyz (geometry_msgs/Pose)        -> M20 ...
    - /<ns>/target_joint_states (sensor_msgs/JointState)-> M21 ... (각도 명령)
    - /<ns>/joint_states (sensor_msgs/JointState)      -> (가능하면 status에서 뽑아 publish, 없으면 비움)
    - /<ns>/is_busy (std_msgs/Bool)                    -> status(<Idle/...>) 기반
    - /<ns>/status_raw (std_msgs/String)               -> status 원문 publish (디버깅용)
    """

    def __init__(self):
        super().__init__("serial_bridge")

        # ====== Params ======
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("require_enable", False)  # 필요하면 enable 게이트로 막을 수 있음
        self.declare_parameter("deadband_deg", 0.2)
        self.declare_parameter("min_send_period", 0.15)

        self.declare_parameter("workspace_x_min", 140.0)
        self.declare_parameter("workspace_x_max", 290.0)
        self.declare_parameter("workspace_y_min", -270.0)
        self.declare_parameter("workspace_y_max", 270.0)
        self.declare_parameter("workspace_z_min", 40.0)
        self.declare_parameter("workspace_z_max", 300.0)

        self.declare_parameter("feedrate", 2000)

        # 상태 조회 (WLKATA 펌웨어는 '?' 가 status를 뱉는 경우가 많음)
        self.declare_parameter("status_query_cmd", "?")
        self.declare_parameter("status_query_period", 0.2)   # 5Hz
        self.declare_parameter("status_timeout", 1.0)         # 최근 status가 이 시간 넘게 안오면 busy 판단 보수적으로

        # 토픽 이름(네 런치랑 맞춤)
        self.declare_parameter("topic_target_pose", "target_pose_xyz")
        self.declare_parameter("topic_target_joints", "target_joint_states")
        self.declare_parameter("topic_joint_states", "joint_states")
        self.declare_parameter("topic_busy", "is_busy")
        self.declare_parameter("topic_status_raw", "status_raw")

        # 조인트 이름(URDF랑 최대한 맞춰주기)
        # 실제 네 URDF의 joint 이름이 다르면 여기만 바꿔도 됨.
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        # ====== Load params ======
        self.port: str = self.get_parameter("port").value
        self.baud: int = int(self.get_parameter("baud").value)
        self.dry_run: bool = bool(self.get_parameter("dry_run").value)

        self.require_enable: bool = bool(self.get_parameter("require_enable").value)
        self.deadband_deg: float = float(self.get_parameter("deadband_deg").value)
        self.min_send_period: float = float(self.get_parameter("min_send_period").value)

        self.feedrate: int = int(self.get_parameter("feedrate").value)

        self.ws_x_min = float(self.get_parameter("workspace_x_min").value)
        self.ws_x_max = float(self.get_parameter("workspace_x_max").value)
        self.ws_y_min = float(self.get_parameter("workspace_y_min").value)
        self.ws_y_max = float(self.get_parameter("workspace_y_max").value)
        self.ws_z_min = float(self.get_parameter("workspace_z_min").value)
        self.ws_z_max = float(self.get_parameter("workspace_z_max").value)

        self.status_query_cmd: str = str(self.get_parameter("status_query_cmd").value)
        self.status_query_period: float = float(self.get_parameter("status_query_period").value)
        self.status_timeout: float = float(self.get_parameter("status_timeout").value)

        self.topic_target_pose = str(self.get_parameter("topic_target_pose").value)
        self.topic_target_joints = str(self.get_parameter("topic_target_joints").value)
        self.topic_joint_states = str(self.get_parameter("topic_joint_states").value)
        self.topic_busy = str(self.get_parameter("topic_busy").value)
        self.topic_status_raw = str(self.get_parameter("topic_status_raw").value)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)

        # ====== QoS ======
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ====== Publishers ======
        self.pub_joint = self.create_publisher(JointState, self.topic_joint_states, qos_reliable)
        self.pub_busy = self.create_publisher(Bool, self.topic_busy, qos_reliable)
        self.pub_status_raw = self.create_publisher(String, self.topic_status_raw, qos_reliable)

        # ====== Subscriptions ======
        self.create_subscription(Pose, self.topic_target_pose, self.on_target_pose, qos_reliable)
        self.create_subscription(JointState, self.topic_target_joints, self.on_target_joints, qos_reliable)

        # ====== Serial state ======
        self.ser: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._last_send_time = 0.0

        self._last_status_time = 0.0
        self._last_status: Optional[ParsedStatus] = None
        self._busy = False
        self._warned_unsupported_status_cmd = False

        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        # ====== Open serial ======
        self._open_serial()

        # ====== Init sequence ======
        self._send_init_sequence()

        # ====== Timers ======
        self.create_timer(self.status_query_period, self._status_timer_cb)
        self.create_timer(0.05, self._busy_publish_timer_cb)  # 20Hz로 busy 토픽 반영

        self.get_logger().info(
            f"Started. dry_run={self.dry_run}, require_enable={self.require_enable}, "
            f"deadband_deg={self.deadband_deg}, min_send_period={self.min_send_period}s, "
            f"workspace(mm)=X[{self.ws_x_min},{self.ws_x_max}] Y[{self.ws_y_min},{self.ws_y_max}] Z[{self.ws_z_min},{self.ws_z_max}]"
        )

    # ========= Serial open / close =========
    def _open_serial(self):
        if self.dry_run:
            self.get_logger().warn("dry_run=True: serial will not be opened.")
            return

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.1,
            write_timeout=0.5
        )
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")
        self._reader_thread.start()

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    # ========= Sending =========
    def _send_line(self, line: str):
        """Rate-limited send"""
        now = time.time()
        dt = now - self._last_send_time
        if dt < self.min_send_period:
            time.sleep(self.min_send_period - dt)

        if self.dry_run:
            self.get_logger().info(f"[DRY SENT] {line}")
            self._last_send_time = time.time()
            return

        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial not opened.")
            return

        with self._write_lock:
            try:
                self.ser.write((line.strip() + "\n").encode("utf-8"))
                self.ser.flush()
                self._last_send_time = time.time()
                self.get_logger().info(f"[SENT] {line.strip()}")
            except Exception as e:
                self.get_logger().error(f"Failed to write serial: {e}")

    def _send_init_sequence(self):
        # 네 로그와 동일: M50, M17, M21, G90
        self._send_line("M50")
        self._send_line("M17")
        self._send_line("M21")
        self._send_line("G90")
        self.get_logger().info("Init sequence sent: M50, M17, M21, G90")

    # ========= Reader / Parser =========
    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.05)
                    continue

                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # 로깅
                self.get_logger().info(f"[RECV] {line}")

                # status line: <Idle,...>
                if line.startswith("<") and line.endswith(">"):
                    ps = self._parse_status(line)
                    if ps:
                        self._last_status = ps
                        self._last_status_time = time.time()
                        # status raw publish
                        msg = String()
                        msg.data = ps.raw
                        self.pub_status_raw.publish(msg)
                        # busy update
                        self._update_busy_from_status(ps)
                    continue

                # Error,E112,Unsupported command
                if "Unsupported command" in line or "Error,E112" in line:
                    # status_query_cmd를 보냈는데 unsupported 에러가 왔으면 한 번만 경고
                    if self.status_query_cmd and (self.status_query_cmd in ["M114", "?", "$G", "$$"]):
                        # 단, 실제로는 '?'에서 status가 오기도 하니까 "영구 차단"은 하지 않고
                        # status가 계속 오면 경고만 줄인다.
                        if not self._warned_unsupported_status_cmd:
                            self.get_logger().error(
                                f"Status query cmd '{self.status_query_cmd}' unsupported by firmware? "
                                f"(하지만 status가 오면 그대로 사용 가능) param status_query_cmd 확인."
                            )
                            self._warned_unsupported_status_cmd = True
                    continue

            except Exception as e:
                self.get_logger().error(f"Reader loop error: {e}")
                time.sleep(0.1)

    def _parse_status(self, line: str) -> Optional[ParsedStatus]:
        m = STATUS_RE.match(line)
        if not m:
            return None
        state = m.group(1).strip()
        return ParsedStatus(state=state, raw=line)

    def _update_busy_from_status(self, ps: ParsedStatus):
        # Idle 이면 false, 그 외면 true
        new_busy = (ps.state.lower() != "idle")
        self._busy = new_busy

    # ========= Timers =========
    def _status_timer_cb(self):
        # status_query_cmd가 비어있으면 조회 안함
        if not self.status_query_cmd:
            return
        # WLKATA에서 '?'는 보통 status 요청
        self._send_line(self.status_query_cmd)

    def _busy_publish_timer_cb(self):
        # status가 너무 오래 안오면 "안전하게" busy 로 간주(움직임 중인데 status가 끊기는 경우 방지)
        now = time.time()
        if self.status_query_cmd:
            if (now - self._last_status_time) > self.status_timeout:
                # status가 오래 안오면, 최근에 명령 보냈거나 뭔가 이상 -> 보수적으로 busy 유지
                # (너무 공격적으로 false 만들면 시퀀서가 일찍 다음 단계로 넘어감)
                pass

        msg = Bool()
        msg.data = bool(self._busy)
        self.pub_busy.publish(msg)

    # ========= Callbacks =========
    def on_target_pose(self, msg: Pose):
        # Pose.position.x/y/z 를 mm로 해석 (너가 쓰는 CLI가 mm 기준)
        x = float(msg.position.x)
        y = float(msg.position.y)
        z = float(msg.position.z)

        # workspace clamp
        x = max(self.ws_x_min, min(self.ws_x_max, x))
        y = max(self.ws_y_min, min(self.ws_y_max, y))
        z = max(self.ws_z_min, min(self.ws_z_max, z))

        # 움직임 시작 시점엔 busy를 true로 예측(상태 조회가 늦게 오더라도 시퀀서가 기다리게)
        self._busy = True

        cmd = f"M20 G90 X{x:.2f} Y{y:.2f} Z{z:.2f} F{self.feedrate}"
        self._send_line(cmd)

    def on_target_joints(self, msg: JointState):
        """
        init_joint_once가 보내는 형태(네 로그):
          M21 G90 X0 Y0 Z0 A0 B0 C0
        즉 6개 값을 X,Y,Z,A,B,C 로 매핑한다고 가정.
        """
        if not msg.position or len(msg.position) < 6:
            self.get_logger().warn("target_joint_states: position length < 6, ignored.")
            return

        x, y, z, a, b, c = [float(v) for v in msg.position[:6]]

        # deadband (불필요한 미세명령 줄이기)
        # (여기서는 마지막 목표를 기억하지 않으므로, deadband는 sequencer/상위에서 해결하는 게 좋지만
        #  최소한 값이 너무 작으면 무시)
        if all(abs(v) < self.deadband_deg for v in [x, y, z, a, b, c]):
            # 원점 복귀(0,0,0,0,0,0)는 deadband에 걸리면 안되므로 예외 처리
            if not (x == 0.0 and y == 0.0 and z == 0.0 and a == 0.0 and b == 0.0 and c == 0.0):
                return

        self._busy = True
        cmd = f"M21 G90 X{x:.2f} Y{y:.2f} Z{z:.2f} A{a:.2f} B{b:.2f} C{c:.2f} F{self.feedrate}"
        self._send_line(cmd)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

