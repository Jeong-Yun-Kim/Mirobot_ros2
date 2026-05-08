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
    state: str
    raw: str

class SerialBridge(Node):
    """
    - /<ns>/target_pose_xyz (geometry_msgs/Pose)         -> M20 ...
    - /<ns>/target_joint_states (sensor_msgs/JointState) -> M21 ...
    - /<ns>/raw_cmd (std_msgs/String)                    -> 그대로 시리얼 전송 (!, ~, M3S1000 등)
    - /<ns>/is_busy (std_msgs/Bool)                      -> status 기반
    - /<ns>/status_raw (std_msgs/String)                 -> status 원문
    """

    def __init__(self):
        super().__init__("serial_bridge")

        # ===== Params =====
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("require_enable", False)
        self.declare_parameter("deadband_deg", 0.2)
        self.declare_parameter("min_send_period", 0.15)

        self.declare_parameter("workspace_x_min", 140.0)
        self.declare_parameter("workspace_x_max", 290.0)
        self.declare_parameter("workspace_y_min", -270.0)
        self.declare_parameter("workspace_y_max", 270.0)
        self.declare_parameter("workspace_z_min", 40.0)
        self.declare_parameter("workspace_z_max", 300.0)

        self.declare_parameter("feedrate", 2000)

        self.declare_parameter("status_query_cmd", "?")
        self.declare_parameter("status_query_period", 0.2)
        self.declare_parameter("status_timeout", 1.0)

        # topic names
        self.declare_parameter("topic_target_pose", "target_pose_xyz")
        self.declare_parameter("topic_target_joints", "target_joint_states")
        self.declare_parameter("topic_busy", "is_busy")
        self.declare_parameter("topic_status_raw", "status_raw")
        self.declare_parameter("topic_raw_cmd", "raw_cmd")  # ✅ 추가

        self.declare_parameter("joint_names", ["joint1","joint2","joint3","joint4","joint5","joint6"])

        # ===== Load params =====
        self.port: str = str(self.get_parameter("port").value)
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
        self.topic_busy = str(self.get_parameter("topic_busy").value)
        self.topic_status_raw = str(self.get_parameter("topic_status_raw").value)
        self.topic_raw_cmd = str(self.get_parameter("topic_raw_cmd").value)  # ✅ 추가

        self.joint_names = list(self.get_parameter("joint_names").value)

        # ===== QoS =====
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ===== Pubs/Subs =====
        self.pub_busy = self.create_publisher(Bool, self.topic_busy, qos)
        self.pub_status_raw = self.create_publisher(String, self.topic_status_raw, qos)

        self.create_subscription(Pose, self.topic_target_pose, self.on_target_pose, qos)
        self.create_subscription(JointState, self.topic_target_joints, self.on_target_joints, qos)

        # ✅ raw_cmd 구독: 그대로 전송
        self.create_subscription(String, self.topic_raw_cmd, self.on_raw_cmd, qos)

        # ===== State =====
        self.ser: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        self._busy = False
        self._last_send_time = 0.0

        self._last_status: Optional[ParsedStatus] = None
        self._last_status_time = 0.0

        self._warned_unsupported_status_cmd = False

        self._open_serial()

        # timers
        self.create_timer(self.status_query_period, self._status_timer_cb)
        self.create_timer(0.1, self._busy_publish_timer_cb)

        self.get_logger().info(
            f"Started. port={self.port} baud={self.baud} dry_run={self.dry_run} "
            f"workspace(mm)=X[{self.ws_x_min},{self.ws_x_max}] Y[{self.ws_y_min},{self.ws_y_max}] Z[{self.ws_z_min},{self.ws_z_max}]"
        )

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

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

        # init
        self._send_line("M50")
        self._send_line("M17")
        self._send_line("M21")
        self._send_line("G90")

    def _send_line(self, line: str):
        now = time.time()
        dt = now - self._last_send_time
        if dt < self.min_send_period:
            time.sleep(self.min_send_period - dt)

        if self.dry_run:
            self._last_send_time = time.time()
            return

        if not self.ser or not self.ser.is_open:
            return

        with self._write_lock:
            try:
                self.ser.write((line.strip() + "\n").encode("utf-8"))
                self.ser.flush()
                self._last_send_time = time.time()
            except Exception:
                pass

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

                # status line: <Idle,...>
                if line.startswith("<") and line.endswith(">"):
                    ps = self._parse_status(line)
                    if ps:
                        self._last_status = ps
                        self._last_status_time = time.time()

                        msg = String()
                        msg.data = ps.raw
                        self.pub_status_raw.publish(msg)

                        self._update_busy_from_status(ps)
                    continue

            except Exception:
                time.sleep(0.1)

    def _parse_status(self, line: str) -> Optional[ParsedStatus]:
        m = STATUS_RE.match(line)
        if not m:
            return None
        state = m.group(1).strip()
        return ParsedStatus(state=state, raw=line)

    def _update_busy_from_status(self, ps: ParsedStatus):
        self._busy = (ps.state.lower() != "idle")

    def _status_timer_cb(self):
        if not self.status_query_cmd:
            return
        self._send_line(self.status_query_cmd)

    def _busy_publish_timer_cb(self):
        # status가 오래 안오면 busy를 함부로 false로 만들지 않음
        msg = Bool()
        msg.data = bool(self._busy)
        self.pub_busy.publish(msg)

    # ===== Callbacks =====
    def on_raw_cmd(self, msg: String):
        # ✅ 그냥 보내기: "!" "~" "M3S1000" "G04 P0" 등
        cmd = (msg.data or "").strip()
        if not cmd:
            return
        self._busy = True
        self._send_line(cmd)

    def on_target_pose(self, msg: Pose):
        x = float(msg.position.x)
        y = float(msg.position.y)
        z = float(msg.position.z)

        x = max(self.ws_x_min, min(self.ws_x_max, x))
        y = max(self.ws_y_min, min(self.ws_y_max, y))
        z = max(self.ws_z_min, min(self.ws_z_max, z))

        self._busy = True
        cmd = f"M20 G90 X{x:.2f} Y{y:.2f} Z{z:.2f} F{self.feedrate}"
        self._send_line(cmd)

    def on_target_joints(self, msg: JointState):
        if not msg.position or len(msg.position) < 6:
            return

        x, y, z, a, b, c = [float(v) for v in msg.position[:6]]

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

