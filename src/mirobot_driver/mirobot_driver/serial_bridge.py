#!/usr/bin/env python3
# mirobot_driver/serial_bridge.py

import math
import time
import threading
from typing import List, Optional

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose  # ✅ XYZ 제어용


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SerialBridge(Node):
    """
    ROS2 -> WLKATA Mirobot serial bridge

    Sub:
      - /target_joint_states (sensor_msgs/JointState) : command target (rad)
      - /target_pose_xyz    (geometry_msgs/Pose)      : cartesian XYZ target (mm)
      - /mirobot/enable (Bool) : enable/disable (optional)
      - /mirobot/home   (Bool) : home (optional)

    Pub:
      - /joint_states (sensor_msgs/JointState) : last commanded (for rviz preview)
    """

    def __init__(self):
        super().__init__("mirobot_driver")

        # ---------- Publishers ----------
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ---------- Parameters ----------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout", 0.2)

        # 기본은 False 권장 (진짜 로봇 움직일 거면 False)
        self.declare_parameter("dry_run", False)

        # joint name order expected by driver
        self.declare_parameter("joint_order", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        # deg limits (edit to match your hardware if needed)
        self.declare_parameter("joint_limits_deg_low",  [-170, -120, -170, -190, -120, -360])
        self.declare_parameter("joint_limits_deg_high", [ 170,  120,  170,  190,  120,  360])

        # feedrate for command (device-specific)
        self.declare_parameter("feedrate", 2000.0)

        # 떨림 방지: 이 정도 이하 변화는 전송 안 함 (deg)
        self.declare_parameter("deadband_deg", 0.20)

        # 전송 주기 제한 (초). 예: 0.15 => 최대 약 6~7Hz 정도로만 보냄
        self.declare_parameter("min_send_period_sec", 0.15)

        # enable/home 게이트를 사용할지
        self.declare_parameter("require_enable", False)

        # ✅ XYZ workspace limits (mm) - 네가 준 범위 기본값으로 설정
        self.declare_parameter("ws_x_min", 140.0)
        self.declare_parameter("ws_x_max", 290.0)
        self.declare_parameter("ws_y_min", -270.0)
        self.declare_parameter("ws_y_max", 270.0)
        self.declare_parameter("ws_z_min", 40.0)
        self.declare_parameter("ws_z_max", 300.0)

        # ---------- Read params ----------
        self.port: str = self.get_parameter("port").value
        self.baud: int = int(self.get_parameter("baud").value)
        self.timeout: float = float(self.get_parameter("timeout").value)
        self.dry_run: bool = bool(self.get_parameter("dry_run").value)

        self.joint_order: List[str] = list(self.get_parameter("joint_order").value)
        self.lim_lo: List[float] = list(self.get_parameter("joint_limits_deg_low").value)
        self.lim_hi: List[float] = list(self.get_parameter("joint_limits_deg_high").value)

        self.feedrate: float = float(self.get_parameter("feedrate").value)
        self.deadband_deg: float = float(self.get_parameter("deadband_deg").value)
        self.min_send_period_sec: float = float(self.get_parameter("min_send_period_sec").value)

        self.require_enable: bool = bool(self.get_parameter("require_enable").value)
        self.enabled: bool = not self.require_enable  # require_enable이면 기본 false

        # workspace
        self.ws_x_min = float(self.get_parameter("ws_x_min").value)
        self.ws_x_max = float(self.get_parameter("ws_x_max").value)
        self.ws_y_min = float(self.get_parameter("ws_y_min").value)
        self.ws_y_max = float(self.get_parameter("ws_y_max").value)
        self.ws_z_min = float(self.get_parameter("ws_z_min").value)
        self.ws_z_max = float(self.get_parameter("ws_z_max").value)

        # ---------- Serial ----------
        self.ser: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()

        self._last_sent_deg: Optional[List[float]] = None
        self._last_send_time = 0.0

        self._open_serial_and_init()

        # ---------- Subscriptions ----------
        self.cmd_sub = self.create_subscription(JointState, "/target_joint_states", self.on_cmd, 10)
        self.pose_sub = self.create_subscription(Pose, "/target_pose_xyz", self.on_pose_xyz, 10)  # ✅ XYZ
        self.enable_sub = self.create_subscription(Bool, "/mirobot/enable", self.on_enable, 1)
        self.home_sub = self.create_subscription(Bool, "/mirobot/home", self.on_home, 1)

        self.get_logger().info(
            f"Started. dry_run={self.dry_run}, require_enable={self.require_enable}, "
            f"deadband_deg={self.deadband_deg}, min_send_period={self.min_send_period_sec}s, "
            f"workspace(mm)=X[{self.ws_x_min},{self.ws_x_max}] "
            f"Y[{self.ws_y_min},{self.ws_y_max}] Z[{self.ws_z_min},{self.ws_z_max}]"
        )

    # ---------------- Serial helpers ----------------
    def _open_serial_and_init(self):
        if self.dry_run:
            self.get_logger().warn("dry_run=True (serial will NOT be used). Set dry_run=False to move real robot.")
            return

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            # ---- Init sequence ----
            # M50: unlock, M17: enable steppers, M21: angle(joint) mode, G90: absolute
            self._send_line("M50")
            time.sleep(0.05)
            self._send_line("M17")
            time.sleep(0.05)
            self._send_line("M21")
            time.sleep(0.05)
            self._send_line("G90")
            time.sleep(0.05)

            self.get_logger().info("Init sequence sent: M50, M17, M21, G90")

        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Failed to open serial: {e}")

    def _send_line(self, line: str):
        if self.dry_run:
            self.get_logger().info(f"[DRY] {line}")
            return

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial not open")
            return

        with self.serial_lock:
            self.ser.write((line + "\r\n").encode("ascii"))

        self.get_logger().info(f"[SENT] {line}")

    # ---------------- Command formatting ----------------
    def _format_joint_command(self, q_deg: List[float]) -> str:
        # Angle mode (M21): X,Y,Z,A,B,C correspond to J1..J6
        return (
            f"M21 G90 "
            f"X{q_deg[0]:.2f} Y{q_deg[1]:.2f} Z{q_deg[2]:.2f} "
            f"A{q_deg[3]:.2f} B{q_deg[4]:.2f} C{q_deg[5]:.2f} "
            f"F{self.feedrate:.0f}"
        )

    def _format_xyz_command(self, x_mm: float, y_mm: float, z_mm: float) -> str:
        # Cartesian mode (M20): X,Y,Z = Cartesian target
        return f"M20 G90 X{x_mm:.2f} Y{y_mm:.2f} Z{z_mm:.2f} F{self.feedrate:.0f}"

    # ---------------- ROS callbacks ----------------
    def on_enable(self, msg: Bool):
        if not self.require_enable:
            # require_enable을 안 써도 사용자가 enable 쏘면 모터 on/off는 해줌
            self._send_line("M17" if msg.data else "M18")
            return

        self.enabled = bool(msg.data)
        if self.enabled:
            self._send_line("M17")
            self.get_logger().info("Enabled = True")
        else:
            self._send_line("M18")
            self.get_logger().warn("Enabled = False (commands will be ignored)")

    def on_home(self, msg: Bool):
        if not msg.data:
            return
        self._send_line("G28")  # 펌웨어에 따라 다를 수 있음(필요시 변경)

    def on_cmd(self, msg: JointState):
        if self.require_enable and not self.enabled:
            return

        if msg.position is None or len(msg.position) < 6:
            self.get_logger().warn("target_joint_states has <6 positions; ignored")
            return

        # map by name if possible
        q_rad = [0.0] * 6
        if msg.name and len(msg.name) == len(msg.position):
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            missing = [jn for jn in self.joint_order if jn not in name_to_pos]
            if missing:
                self.get_logger().warn(f"Missing joints in command name[]: {missing}. Fill with 0.0 rad.")
            for i, jn in enumerate(self.joint_order):
                q_rad[i] = float(name_to_pos.get(jn, 0.0))
        else:
            self.get_logger().warn("target_joint_states.name is empty; using first 6 positions as joint1..6")
            q_rad = [float(p) for p in msg.position[:6]]

        # rad -> deg, clamp by joint limits
        q_deg = []
        for i in range(6):
            deg = q_rad[i] * 180.0 / math.pi
            deg = clamp(deg, self.lim_lo[i], self.lim_hi[i])
            q_deg.append(deg)

        # deadband / min period
        now = time.time()

        if self._last_sent_deg is not None:
            diffs = [abs(q_deg[i] - self._last_sent_deg[i]) for i in range(6)]
            if max(diffs) < self.deadband_deg:
                self._publish_state_from_deg(q_deg)
                return

        if (now - self._last_send_time) < self.min_send_period_sec:
            self._publish_state_from_deg(q_deg)
            return

        cmd = self._format_joint_command(q_deg)
        if self.ser is None or (not self.dry_run and not self.ser.is_open):
            self.get_logger().error("Serial not open; cannot send command")
            return

        self._send_line(cmd)
        self._last_sent_deg = q_deg
        self._last_send_time = now
        self._publish_state_from_deg(q_deg)

    def on_pose_xyz(self, msg: Pose):
        """XYZ(mm) target -> send M20 ... with workspace clamp."""
        if self.require_enable and not self.enabled:
            return

        x = float(msg.position.x)
        y = float(msg.position.y)
        z = float(msg.position.z)

        x_c = clamp(x, self.ws_x_min, self.ws_x_max)
        y_c = clamp(y, self.ws_y_min, self.ws_y_max)
        z_c = clamp(z, self.ws_z_min, self.ws_z_max)

        if (x_c, y_c, z_c) != (x, y, z):
            self.get_logger().warn(
                f"XYZ out of workspace. requested=({x:.1f},{y:.1f},{z:.1f}) "
                f"clamped=({x_c:.1f},{y_c:.1f},{z_c:.1f})"
            )

        cmd = self._format_xyz_command(x_c, y_c, z_c)
        self._send_line(cmd)

        # ⚠️ 주의:
        # 이 드라이버는 현재 “로봇에서 실제 joint feedback을 읽어오는 구조”가 아니라서
        # XYZ 명령을 보내도 RViz가 정확히 따라 움직이진 않을 수 있어(조인트값을 모르기 때문).
        # RViz에서 XYZ 이동을 정확히 애니메이션하려면 IK(역기구학)로 joint를 계산해 publish하거나,
        # 로봇에서 상태를 읽어오는 피드백이 필요함.

    def _publish_state_from_deg(self, q_deg: List[float]):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_order)
        js.position = [deg * math.pi / 180.0 for deg in q_deg]
        self.state_pub.publish(js)


def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

