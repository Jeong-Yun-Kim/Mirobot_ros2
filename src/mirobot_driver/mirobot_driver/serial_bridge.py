# mirobot_driver/serial_bridge.py
import math
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class SerialBridge(Node):
    def __init__(self):
        super().__init__('mirobot_serial_bridge')
# 상태 퍼블리셔 (RViz / robot_state_publisher 용)
        from sensor_msgs.msg import JointState
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        # ---------- 파라미터 ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('dry_run', True)  # 먼저 True로 확인 후 False로!
        # 라디안 입력 기준, 내부에서 도(deg)로 변환
        self.declare_parameter('joint_order', ['joint1','joint2','joint3','joint4','joint5','joint6'])
        # 각도 제한(도): 반드시 실제 장비 스펙에 맞게 수정!
        self.declare_parameter('joint_limits_deg_low',  [-170, -120, -170, -190, -120, -360])
        self.declare_parameter('joint_limits_deg_high', [ 170,  120,  170,  190,  120,  360])
        # 이동 속도 등(장비 포맷에 맞게 활용)
        self.declare_parameter('feedrate', 2000.0)  # 예: G0 F(속도)
        # 프로토콜 선택: 'gcode_example' 또는 'raw'
        self.declare_parameter('protocol', 'gcode_example')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        self.joint_order = list(self.get_parameter('joint_order').value)
        self.lim_lo = list(self.get_parameter('joint_limits_deg_low').value)
        self.lim_hi = list(self.get_parameter('joint_limits_deg_high').value)
        self.feedrate = float(self.get_parameter('feedrate').value)
        self.protocol = self.get_parameter('protocol').get_parameter_value().string_value

        # ---------- 시리얼 ----------
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
            time.sleep(0.2)
            self.ser.write(("M50\r\n").encode("ascii"))
            self.get_logger().info("[SENT] M50")
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')

        # ---------- 구독 ----------
        # 권장: 상태(/joint_states)와 명령(/target_joint_states)을 분리
        self.cmd_sub = self.create_subscription(JointState, '/target_joint_states', self.on_cmd, 10)

        # enable / home 트리거(선택)
        self.enable_sub = self.create_subscription(Bool, '/mirobot/enable', self.on_enable, 1)
        self.home_sub   = self.create_subscription(Bool, '/mirobot/home',   self.on_home,   1)

        self.get_logger().info(f'dry_run={self.dry_run}, protocol={self.protocol}')

    # -------------- 명령 콜백 --------------
    def on_cmd(self, msg: JointState):
        # 1) msg.position 라디안 → 도
        #    입력 joint_names가 있으면 order 맞춤, 없으면 길이만 체크
        if msg.position is None or len(msg.position) < 6:
            self.get_logger().warn('command has <6 joints; ignored')
            return

        # name이 채워져 있으면 이름 기준으로 매핑
        if msg.name and len(msg.name) == len(msg.position):
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            positions_deg = []
            for name in self.joint_order:
                rad = name_to_pos.get(name, 0.0)
                positions_deg.append(rad * 180.0 / math.pi)
        else:
            # 이름이 없으면 그냥 앞 6개 사용
            positions_deg = [p * 180.0 / math.pi for p in msg.position[:6]]

        # 2) 리밋 적용
        for i in range(6):
            positions_deg[i] = clamp(positions_deg[i], self.lim_lo[i], self.lim_hi[i])

        # 3) 명령 문자열 생성
        cmd = self.format_command(positions_deg)

        # 4) 전송 or 프린트
        if self.dry_run or self.ser is None or not self.ser.is_open:
            self.get_logger().info(f'[DRY] {cmd}')
        else:
            try:
                self.ser.write((cmd + '\r\n').encode('ascii'))
                # 필요 시 OK 응답 읽기: resp = self.ser.readline().decode(errors='ignore').strip()
                # self.get_logger().info(f'>> {cmd}  resp={resp}')
            except Exception as e:
                self.get_logger().error(f'serial write failed: {e}')
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_order
        js.position = [deg * math.pi / 180.0 for deg in positions_deg]
        self.state_pub.publish(js)     

    # ----------- 포맷터: 여기만 장비 포맷에 맞게 바꾸면 됨 -----------
    def format_command(self, q_deg):
        """
        q_deg: list[6] of joint angles in degrees (J1..J6)
        Returns: Mirobot-compatible command line (without line ending)
        Example: M50 G0 X10.00 Y15.00 Z0.00 A0.00 B0.00 C0.00 F3000
    """
        cmd = (
            f"M50 G0 "
            f"X{q_deg[0]:.2f} Y{q_deg[1]:.2f} Z{q_deg[2]:.2f} "
            f"A{q_deg[3]:.2f} B{q_deg[4]:.2f} C{q_deg[5]:.2f} "
            f"F{self.feedrate:.0f}"
        )
        return cmd

    # -------- enable/home 간단 트리거(원하면 사용) --------
    def on_enable(self, msg: Bool):
        if msg.data:
            self._send_line("M17")  # 예: stepper enable (장비마다 다름)
        else:
            self._send_line("M18")  # 예: stepper disable

    def on_home(self, msg: Bool):
        if msg.data:
            self._send_line("G28")  # 예: home all axes

    def _send_line(self, line: str):
        if self.dry_run or self.ser is None or not self.ser.is_open:
            self.get_logger().info(f'[DRY] {line}')
            return
        try:
            self.ser.write((line + '\r\n').encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'serial write failed: {e}')

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
