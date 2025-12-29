#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

class MirobotJointCLI(Node):
    def __init__(self):
        super().__init__("mirobot_joint_cli")
        self.pub = self.create_publisher(JointState, "/target_joint_states", 10)

    def run(self):
        print("\n=== Mirobot Joint CLI ===")
        print(" - 6개 조인트 값을 입력하면 /target_joint_states로 publish합니다.")
        print(" - 'b' 또는 'q' 입력하면 종료합니다.\n")

        unit = input("단위 선택 (rad/deg) [기본 rad]: ").strip().lower()
        if unit not in ["", "rad", "deg"]:
            print("단위가 이상해서 기본(rad)로 진행할게요.")
            unit = "rad"
        if unit == "":
            unit = "rad"

        print("\n입력 예시:")
        if unit == "rad":
            print("  0  -0.5  0.5  0  0.3  0")
        else:
            print("  0  -30   30   0  15   0")

        while rclpy.ok():
            s = input("\n[j1 j2 j3 j4 j5 j6] 입력: ").strip().lower()
            if s in ["q", "quit", "exit", "b"]:
                print("종료합니다.")
                return

            parts = s.replace(",", " ").split()
            if len(parts) != 6:
                print("❗ 6개 숫자를 입력해야 해요. 예: 0 0 0 0 0 0")
                continue

            try:
                vals = [float(x) for x in parts]
            except ValueError:
                print("❗ 숫자만 입력해줘요.")
                continue

            if unit == "deg":
                vals = [v * math.pi / 180.0 for v in vals]

            msg = JointState()
            msg.name = JOINT_NAMES
            msg.position = vals

            self.pub.publish(msg)
            self.get_logger().info(f"Published /target_joint_states (rad): {vals}")

def main():
    rclpy.init()
    node = MirobotJointCLI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

