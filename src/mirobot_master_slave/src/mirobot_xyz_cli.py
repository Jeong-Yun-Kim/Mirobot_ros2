#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# 안전 작업 범위(mm)
X_MIN, X_MAX = 140.0, 290.0
Y_MIN, Y_MAX = -270.0, 270.0
Z_MIN, Z_MAX = 40.0, 300.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class MirobotXYZCLI(Node):
    def __init__(self):
        super().__init__("mirobot_xyz_cli")
        # 마스터에게 명령
        self.pub = self.create_publisher(Pose, "/mirobot1/target_pose_xyz", 10)

    def run(self):
        self.get_logger().info("입력: x y z (mm). 예) 200 0 120   | 종료: q")
        while rclpy.ok():
            s = input("XYZ> ").strip()
            if s.lower() in ("q", "quit", "exit"):
                break
            parts = s.split()
            if len(parts) != 3:
                print("형식: x y z")
                continue
            try:
                x, y, z = map(float, parts)
            except ValueError:
                print("숫자만 입력")
                continue

            x = clamp(x, X_MIN, X_MAX)
            y = clamp(y, Y_MIN, Y_MAX)
            z = clamp(z, Z_MIN, Z_MAX)

            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            msg.orientation.w = 1.0  # 기본

            self.pub.publish(msg)
            self.get_logger().info(f"Publish /mirobot1/target_pose_xyz = ({x:.1f}, {y:.1f}, {z:.1f})")

def main():
    rclpy.init()
    node = MirobotXYZCLI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

