#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# 네가 준 작업영역(mm)
X_MIN, X_MAX = 140.0, 290.0
Y_MIN, Y_MAX = -270.0, 270.0
Z_MIN, Z_MAX = 40.0, 300.0

def in_range(v, lo, hi):
    return lo <= v <= hi

class MirobotXYZCLI(Node):
    def __init__(self):
        super().__init__("mirobot_xyz_cli")
        self.pub = self.create_publisher(Pose, "/target_pose_xyz", 10)

    def run(self):
        print("\n=== Mirobot XYZ CLI ===")
        print(f"Workspace(mm): X[{X_MIN},{X_MAX}] Y[{Y_MIN},{Y_MAX}] Z[{Z_MIN},{Z_MAX}]")
        print("입력 예: 200 0 150")
        print("q 입력하면 종료\n")

        while rclpy.ok():
            s = input("X Y Z(mm) 입력: ").strip().lower()
            if s in ["q", "quit", "exit"]:
                break

            parts = s.replace(",", " ").split()
            if len(parts) != 3:
                print("❗ 3개 숫자를 입력해줘요. 예: 200 0 150")
                continue

            try:
                x, y, z = [float(p) for p in parts]
            except ValueError:
                print("❗ 숫자만 입력해줘요.")
                continue

            if not (in_range(x, X_MIN, X_MAX) and in_range(y, Y_MIN, Y_MAX) and in_range(z, Z_MIN, Z_MAX)):
                print("⚠️ 범위를 벗어났어요. 그래도 보내면 드라이버가 자동 clamp합니다.")
                print(f"   요청: ({x}, {y}, {z})")

            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            self.pub.publish(msg)
            self.get_logger().info(f"Published /target_pose_xyz: ({x}, {y}, {z})")

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

