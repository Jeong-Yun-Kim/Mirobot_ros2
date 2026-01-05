#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class MirobotSlaveReplicator(Node):
    def __init__(self):
        super().__init__("mirobot_slave_replicator")

        self.sub = self.create_subscription(
            Pose,
            "/mirobot1/target_pose_xyz",
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            Pose,
            "/mirobot2/target_pose_xyz",
            10
        )

        self.get_logger().info("Replicator ON: /mirobot1/target_pose_xyz -> /mirobot2/target_pose_xyz")

    def cb(self, msg: Pose):
        self.pub.publish(msg)
        self.get_logger().info(
            f"Replicated to mirobot2: ({msg.position.x:.1f}, {msg.position.y:.1f}, {msg.position.z:.1f})"
        )

def main():
    rclpy.init()
    node = MirobotSlaveReplicator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

