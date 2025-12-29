#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointRelay(Node):
    def __init__(self):
        super().__init__('joint_relay')
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.pub = self.create_publisher(JointState, '/target_joint_states', 10)

    def cb(self, msg: JointState):
        # 그대로 전달
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

