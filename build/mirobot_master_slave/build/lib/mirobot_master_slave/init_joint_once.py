#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class InitJointOnce(Node):
    def __init__(self):
        super().__init__("init_joint_once")
        self.declare_parameter("topic", "/mirobot1/target_joint_states")
        self.declare_parameter("delay_sec", 2.0)
        self.declare_parameter("names", ["joint1","joint2","joint3","joint4","joint5","joint6"])
        self.declare_parameter("positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(JointState, self.topic, 10)

        delay = float(self.get_parameter("delay_sec").value)
        self.get_logger().info(f"Will publish init joints to {self.topic} after {delay:.2f}s")
        self.timer = self.create_timer(delay, self._fire)

    def _fire(self):
        msg = JointState()
        msg.name = list(self.get_parameter("names").value)
        msg.position = [float(x) for x in self.get_parameter("positions").value]
        self.pub.publish(msg)
        self.get_logger().info("Init joint published. Exiting.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitJointOnce()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

