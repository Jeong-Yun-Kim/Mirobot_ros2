from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimulateOrderNode(Node):
    def __init__(self, topic: str):
        super().__init__("simulate_mirobot_order")
        self.pub = self.create_publisher(String, topic, 10)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Publish one test /robot/cmd order")
    parser.add_argument("--topic", default="/robot/cmd")
    parser.add_argument("--id", type=int, default=999)
    parser.add_argument("--red", type=int, default=1)
    parser.add_argument("--green", type=int, default=0)
    parser.add_argument("--blue", type=int, default=0)
    parser.add_argument("--task", default="fill", choices=["fill", "pack"])
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = SimulateOrderNode(args.topic)
    msg = String()
    msg.data = json.dumps({"task": args.task, "id": args.id, "red": args.red, "green": args.green, "blue": args.blue}, ensure_ascii=False)
    # Give discovery a short moment.
    deadline = time.time() + 0.8
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.pub.publish(msg)
    node.get_logger().info(f"published {args.topic}: {msg.data}")
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
