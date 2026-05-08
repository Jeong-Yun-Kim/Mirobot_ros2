#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose


class MrOrderBridge(Node):
    def __init__(self):
        super().__init__("mr_order_bridge")

        self.create_subscription(String, "/mr_move", self.on_move, 10)
        self.create_subscription(String, "/mr_estop", self.on_estop, 10)
        self.create_subscription(Bool, "/sequence_active", self.on_seq, 10)

        self.pub_pose = self.create_publisher(Pose, "/mirobot1/target_pose_xyz", 10)
        self.pub_done = self.create_publisher(String, "/mr_done", 10)

        # serial_bridge.py에 추가한 토픽
        self.pub_raw_master = self.create_publisher(String, "/mirobot1/raw_cmd", 10)
        self.pub_raw_slave = self.create_publisher(String, "/mirobot2/raw_cmd", 10)

        self.current_id = None
        self.estop = False
        self.active = False
        self.prev_active = False

    def _raw(self, s: str):
        m = String()
        m.data = s
        self.pub_raw_master.publish(m)
        self.pub_raw_slave.publish(m)

    def _trigger_master(self):
        p = Pose()
        p.position.x = 200.0
        p.position.y = 0.0
        p.position.z = 120.0
        p.orientation.w = 1.0
        self.pub_pose.publish(p)

    def on_estop(self, msg: String):
        t = (msg.data or "").strip().upper()

        if t == "ESTOP":
            self.estop = True
            self._raw("!")  
            return

        # START
        self.estop = False
        self._raw("~")      

        if self.current_id is not None and (not self.active):
            self._trigger_master()

    def on_move(self, msg: String):
        if self.estop:
            return
        if self.current_id is not None:
            return

        data = json.loads(msg.data)
        self.current_id = int(data["id"])
        self._trigger_master()

    def on_seq(self, msg: Bool):
        self.active = bool(msg.data)

        if self.prev_active and (not self.active) and (self.current_id is not None):
            out = String()
            out.data = json.dumps({"id": self.current_id}, ensure_ascii=False)
            self.pub_done.publish(out)
            self.current_id = None

        self.prev_active = self.active


def main():
    rclpy.init()
    node = MrOrderBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
