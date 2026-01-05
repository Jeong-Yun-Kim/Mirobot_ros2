#!/usr/bin/env python3
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


class State(Enum):
    IDLE = 0
    WAIT_MASTER_BUSY = 1
    WAIT_MASTER_DONE = 2
    WAIT_SLAVE_DONE = 3
    WAIT_SLAVE_HOME_DONE = 4
    WAIT_MASTER_HOME_DONE = 5


class RoundTripSequencer(Node):
    """
    흐름:
      (1) 마스터 목표 Pose를 감지(/mirobot1/target_pose_xyz)
      (2) 마스터가 Busy->Idle 될 때까지 기다림
      (3) 슬레이브에 같은 Pose publish (/mirobot2/target_pose_xyz)
      (4) 슬레이브 Busy->Idle 기다림
      (5) 슬레이브 원위치(조인트 0) publish (/mirobot2/target_joint_states)
      (6) 슬레이브 Busy->Idle 기다림
      (7) 마스터 원위치(조인트 0) publish (/mirobot1/target_joint_states)
      (8) 마스터 Busy->Idle 기다림
    + (추가) /sequence_active (Bool) publish:
        시퀀스 진행 중 True, IDLE 복귀 시 False
    """

    def __init__(self):
        super().__init__("mirobot_roundtrip_sequencer")

        self.declare_parameter("master_pose_topic", "/mirobot1/target_pose_xyz")
        self.declare_parameter("slave_pose_topic", "/mirobot2/target_pose_xyz")
        self.declare_parameter("master_busy_topic", "/mirobot1/is_busy")
        self.declare_parameter("slave_busy_topic", "/mirobot2/is_busy")
        self.declare_parameter("master_home_topic", "/mirobot1/target_joint_states")
        self.declare_parameter("slave_home_topic", "/mirobot2/target_joint_states")

        self.declare_parameter("sequence_active_topic", "/sequence_active")

        self.declare_parameter("home_joint_positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("settle_time", 0.4)  # Idle 판단 후 안정화 대기

        self.master_pose_topic = self.get_parameter("master_pose_topic").value
        self.slave_pose_topic = self.get_parameter("slave_pose_topic").value
        self.master_busy_topic = self.get_parameter("master_busy_topic").value
        self.slave_busy_topic = self.get_parameter("slave_busy_topic").value
        self.master_home_topic = self.get_parameter("master_home_topic").value
        self.slave_home_topic = self.get_parameter("slave_home_topic").value
        self.sequence_active_topic = self.get_parameter("sequence_active_topic").value

        self.home_pos = list(self.get_parameter("home_joint_positions").value)
        self.home_names = list(self.get_parameter("home_joint_names").value)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.settle_time = float(self.get_parameter("settle_time").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # pubs
        self.pub_slave_pose = self.create_publisher(Pose, self.slave_pose_topic, qos)
        self.pub_slave_home = self.create_publisher(JointState, self.slave_home_topic, qos)
        self.pub_master_home = self.create_publisher(JointState, self.master_home_topic, qos)
        self.pub_seq_active = self.create_publisher(Bool, self.sequence_active_topic, qos)

        # subs
        self.create_subscription(Pose, self.master_pose_topic, self.on_master_cmd, qos)
        self.create_subscription(Bool, self.master_busy_topic, self.on_master_busy, qos)
        self.create_subscription(Bool, self.slave_busy_topic, self.on_slave_busy, qos)

        self.state = State.IDLE
        self.master_busy = False
        self.slave_busy = False

        self._target_pose: Pose = None
        self._t_state_enter = time.time()
        self._saw_master_busy_edge = False
        self._saw_slave_busy_edge = False

        # sequence_active 관리용
        self._sequence_active = False
        self._publish_sequence_active(False)  # 시작은 False

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            "RoundTripSequencer started\n"
            f" master_pose_topic={self.master_pose_topic}\n"
            f" slave_pose_topic={self.slave_pose_topic}\n"
            f" master_busy_topic={self.master_busy_topic}\n"
            f" slave_busy_topic={self.slave_busy_topic}\n"
            f" slave_home_topic={self.slave_home_topic}\n"
            f" master_home_topic={self.master_home_topic}\n"
            f" sequence_active_topic={self.sequence_active_topic}\n"
            f" timeout={self.timeout_sec}s, settle_time={self.settle_time}s"
        )

    def _publish_sequence_active(self, active: bool):
        # 중복 publish 줄이기(엣지 기반)
        if self._sequence_active == active:
            return
        self._sequence_active = active
        msg = Bool()
        msg.data = bool(active)
        self.pub_seq_active.publish(msg)
        self.get_logger().info(f"[SEQ_ACTIVE] {active}")

    def _enter(self, st: State):
        prev = self.state
        self.state = st
        self._t_state_enter = time.time()

        # IDLE <-> active 상태 전환 시 /sequence_active publish
        if prev == State.IDLE and st != State.IDLE:
            self._publish_sequence_active(True)
        if st == State.IDLE and prev != State.IDLE:
            self._publish_sequence_active(False)

    def _elapsed(self) -> float:
        return time.time() - self._t_state_enter

    def on_master_cmd(self, msg: Pose):
        # 마스터 CLI가 publish한 목표 Pose를 감지
        self._target_pose = msg

        # 새 명령이 들어오면 시퀀스 시작
        self._saw_master_busy_edge = False
        self._saw_slave_busy_edge = False

        self.get_logger().info(
            f"[MASTER CMD] X={msg.position.x:.1f}, Y={msg.position.y:.1f}, Z={msg.position.z:.1f} -> wait master done"
        )
        self._enter(State.WAIT_MASTER_BUSY)

    def on_master_busy(self, msg: Bool):
        self.master_busy = bool(msg.data)

    def on_slave_busy(self, msg: Bool):
        self.slave_busy = bool(msg.data)

    def _publish_slave_pose(self):
        if self._target_pose is None:
            return
        self.pub_slave_pose.publish(self._target_pose)
        self.get_logger().info("[SLAVE START] publish slave target_pose_xyz")

    def _publish_slave_home(self):
        js = JointState()
        js.name = self.home_names
        js.position = self.home_pos
        self.pub_slave_home.publish(js)
        self.get_logger().info("[SLAVE HOME] publish slave target_joint_states (home)")

    def _publish_master_home(self):
        js = JointState()
        js.name = self.home_names
        js.position = self.home_pos
        self.pub_master_home.publish(js)
        self.get_logger().info("[MASTER HOME] publish master target_joint_states (home)")

    def _force_end(self, reason: str):
        self.get_logger().warn(f"[FORCE END] {reason}")
        self._enter(State.IDLE)

    def tick(self):
        # 공통 timeout 처리
        if self.state != State.IDLE and self._elapsed() > self.timeout_sec:
            self.get_logger().warn(
                f"[TIMEOUT] state={self.state.name}, elapsed={self._elapsed():.1f}s -> force advance"
            )
            if self.state in (State.WAIT_MASTER_BUSY, State.WAIT_MASTER_DONE):
                self.get_logger().warn("[FORCE] master done timeout -> start slave anyway")
                self._publish_slave_pose()
                self._enter(State.WAIT_SLAVE_DONE)
            elif self.state == State.WAIT_SLAVE_DONE:
                self._publish_slave_home()
                self._enter(State.WAIT_SLAVE_HOME_DONE)
            elif self.state == State.WAIT_SLAVE_HOME_DONE:
                self._publish_master_home()
                self._enter(State.WAIT_MASTER_HOME_DONE)
            elif self.state == State.WAIT_MASTER_HOME_DONE:
                self._force_end("master home done timeout")
            return

        # ===== 상태 머신 =====
        if self.state == State.IDLE:
            return

        if self.state == State.WAIT_MASTER_BUSY:
            if self.master_busy:
                self._saw_master_busy_edge = True
                self._enter(State.WAIT_MASTER_DONE)
                return

            if self._elapsed() > 0.8:
                self.get_logger().warn("[MASTER BUSY ASSUME] no busy edge -> assume busy after 0.80s")
                self._saw_master_busy_edge = True
                self._enter(State.WAIT_MASTER_DONE)
                return

        elif self.state == State.WAIT_MASTER_DONE:
            if self._saw_master_busy_edge and (not self.master_busy):
                time.sleep(self.settle_time)
                self._publish_slave_pose()
                self._enter(State.WAIT_SLAVE_DONE)
                return

        elif self.state == State.WAIT_SLAVE_DONE:
            if self.slave_busy:
                self._saw_slave_busy_edge = True

            if self._saw_slave_busy_edge and (not self.slave_busy):
                time.sleep(self.settle_time)
                self._publish_slave_home()
                self._enter(State.WAIT_SLAVE_HOME_DONE)
                return

            if (not self._saw_slave_busy_edge) and self._elapsed() > 0.8:
                self.get_logger().warn("[SLAVE BUSY ASSUME] no busy edge -> assume busy after 0.80s")
                self._saw_slave_busy_edge = True

        elif self.state == State.WAIT_SLAVE_HOME_DONE:
            if not self.slave_busy:
                if self._elapsed() < 0.8:
                    return
                time.sleep(self.settle_time)
                self._publish_master_home()
                self._enter(State.WAIT_MASTER_HOME_DONE)
                return

        elif self.state == State.WAIT_MASTER_HOME_DONE:
            if not self.master_busy:
                if self._elapsed() < 0.8:
                    return
                time.sleep(self.settle_time)
                self.get_logger().info("[DONE] roundtrip sequence complete -> IDLE")
                self._enter(State.IDLE)
                return


def main():
    rclpy.init()
    node = RoundTripSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

