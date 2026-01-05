#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import copy
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


def _now() -> float:
    return time.time()


class StopDetector:
    """
    joint_states 기반 정지 판정기

    - position이 들어올 때마다 속도(|dq|/dt)와 변위(|dq|)를 계산
    - 속도와 변위가 임계값 아래로 '연속' settle_time 동안 유지되면 stopped=True
    """
    def __init__(self, vel_eps: float, delta_eps: float, settle_time: float):
        self.vel_eps = float(vel_eps)         # rad/s
        self.delta_eps = float(delta_eps)     # rad
        self.settle_time = float(settle_time)

        self._last_pos: Optional[List[float]] = None
        self._last_t: Optional[float] = None

        self._still_since: Optional[float] = None
        self._moving_seen: bool = False

        self.last_msg_time: Optional[float] = None

        # 디버깅용
        self.last_speed = 0.0
        self.last_delta = 0.0

    def reset_for_new_goal(self):
        self._still_since = None
        self._moving_seen = False
        # last_pos/last_t는 유지(센서 continuity)해도 되고, 리셋해도 되지만
        # 리셋하면 첫 샘플에서 속도 계산이 안 되므로 유지하는 게 보통 더 안정적임.

    def update(self, msg: JointState):
        t = _now()
        self.last_msg_time = t

        pos = list(msg.position) if msg.position is not None else []
        if len(pos) == 0:
            return

        if self._last_pos is None or self._last_t is None or len(self._last_pos) != len(pos):
            self._last_pos = pos
            self._last_t = t
            self._still_since = None
            self.last_speed = 0.0
            self.last_delta = 0.0
            return

        dt = max(1e-3, (t - self._last_t))
        # L2 norm of delta
        delta_sq = 0.0
        for a, b in zip(pos, self._last_pos):
            d = (a - b)
            delta_sq += d * d
        delta = delta_sq ** 0.5
        speed = delta / dt

        self.last_delta = float(delta)
        self.last_speed = float(speed)

        # moving/still 판단
        moving = (speed > self.vel_eps) or (delta > self.delta_eps)

        if moving:
            self._moving_seen = True
            self._still_since = None
        else:
            if self._still_since is None:
                self._still_since = t

        self._last_pos = pos
        self._last_t = t

    def moving_seen(self) -> bool:
        return self._moving_seen

    def stopped_stable(self) -> bool:
        if self._still_since is None:
            return False
        return (_now() - self._still_since) >= self.settle_time


class MirobotSequential(Node):
    """
    목표가 /mirobot1/target_pose_xyz 로 들어오면,
      1) 마스터가 목표로 이동 완료(정지 판정) -> 슬레이브에 동일 목표 publish
      2) 슬레이브 이동 완료(정지 판정) ->
         - enable_round_trip=True이면: 슬레이브를 home으로, 그 다음 마스터를 home으로
         - 아니면 종료(IDLE)

    정지 판정은:
      - is_idle 토픽이 정상적으로 변하면(is_idle 사용)
      - is_idle이 불안정하면 joint_states 기반 StopDetector 사용
    """
    def __init__(self):
        super().__init__('mirobot_status_joint_sequential_replicator')

        # ---------------- Params ----------------
        self.declare_parameter('master_pose_topic', '/mirobot1/target_pose_xyz')
        self.declare_parameter('slave_pose_topic',  '/mirobot2/target_pose_xyz')

        self.declare_parameter('master_is_idle_topic', '/mirobot1/is_idle')
        self.declare_parameter('slave_is_idle_topic',  '/mirobot2/is_idle')

        self.declare_parameter('master_joint_topic', '/mirobot1/joint_states')
        self.declare_parameter('slave_joint_topic',  '/mirobot2/joint_states')

        # 멈춤판정 튜닝
        self.declare_parameter('settle_time', 1.0)     # 연속 안정 시간(초)
        self.declare_parameter('vel_eps', 0.03)        # rad/s
        self.declare_parameter('delta_eps', 0.003)     # rad (샘플 간 변위)
        self.declare_parameter('joint_timeout', 1.0)   # joint_states가 이 시간 이상 안 오면 불신

        # 타이밍/안전장치
        self.declare_parameter('poll_hz', 30.0)
        self.declare_parameter('timeout', 45.0)         # 단계별 timeout
        self.declare_parameter('min_wait', 0.20)        # cmd 후 최소 대기
        self.declare_parameter('force_busy_after', 0.8) # busy edge 못 잡으면 이 시간 후 busy로 간주

        # 왕복 동작
        self.declare_parameter('enable_round_trip', True)
        self.declare_parameter('home_x', 200.0)
        self.declare_parameter('home_y', 140.0)
        self.declare_parameter('home_z', 100.0)

        # ---------------- Read ----------------
        self.master_pose_topic = self.get_parameter('master_pose_topic').value
        self.slave_pose_topic  = self.get_parameter('slave_pose_topic').value

        self.master_is_idle_topic = self.get_parameter('master_is_idle_topic').value
        self.slave_is_idle_topic  = self.get_parameter('slave_is_idle_topic').value

        self.master_joint_topic = self.get_parameter('master_joint_topic').value
        self.slave_joint_topic  = self.get_parameter('slave_joint_topic').value

        self.settle_time = float(self.get_parameter('settle_time').value)
        self.vel_eps = float(self.get_parameter('vel_eps').value)
        self.delta_eps = float(self.get_parameter('delta_eps').value)
        self.joint_timeout = float(self.get_parameter('joint_timeout').value)

        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.min_wait = float(self.get_parameter('min_wait').value)
        self.force_busy_after = float(self.get_parameter('force_busy_after').value)

        self.enable_round_trip = bool(self.get_parameter('enable_round_trip').value)

        self.home_pose = Pose()
        self.home_pose.position.x = float(self.get_parameter('home_x').value)
        self.home_pose.position.y = float(self.get_parameter('home_y').value)
        self.home_pose.position.z = float(self.get_parameter('home_z').value)

        # ---------------- QoS ----------------
        # Pose command는 보통 RELIABLE이 안전
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # joint_states는 sensor_data(best effort)로 받는 게 호환성 좋음
        qos_js = qos_profile_sensor_data

        # ---------------- Pub/Sub ----------------
        self.sub_master_cmd = self.create_subscription(Pose, self.master_pose_topic, self.cb_master_cmd, qos_cmd)

        self.sub_master_idle = self.create_subscription(Bool, self.master_is_idle_topic, self.cb_master_idle, 10)
        self.sub_slave_idle  = self.create_subscription(Bool, self.slave_is_idle_topic,  self.cb_slave_idle, 10)

        self.sub_master_js = self.create_subscription(JointState, self.master_joint_topic, self.cb_master_js, qos_js)
        self.sub_slave_js  = self.create_subscription(JointState, self.slave_joint_topic,  self.cb_slave_js,  qos_js)

        self.pub_slave_cmd  = self.create_publisher(Pose, self.slave_pose_topic, qos_cmd)
        self.pub_master_cmd = self.create_publisher(Pose, self.master_pose_topic, qos_cmd)

        # ---------------- State ----------------
        self.state = 'IDLE'
        self.cmd_time: Optional[float] = None
        self.target_pose: Optional[Pose] = None

        self.master_idle: Optional[bool] = None
        self.slave_idle: Optional[bool] = None

        self.master_idle_since: Optional[float] = None
        self.slave_idle_since: Optional[float] = None

        self.seen_master_busy = False
        self.seen_slave_busy = False

        self.master_stop = StopDetector(self.vel_eps, self.delta_eps, self.settle_time)
        self.slave_stop  = StopDetector(self.vel_eps, self.delta_eps, self.settle_time)

        self.timer = self.create_timer(1.0 / self.poll_hz, self.tick)

        self.get_logger().info(
            "SequentialReplicator (is_idle + joint_states) started\n"
            f" master_pose_topic={self.master_pose_topic}\n"
            f" slave_pose_topic={self.slave_pose_topic}\n"
            f" master_is_idle_topic={self.master_is_idle_topic}\n"
            f" slave_is_idle_topic={self.slave_is_idle_topic}\n"
            f" master_joint_topic={self.master_joint_topic}\n"
            f" slave_joint_topic={self.slave_joint_topic}\n"
            f" settle_time={self.settle_time:.2f}s, vel_eps={self.vel_eps:.3f}, delta_eps={self.delta_eps:.4f}\n"
            f" joint_timeout={self.joint_timeout:.2f}s, force_busy_after={self.force_busy_after:.2f}s\n"
            f" enable_round_trip={self.enable_round_trip}, home=({self.home_pose.position.x:.1f},{self.home_pose.position.y:.1f},{self.home_pose.position.z:.1f})"
        )

    # ---------------- Callbacks ----------------
    def cb_master_cmd(self, msg: Pose):
        self.target_pose = copy.deepcopy(msg)
        self.cmd_time = _now()
        self.state = 'WAIT_MASTER_DONE'

        # 새 목표 시작: busy/idle/stop 판정 리셋
        self.seen_master_busy = False
        self.master_idle_since = None
        self.master_stop.reset_for_new_goal()

        self.seen_slave_busy = False
        self.slave_idle_since = None
        self.slave_stop.reset_for_new_goal()

        self.get_logger().info(
            f"[MASTER CMD] X={msg.position.x:.1f}, Y={msg.position.y:.1f}, Z={msg.position.z:.1f}"
        )

    def cb_master_idle(self, msg: Bool):
        t = _now()
        self.master_idle = bool(msg.data)

        if self.master_idle is False:
            self.seen_master_busy = True
            self.master_idle_since = None
        else:
            if self.master_idle_since is None:
                self.master_idle_since = t

    def cb_slave_idle(self, msg: Bool):
        t = _now()
        self.slave_idle = bool(msg.data)

        if self.slave_idle is False:
            self.seen_slave_busy = True
            self.slave_idle_since = None
        else:
            if self.slave_idle_since is None:
                self.slave_idle_since = t

    def cb_master_js(self, msg: JointState):
        self.master_stop.update(msg)
        if self.master_stop.moving_seen():
            self.seen_master_busy = True

    def cb_slave_js(self, msg: JointState):
        self.slave_stop.update(msg)
        if self.slave_stop.moving_seen():
            self.seen_slave_busy = True

    # ---------------- Helpers ----------------
    def _idle_stable(self, idle_flag: Optional[bool], idle_since: Optional[float]) -> bool:
        if idle_flag is not True:
            return False
        if idle_since is None:
            return False
        return (_now() - idle_since) >= self.settle_time

    def _joint_ok(self, det: StopDetector) -> bool:
        if det.last_msg_time is None:
            return False
        return (_now() - det.last_msg_time) <= self.joint_timeout

    def _master_done(self) -> bool:
        # 1) is_idle가 신뢰 가능하면(is_idle True 안정) 우선 사용
        if self._idle_stable(self.master_idle, self.master_idle_since):
            return True

        # 2) is_idle이 불안정/미수신이면 joint_states 기반 사용
        if self._joint_ok(self.master_stop) and self.master_stop.stopped_stable():
            return True

        return False

    def _slave_done(self) -> bool:
        if self._idle_stable(self.slave_idle, self.slave_idle_since):
            return True

        if self._joint_ok(self.slave_stop) and self.slave_stop.stopped_stable():
            return True

        return False

    def _ensure_busy_seen(self, which: str, elapsed: float):
        """
        busy edge를 못 잡아도 force_busy_after 이후에는 busy로 간주.
        (is_idle이 True만 찍히는 환경에서도 진행되도록)
        """
        if which == 'master':
            if self.seen_master_busy:
                return
            if elapsed >= self.force_busy_after:
                self.seen_master_busy = True
                self.master_idle_since = None
                self.master_stop.reset_for_new_goal()  # still_since를 새로 잡게
                self.get_logger().warn(
                    f"[MASTER BUSY ASSUME] no busy edge -> assume busy after {self.force_busy_after:.2f}s"
                )
        elif which == 'slave':
            if self.seen_slave_busy:
                return
            if elapsed >= self.force_busy_after:
                self.seen_slave_busy = True
                self.slave_idle_since = None
                self.slave_stop.reset_for_new_goal()
                self.get_logger().warn(
                    f"[SLAVE BUSY ASSUME] no busy edge -> assume busy after {self.force_busy_after:.2f}s"
                )

    def _reset_idle(self):
        self.state = 'IDLE'
        self.cmd_time = None
        self.target_pose = None

    # ---------------- Main loop ----------------
    def tick(self):
        if self.state == 'IDLE' or self.cmd_time is None:
            return

        elapsed = _now() - self.cmd_time

        if elapsed < self.min_wait:
            return

        if elapsed > self.timeout:
            self.get_logger().warn(f"[TIMEOUT] state={self.state}, elapsed={elapsed:.1f}s -> force advance")
            if self.state == 'WAIT_MASTER_DONE':
                # 강제: 슬레이브 시작
                if self.target_pose is not None:
                    self.pub_slave_cmd.publish(self.target_pose)
                    self.get_logger().warn("[FORCE] master done timeout -> start slave anyway")
                self.state = 'WAIT_SLAVE_DONE'
                self.cmd_time = _now()
                self.seen_slave_busy = False
                self.slave_idle_since = None
                self.slave_stop.reset_for_new_goal()
                return

            if self.state == 'WAIT_SLAVE_DONE':
                # 강제: 왕복/종료
                if self.enable_round_trip:
                    # 슬레이브 home -> 마스터 home (순서)
                    self.pub_slave_cmd.publish(self.home_pose)
                    self.get_logger().warn("[FORCE] slave done timeout -> slave home anyway")
                    self.state = 'WAIT_SLAVE_HOME_DONE'
                    self.cmd_time = _now()
                    self.seen_slave_busy = False
                    self.slave_idle_since = None
                    self.slave_stop.reset_for_new_goal()
                else:
                    self._reset_idle()
                return

            if self.state == 'WAIT_SLAVE_HOME_DONE':
                self.pub_master_cmd.publish(self.home_pose)
                self.get_logger().warn("[FORCE] slave home timeout -> master home anyway")
                self.state = 'WAIT_MASTER_HOME_DONE'
                self.cmd_time = _now()
                self.seen_master_busy = False
                self.master_idle_since = None
                self.master_stop.reset_for_new_goal()
                return

            if self.state == 'WAIT_MASTER_HOME_DONE':
                self.get_logger().warn("[FORCE] master home timeout -> finish")
                self._reset_idle()
                return

        # ---------- 정상 진행 ----------
        if self.state == 'WAIT_MASTER_DONE':
            self._ensure_busy_seen('master', elapsed)

            # busy를 본 뒤에만 done 체크(너무 빨리 통과 방지)
            if not self.seen_master_busy:
                return

            if self._master_done():
                # 슬레이브 시작
                if self.target_pose is not None:
                    self.pub_slave_cmd.publish(self.target_pose)
                    self.get_logger().info("[SLAVE START] master done -> publish slave target")
                self.state = 'WAIT_SLAVE_DONE'
                self.cmd_time = _now()
                self.seen_slave_busy = False
                self.slave_idle_since = None
                self.slave_stop.reset_for_new_goal()
                return

        if self.state == 'WAIT_SLAVE_DONE':
            self._ensure_busy_seen('slave', elapsed)

            if not self.seen_slave_busy:
                return

            if self._slave_done():
                self.get_logger().info("[SLAVE DONE]")
                if self.enable_round_trip:
                    # ✅ 최종 목표 흐름: 슬레이브가 원위치 -> 마스터가 원위치
                    self.pub_slave_cmd.publish(self.home_pose)
                    self.get_logger().info("[SLAVE HOME] publish slave home")
                    self.state = 'WAIT_SLAVE_HOME_DONE'
                    self.cmd_time = _now()
                    self.seen_slave_busy = False
                    self.slave_idle_since = None
                    self.slave_stop.reset_for_new_goal()
                else:
                    self._reset_idle()
                return

        if self.state == 'WAIT_SLAVE_HOME_DONE':
            self._ensure_busy_seen('slave', elapsed)

            if not self.seen_slave_busy:
                return

            if self._slave_done():
                self.get_logger().info("[SLAVE HOME DONE] -> now master home")
                self.pub_master_cmd.publish(self.home_pose)
                self.state = 'WAIT_MASTER_HOME_DONE'
                self.cmd_time = _now()
                self.seen_master_busy = False
                self.master_idle_since = None
                self.master_stop.reset_for_new_goal()
                return

        if self.state == 'WAIT_MASTER_HOME_DONE':
            self._ensure_busy_seen('master', elapsed)

            if not self.seen_master_busy:
                return

            if self._master_done():
                self.get_logger().info("[MASTER HOME DONE] sequence finished")
                self._reset_idle()
                return


def main():
    rclpy.init()
    node = MirobotSequential()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

