"""
nav2_goal_publisher.py
======================
ROS 2 Node: Nav2 전방 목표 지속 게시자

동작 원리:
  - 'AUTO' 모드일 때 로봇 전방 GOAL_DIST_M 지점을 NavigateToPose 목표로 게시
  - 목표 달성 또는 실패 시 새로운 목표 즉시 게시
  - Nav2 DWBLocalPlanner가 목표까지 경로 계획 + 장애물 자동 회피
  - 장애물 우회 후 전방 방향으로 경로 자동 복귀

토픽:
  구독: /mode (std_msgs/String) — "MANUAL" | "AUTO"
  구독: /tf   (TF2)             — odom → base_link 변환

Action Client:
  navigate_to_pose (nav2_msgs/action/NavigateToPose)

파라미터:
  goal_distance_m   float  3.0   (목표까지 전방 거리)
  update_rate_hz    float  0.5   (목표 갱신 주기 Hz)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros.exceptions import (
        LookupException, ConnectivityException, ExtrapolationException
    )
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False


GOAL_DISTANCE_M = 3.0   # $전방 목표 거리 (m)
UPDATE_RATE_HZ  = 0.5   # $목표 갱신 주기 (0.5Hz = 2초마다)


class Nav2GoalPublisher(Node):

    def __init__(self):
        super().__init__('nav2_goal_publisher')

        self.declare_parameter('goal_distance_m', GOAL_DISTANCE_M)
        self.declare_parameter('update_rate_hz',  UPDATE_RATE_HZ)

        self._goal_dist = self.get_parameter('goal_distance_m').value
        rate_hz         = self.get_parameter('update_rate_hz').value

        # ── 상태 ─────────────────────────────────────────────────
        self._mode     = 'MANUAL'
        self._mode_lock = threading.Lock()
        self._nav_busy  = False

        # ── TF2 ──────────────────────────────────────────────────
        if _TF2_AVAILABLE:
            self._tf_buffer   = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        else:
            self.get_logger().warn('tf2_ros 미사용: TF 변환 없이 전방 고정 목표 사용')
            self._tf_buffer = None

        # ── Nav2 Action Client ───────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── /mode 구독 ────────────────────────────────────────────
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )

        # ── 목표 게시 타이머 ──────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._goal_tick)

        self.get_logger().info(
            f'Nav2GoalPublisher 준비 완료 '
            f'(전방 {self._goal_dist:.1f}m, {rate_hz:.1f}Hz 갱신)'
        )

    # ── /mode 콜백 ────────────────────────────────────────────────
    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old         = self._mode
            self._mode  = msg.data
        if old != msg.data:
            self.get_logger().info(f'[NAV2] 모드: {old} → {msg.data}')

    # ── 전방 목표 좌표 계산 ───────────────────────────────────────
    def _get_forward_goal(self) -> PoseStamped | None:
        """
        현재 로봇 자세에서 전방 GOAL_DIST_M 지점의 PoseStamped 반환.
        TF2 미사용 시: odom 프레임에서 x+방향으로 고정 목표.
        """
        goal = PoseStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'base_link'   # base_link 기준 전방

        goal.pose.position.x    = self._goal_dist
        goal.pose.position.y    = 0.0
        goal.pose.position.z    = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        return goal

    # ── 목표 전송 (ActionClient) ──────────────────────────────────
    def _send_goal(self, pose: PoseStamped) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('[NAV2] navigate_to_pose 서버 미응답')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        self._nav_busy = True
        future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[NAV2] 목표 거부됨')
            self._nav_busy = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self._nav_busy = False
        result = future.result().result
        self.get_logger().info(f'[NAV2] 목표 완료 (result={result})')

    def _feedback_cb(self, feedback_msg) -> None:
        pass   # 필요시 진행 로그 추가

    # ── 타이머: 목표 갱신 ────────────────────────────────────────
    def _goal_tick(self) -> None:
        with self._mode_lock:
            mode = self._mode

        if mode != 'AUTO':
            return

        # 이전 목표 진행 중이면 갱신하지 않음 (재계획은 Nav2가 처리)
        if self._nav_busy:
            return

        goal = self._get_forward_goal()
        if goal is not None:
            self.get_logger().info(f'[NAV2] 전방 {self._goal_dist:.1f}m 목표 게시')
            self._send_goal(goal)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
