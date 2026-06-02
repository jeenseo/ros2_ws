"""
avoidance_node.py
=================
ROS 2 Node: 장애물 회피 자율 주행 (AUTO 모드)

수정 사항:
1. rclpy.qos.qos_profile_sensor_data 적용 (LiDAR 데이터 수신 불가 해결)
2. lidar_offset 파라미터 추가 및 수식 적용 (앞/뒤 180도 착각 문제 해결)
3. math.isinf, math.isnan 예외 처리 추가 (안정성 강화)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# [수정 1] 센서 데이터를 받기 위한 필수 QoS 라이브러리 추가
from rclpy.qos import qos_profile_sensor_data

class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('avoidance_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('front_body_cm',     6.5)
        self.declare_parameter('left_body_cm',      17.0)
        self.declare_parameter('right_body_cm',     17.5)
        self.declare_parameter('safety_margin_cm',  40.0)
        self.declare_parameter('turn_10deg_sec',    0.2)
        self.declare_parameter('backward_10cm_sec', 0.5)
        self.declare_parameter('forward_speed',     0.2002)
        self.declare_parameter('turn_speed',        0.2002)
        # [수정 2] 런치 파일에서 전달하는 lidar_offset 파라미터 선언
        self.declare_parameter('lidar_offset',      0.0)

        margin = self.get_parameter('safety_margin_cm').value

        # 안전 경계 (차체 치수 + 마진)
        self._FRONT_SAFE = self.get_parameter('front_body_cm').value + margin
        self._LEFT_SAFE  = self.get_parameter('left_body_cm').value  + margin
        self._RIGHT_SAFE = self.get_parameter('right_body_cm').value + margin

        self._turn_sec = self.get_parameter('turn_10deg_sec').value
        self._back_sec = self.get_parameter('backward_10cm_sec').value
        self._fwd_spd  = self.get_parameter('forward_speed').value
        self._trn_spd  = self.get_parameter('turn_speed').value
        
        # 라이다 조립 방향 보정값 (라디안 변환)
        self._lidar_offset_rad = math.radians(self.get_parameter('lidar_offset').value)

        self.get_logger().info(
            f'바운딩박스 안전 구역: '
            f'전방={self._FRONT_SAFE:.1f}cm  '
            f'좌={self._LEFT_SAFE:.1f}cm  '
            f'우={self._RIGHT_SAFE:.1f}cm'
        )

        # ── 공유 상태 ─────────────────────────────────────────────
        self._lock        = threading.Lock()
        self._mode        = 'MANUAL'
        self._threat_dist = float('inf')   # 전방 위협 거리 (cm)

        # ── 상태머신 ──────────────────────────────────────────────
        self._state       = 'FORWARD'
        self._state_start = time.monotonic()

        # ── 토픽 게시자 / 구독자 ──────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # [수정 1] QoS 프로필을 qos_profile_sensor_data로 변경
        self._sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data
        )
        
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )

        # ── 10Hz 상태머신 타이머 ──────────────────────────────────
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('AvoidanceNode 준비 완료 (QoS 및 Offset 패치 완료)')

    # ── LaserScan 콜백: 비대칭 바운딩박스 위협 판정 ──────────────
    def _scan_cb(self, msg: LaserScan) -> None:
        threat = float('inf')

        for i, r_m in enumerate(msg.ranges):
            # [수정 3] 무한대(inf)나 결측치(NaN) 필터링 강화
            if math.isinf(r_m) or math.isnan(r_m) or r_m <= msg.range_min or r_m >= msg.range_max or r_m == 0.0:
                continue

            # [수정 2] 라이다 각도에 오프셋(보정값) 추가
            angle_rad = msg.angle_min + i * msg.angle_increment + self._lidar_offset_rad

            r_cm = r_m * 100.0

            x_cm = r_cm * math.cos(angle_rad)   # 전방 +
            y_cm = r_cm * math.sin(angle_rad)   # 좌측 +

            if x_cm > 0.0 and (-self._RIGHT_SAFE <= y_cm <= self._LEFT_SAFE):
                if x_cm < threat:
                    threat = x_cm

        with self._lock:
            self._threat_dist = threat

    # ── /mode 콜백 ────────────────────────────────────────────────
    def _mode_cb(self, msg: String) -> None:
        with self._lock:
            old        = self._mode
            self._mode = msg.data
        if old != msg.data:
            self.get_logger().info(f'모드 전환: {old} → {msg.data}')
            if msg.data == 'AUTO':
                self._state       = 'FORWARD'
                self._state_start = time.monotonic()

    # ── Twist 게시 헬퍼 ───────────────────────────────────────────
    def _pub(self, linear: float, angular: float) -> None:
        msg           = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    # ── 상태머신 틱 (10Hz 타이머) ─────────────────────────────────
    def _tick(self) -> None:
        with self._lock:
            mode   = self._mode
            threat = self._threat_dist

        if mode != 'AUTO':
            return

        now = time.monotonic()
        s   = self._state

        if s == 'FORWARD':
            if threat > self._FRONT_SAFE:
                self._pub(self._fwd_spd, 0.0)
            else:
                self.get_logger().warn(
                    f'[AUTO] 장애물! 전방 위협 {threat:.1f}cm '
                    f'(한계={self._FRONT_SAFE:.1f}cm) → 정지'
                )
                self._pub(0.0, 0.0)
                self._state       = 'STOP_BEFORE_TURN'
                self._state_start = now

        elif s == 'STOP_BEFORE_TURN':
            if now - self._state_start >= 0.2:
                self.get_logger().info(f'[AUTO] 1차 좌 포인트 턴 ({self._turn_sec:.2f}s)')
                self._state       = 'TURN_LEFT_1'
                self._state_start = now

        elif s == 'TURN_LEFT_1':
            self._pub(0.0, self._trn_spd)
            if now - self._state_start >= self._turn_sec:
                self._pub(0.0, 0.0)
                self._state       = 'RECHECK'
                self._state_start = now

        elif s == 'RECHECK':
            if now - self._state_start >= 0.15:
                if threat > self._FRONT_SAFE:
                    self.get_logger().info('[AUTO] 경로 확보 → 전진 재개')
                    self._state = 'FORWARD'
                else:
                    self.get_logger().warn(f'[AUTO] 여전히 막힘! 후진 ({self._back_sec:.2f}s)')
                    self._state       = 'BACKWARD'
                    self._state_start = now

        elif s == 'BACKWARD':
            self._pub(-self._fwd_spd, 0.0)
            if now - self._state_start >= self._back_sec:
                self._pub(0.0, 0.0)
                self.get_logger().info(f'[AUTO] 2차 좌 포인트 턴 ({self._turn_sec:.2f}s)')
                self._state       = 'TURN_LEFT_2'
                self._state_start = now

        elif s == 'TURN_LEFT_2':
            self._pub(0.0, self._trn_spd)
            if now - self._state_start >= self._turn_sec:
                self._pub(0.0, 0.0)
                self.get_logger().info('[AUTO] 회피 완료 → 전진 재개')
                self._state = 'FORWARD'

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
