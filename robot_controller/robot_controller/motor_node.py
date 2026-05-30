"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK + Encoder Odometry

[CAN TX: 0x123] Pi → STM32
  8바이트 Big-Endian: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]
  단위: -9999 ~ +9999 (정규화 속도)

[CAN RX: 0x124] STM32 → Pi
  4바이트 Big-Endian: [Left_delta(2B)][Right_delta(2B)]
  단위: 엔코더 누산 틱 (STM32가 전송 후 0으로 리셋)
  하드웨어 주의: 우측 모터 역방향 장착 → Python에서 ×(-1) 보정

[오도메트리 모델: 차동 구동]
  바퀴 직경: 0.12 m → 둘레 = π × 0.12 m
  트랙 폭: 0.51 m (바퀴 중심 간 거리)
  엔코더 CPR: 1404 (13 PPR × 27 기어비 × 4 체배)
  Vy = 0 (스트레이핑 없음)

[발행 토픽]
  /odom (nav_msgs/Odometry)   — 위치 + 속도 + 공분산
  TF: odom → base_link         — tf2 브로드캐스트

[cmd_vel Mux]
  /cmd_vel_keyboard → MANUAL 모드 전용
  /cmd_vel_nav2     → AUTO   모드 전용

[E-Stop]
  /e_stop True → CAN 즉시 정지
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

import can
import tf2_ros


# ── cmd_vel QoS: depth=1 ────────────────────────────────────────
_CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# ── 오도메트리 물리 상수 ─────────────────────────────────────────
_WHEEL_DIAMETER_M   = 0.12                           # 바퀴 직경 (m)
_WHEEL_CIRCUM_M     = math.pi * _WHEEL_DIAMETER_M    # 바퀴 둘레 (m)
_ENCODER_CPR        = 1404                           # 엔코더 해상도 (CPR)
_TRACK_WIDTH_M      = 0.51                           # 트랙 폭: 바퀴 중심 간 거리 (m)
_METER_PER_TICK     = _WHEEL_CIRCUM_M / _ENCODER_CPR # m/tick ≈ 0.0002685 m

# ── CAN ID ──────────────────────────────────────────────────────
_CAN_TX_ID          = 0x123   # Pi → STM32 모터 명령
_CAN_RX_FB_ID       = 0x124   # STM32 → Pi 엔코더 피드백


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      _CAN_TX_ID)
        self.declare_parameter('max_speed',   9999)
        self.declare_parameter('odom_frame',  'odom')
        self.declare_parameter('base_frame',  'base_link')

        channel          = self.get_parameter('can_channel').value
        self._can_id     = self.get_parameter('can_id').value
        self._max_speed  = self.get_parameter('max_speed').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── 모드 & E-Stop 상태 ────────────────────────────────────
        self._mode        = 'MANUAL'
        self._mode_lock   = threading.Lock()
        self._is_estopped = False
        self._estop_lock  = threading.Lock()

        # ── 오도메트리 상태 (CAN RX 콜백 스레드와 공유) ───────────
        self._odom_lock  = threading.Lock()
        self._pose_x     = 0.0    # 누산 x 위치 (m)
        self._pose_y     = 0.0    # 누산 y 위치 (m)
        self._pose_yaw   = 0.0    # 누산 yaw 자세 (rad)
        self._vel_x      = 0.0    # 선속도 (m/s)
        self._vel_yaw    = 0.0    # 각속도 (rad/s)
        self._last_fb_time: float | None = None  # 마지막 피드백 수신 시각 (monotonic)

        # ── CAN 버스 초기화 ───────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(
                f'CAN 버스 초기화 완료 ({channel})'
                f' | TX=0x{self._can_id:03X} | RX=0x{_CAN_RX_FB_ID:03X}'
            )
        except Exception as exc:
            self.get_logger().fatal(f'CAN 버스 초기화 실패: {exc}')
            raise

        # ── CAN RX Notifier (백그라운드 스레드에서 콜백 호출) ─────
        # can.Notifier는 별도 스레드를 생성하여 수신 메시지를 처리
        self._notifier = can.Notifier(self._bus, [self._can_rx_callback])

        # ── ROS 구독 ──────────────────────────────────────────────
        self._sub_keyboard = self.create_subscription(
            Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS,
        )
        self._sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel_nav2', self._nav2_cb, _CMD_VEL_QOS,
        )
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10,
        )
        self._sub_estop = self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10,
        )

        # ── ROS 게시자 ────────────────────────────────────────────
        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)
        self._pub_odom      = self.create_publisher(Odometry, '/odom', 10)

        # ── TF 브로드캐스터 ───────────────────────────────────────
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            'MotorNode 준비 완료\n'
            f'  TX: CAN 0x{self._can_id:03X} | IK: Mecanum Vy=0 강제\n'
            f'  RX: CAN 0x{_CAN_RX_FB_ID:03X} | /odom + TF({self._odom_frame}→{self._base_frame})'
        )

    # ══════════════════════════════════════════════════════════════
    # CAN RX: STM32 엔코더 피드백 처리
    # ══════════════════════════════════════════════════════════════

    def _can_rx_callback(self, msg: can.Message) -> None:
        """
        CAN Notifier 콜백 — 백그라운드 스레드에서 호출됨.
        ID 0x124 메시지만 처리.

        Payload (4바이트, Big-Endian):
          Byte 0-1: int16 left_delta_ticks
          Byte 2-3: int16 right_delta_ticks  ← 하드웨어 역방향 → ×(-1) 적용
        """
        if msg.arbitration_id != _CAN_RX_FB_ID:
            return  # 다른 ID는 무시

        if len(msg.data) < 4:
            self.get_logger().warn(f'[RX 0x124] 페이로드 부족: {len(msg.data)}B')
            return

        # ── 디코딩 ────────────────────────────────────────────────
        left_ticks, right_ticks = struct.unpack('>hh', msg.data[:4])
        right_ticks = -right_ticks   # 우측 모터 역방향 보정 (물리 장착)

        # ── dt 계산 ───────────────────────────────────────────────
        now = time.monotonic()
        with self._odom_lock:
            if self._last_fb_time is None:
                # 첫 수신: 상태만 초기화
                self._last_fb_time = now
                return

            dt = now - self._last_fb_time
            self._last_fb_time = now

        if dt <= 0.0 or dt > 1.0:
            # dt가 비정상이면 건너뜀 (노드 재시작 직후 등)
            return

        # ── 틱 → 거리 변환 (m) ───────────────────────────────────
        dist_left  = left_ticks  * _METER_PER_TICK
        dist_right = right_ticks * _METER_PER_TICK

        # ── 차동 구동 오도메트리 계산 ─────────────────────────────
        #
        #   delta_center = (dist_L + dist_R) / 2       [직선 이동량]
        #   delta_yaw    = (dist_R - dist_L) / TRACK   [회전량]
        #
        #   중간 yaw를 사용한 위치 갱신 (2차 정확도):
        #     x += delta_center × cos(yaw + delta_yaw/2)
        #     y += delta_center × sin(yaw + delta_yaw/2)
        #     yaw += delta_yaw
        #
        delta_center = (dist_left + dist_right) * 0.5
        delta_yaw    = (dist_right - dist_left) / _TRACK_WIDTH_M

        with self._odom_lock:
            mid_yaw          = self._pose_yaw + delta_yaw * 0.5
            self._pose_x    += delta_center * math.cos(mid_yaw)
            self._pose_y    += delta_center * math.sin(mid_yaw)
            self._pose_yaw  += delta_yaw
            self._vel_x      = delta_center / dt
            self._vel_yaw    = delta_yaw    / dt

            x   = self._pose_x
            y   = self._pose_y
            yaw = self._pose_yaw
            vx  = self._vel_x
            wz  = self._vel_yaw

        # ── /odom 게시 + TF 브로드캐스트 ─────────────────────────
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(x, y, yaw, vx, wz, stamp)

    # ══════════════════════════════════════════════════════════════
    # 오도메트리 게시 헬퍼
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        """Yaw (rad) → Quaternion (qx, qy, qz, qw)."""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _publish_odom(
        self,
        x: float, y: float, yaw: float,
        vx: float, wz: float,
        stamp,
    ) -> None:
        """
        /odom 토픽 게시 + odom → base_link TF 브로드캐스트.
        이 메서드는 CAN Notifier 콜백(백그라운드 스레드)에서 호출됨.
        """
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        # ── Odometry 메시지 ───────────────────────────────────────
        odom = Odometry()
        odom.header.stamp     = stamp
        odom.header.frame_id  = self._odom_frame
        odom.child_frame_id   = self._base_frame

        # 위치
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 속도
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = wz

        # 공분산 (대각 원소만 설정, 나머지 0)
        # pose: [x, y, z, roll, pitch, yaw] — 2D이므로 x, y, yaw만 유효
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw
        # twist: [vx, vy, vz, wx, wy, wz]
        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # wz

        self._pub_odom.publish(odom)

        # ── TransformStamped (odom → base_link) ──────────────────
        # tf_msg = TransformStamped()
        # tf_msg.header.stamp     = stamp
        # tf_msg.header.frame_id  = self._odom_frame
        # tf_msg.child_frame_id   = self._base_frame

        # tf_msg.transform.translation.x = x
        # tf_msg.transform.translation.y = y
        # tf_msg.transform.translation.z = 0.0
        # tf_msg.transform.rotation.x    = qx
        # tf_msg.transform.rotation.y    = qy
        # tf_msg.transform.rotation.z    = qz
        # tf_msg.transform.rotation.w    = qw

        # self._tf_broadcaster.sendTransform(tf_msg)

    # ══════════════════════════════════════════════════════════════
    # cmd_vel 처리 (TX: 모터 명령)
    # ══════════════════════════════════════════════════════════════

    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old        = self._mode
            self._mode = msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')
            self._send_can(0, 0, 0, 0)

    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev              = self._is_estopped
            self._is_estopped = msg.data

        if msg.data and not prev:
            self.get_logger().warn('[E-STOP] 발동!')
            self._send_can(0, 0, 0, 0)
            ack = Bool(); ack.data = True
            self._pub_estop_ack.publish(ack)
        elif not msg.data and prev:
            self.get_logger().info('[E-STOP] 해제')
            ack = Bool(); ack.data = False
            self._pub_estop_ack.publish(ack)

    def _keyboard_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'MANUAL':
                return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return
        self._apply_twist(msg)

    def _nav2_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'AUTO':
                return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return
        self._apply_twist(msg)

    # ── Mecanum IK (Vy=0, 스키드-스티어 동작) ─────────────────────
    def _apply_twist(self, msg: Twist) -> None:
        """
        Twist → Mecanum X-구성 IK (Vy=0 강제) → CAN TX 0x123.
        방향 반전 보정은 STM32 motor.h DIR_INVERT에서 처리.
        """
        Vx = max(-1.0, min(1.0, float(msg.linear.x)))
        Wz = max(-1.0, min(1.0, float(msg.angular.z)))
        # Vy = 0 (스트레이핑 비활성화)

        N = self._max_speed

        fl_can = int((Vx - Wz) * N)
        fr_can = int((Vx + Wz) * N)
        rl_can = int((Vx - Wz) * N)
        rr_can = int((Vx + Wz) * N)

        fl_can = max(-N, min(N, fl_can))
        fr_can = max(-N, min(N, fr_can))
        rl_can = max(-N, min(N, rl_can))
        rr_can = max(-N, min(N, rr_can))

        self._send_can(fl_can, fr_can, rl_can, rr_can)
        self.get_logger().debug(
            f'[TX] Vx={Vx:+.3f} Wz={Wz:+.3f} | '
            f'FL={fl_can:+6d} FR={fr_can:+6d} RL={rl_can:+6d} RR={rr_can:+6d}'
        )

    # ── CAN TX ────────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송 (ID 0x123)."""
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        tx_msg = can.Message(
            arbitration_id=self._can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(tx_msg)
        except can.CanError as exc:
            self.get_logger().error(f'[CAN TX ERROR] {exc}')

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._notifier.stop()       # CAN RX 백그라운드 스레드 종료
            self._send_can(0, 0, 0, 0)  # 긴급 정지 송신
            self._bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
