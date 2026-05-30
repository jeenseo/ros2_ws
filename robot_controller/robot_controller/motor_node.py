"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK + Encoder Odometry

[아키텍처 규칙]
  이 노드는 절대 TF를 발행하지 않습니다.
  odom → base_footprint TF는 ekf_node (robot_localization) 전담.

[CAN TX: 0x123] Pi → STM32
  8바이트 Big-Endian: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]
  단위: -9999 ~ +9999 (정규화 속도)

[CAN RX: 0x124] STM32 → Pi
  4바이트 Big-Endian: [Left_delta(2B)][Right_delta(2B)]
  단위: 엔코더 누산 틱 (STM32가 전송 후 0으로 리셋)
  하드웨어 주의: 우측 모터 역방향 장착 → right_ticks × (-1) 적용

[오도메트리 출력]
  /odom_motor (nav_msgs/Odometry)
  — EKF의 odom1 소스로 사용
  — TF 발행 없음

[오도메트리 모델: 차동 구동 (Vy=0)]
  바퀴 직경: 0.12 m, 트랙 폭: 0.51 m, CPR: 1404

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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

import can


# ── cmd_vel QoS: depth=1 ────────────────────────────────────────
_CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# ── 오도메트리 물리 상수 ─────────────────────────────────────────
_WHEEL_DIAMETER_M = 0.12
_WHEEL_CIRCUM_M   = math.pi * _WHEEL_DIAMETER_M   # ≈ 0.3770 m
_ENCODER_CPR      = 1404                           # 13 PPR × 27 기어 × 4 체배
_TRACK_WIDTH_M    = 0.51                           # 바퀴 중심 간 거리 (m)
_METER_PER_TICK   = _WHEEL_CIRCUM_M / _ENCODER_CPR # ≈ 0.0002685 m/tick

# ── CAN ID ──────────────────────────────────────────────────────
_CAN_TX_ID    = 0x123   # Pi → STM32 모터 명령
_CAN_RX_FB_ID = 0x124   # STM32 → Pi 엔코더 피드백


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      _CAN_TX_ID)
        self.declare_parameter('max_speed',   9999)
        self.declare_parameter('odom_frame',  'odom')
        self.declare_parameter('base_frame',  'base_footprint')  # EKF child_frame
        self.declare_parameter('odom_topic',  '/odom_motor')     # EKF odom1 소스

        channel          = self.get_parameter('can_channel').value
        self._can_id     = self.get_parameter('can_id').value
        self._max_speed  = self.get_parameter('max_speed').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        odom_topic       = self.get_parameter('odom_topic').value

        # ── 모드 & E-Stop 상태 ────────────────────────────────────
        self._mode        = 'MANUAL'
        self._mode_lock   = threading.Lock()
        self._is_estopped = False
        self._estop_lock  = threading.Lock()

        # ── 오도메트리 상태 (CAN RX Notifier 콜백과 공유) ─────────
        self._odom_lock      = threading.Lock()
        self._pose_x         = 0.0
        self._pose_y         = 0.0
        self._pose_yaw       = 0.0
        self._vel_x          = 0.0
        self._vel_yaw        = 0.0
        self._last_fb_time: float | None = None

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

        # ── CAN RX Notifier ───────────────────────────────────────
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
        self._pub_odom      = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(
            'MotorNode 준비 완료\n'
            f'  TX: CAN 0x{self._can_id:03X} | IK: Mecanum Vy=0\n'
            f'  RX: CAN 0x{_CAN_RX_FB_ID:03X} → {odom_topic}\n'
            f'  TF 발행: 없음 (EKF 전담)'
        )

    # ══════════════════════════════════════════════════════════════
    # CAN RX: STM32 엔코더 피드백
    # ══════════════════════════════════════════════════════════════

    def _can_rx_callback(self, msg: can.Message) -> None:
        """
        CAN Notifier 콜백 (백그라운드 스레드).
        ID 0x124만 처리.

        Payload 4바이트 Big-Endian:
          Byte 0-1: int16 left_delta_ticks
          Byte 2-3: int16 right_delta_ticks (× -1 역방향 보정)
        """
        if msg.arbitration_id != _CAN_RX_FB_ID:
            return

        if len(msg.data) < 4:
            self.get_logger().warn(f'[RX 0x124] 페이로드 부족: {len(msg.data)}B')
            return

        left_ticks, right_ticks = struct.unpack('>hh', msg.data[:4])
        right_ticks = -right_ticks   # 우측 모터 역방향 보정

        # ── dt 계산 ───────────────────────────────────────────────
        now = time.monotonic()
        with self._odom_lock:
            if self._last_fb_time is None:
                self._last_fb_time = now
                return
            dt = now - self._last_fb_time
            self._last_fb_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # ── 틱 → 거리 (m) ────────────────────────────────────────
        dist_left  = left_ticks  * _METER_PER_TICK
        dist_right = right_ticks * _METER_PER_TICK

        # ── 차동 구동 오도메트리 (2차 정확도) ────────────────────
        delta_center = (dist_left + dist_right) * 0.5
        delta_yaw    = (dist_right - dist_left) / _TRACK_WIDTH_M

        with self._odom_lock:
            mid_yaw         = self._pose_yaw + delta_yaw * 0.5
            self._pose_x   += delta_center * math.cos(mid_yaw)
            self._pose_y   += delta_center * math.sin(mid_yaw)
            self._pose_yaw += delta_yaw
            self._vel_x     = delta_center / dt
            self._vel_yaw   = delta_yaw    / dt

            x   = self._pose_x
            y   = self._pose_y
            yaw = self._pose_yaw
            vx  = self._vel_x
            wz  = self._vel_yaw

        stamp = self.get_clock().now().to_msg()
        self._publish_odom(x, y, yaw, vx, wz, stamp)

    # ══════════════════════════════════════════════════════════════
    # 오도메트리 게시 (/odom_motor 전용, TF 발행 없음)
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _publish_odom(self, x, y, yaw, vx, wz, stamp) -> None:
        """
        /odom_motor (nav_msgs/Odometry) 게시.
        TF는 ekf_node가 단독 발행 — 이 노드는 TF를 발행하지 않음.
        """
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame   # base_footprint

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = wz

        # 공분산 (대각 원소, 2D 주행)
        odom.pose.covariance[0]  = 0.05   # x
        odom.pose.covariance[7]  = 0.05   # y
        odom.pose.covariance[35] = 0.1    # yaw
        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # wz

        self._pub_odom.publish(odom)

    # ══════════════════════════════════════════════════════════════
    # cmd_vel Mux
    # ══════════════════════════════════════════════════════════════

    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old, self._mode = self._mode, msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')
            self._send_can(0, 0, 0, 0)

    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev, self._is_estopped = self._is_estopped, msg.data
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
            if self._mode != 'MANUAL': return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0); return
        self._apply_twist(msg)

    def _nav2_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'AUTO': return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0); return
        self._apply_twist(msg)

    def _apply_twist(self, msg: Twist) -> None:
        """Mecanum X-구성 IK (Vy=0 강제) → CAN TX 0x123."""
        Vx = max(-1.0, min(1.0, float(msg.linear.x)))
        Wz = max(-1.0, min(1.0, float(msg.angular.z)))
        N  = self._max_speed

        fl = max(-N, min(N, int((Vx - Wz) * N)))
        fr = max(-N, min(N, int((Vx + Wz) * N)))
        rl = max(-N, min(N, int((Vx - Wz) * N)))
        rr = max(-N, min(N, int((Vx + Wz) * N)))

        self._send_can(fl, fr, rl, rr)
        self.get_logger().debug(
            f'[TX] Vx={Vx:+.3f} Wz={Wz:+.3f} | '
            f'FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        try:
            self._bus.send(can.Message(
                arbitration_id=self._can_id,
                data=data,
                is_extended_id=False,
            ))
        except can.CanError as exc:
            self.get_logger().error(f'[CAN TX ERROR] {exc}')

    def destroy_node(self):
        try:
            self._notifier.stop()
            self._send_can(0, 0, 0, 0)
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
