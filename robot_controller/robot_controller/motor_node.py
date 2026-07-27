"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK + Hybrid Encoder/Command Odometry

[아키텍처 규칙]
  이 노드는 절대 TF를 발행하지 않습니다.
  odom → base_footprint TF는 ekf_node (robot_localization) 전담.

[CAN TX: 0x123] Pi → STM32
  8바이트 Big-Endian: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]
  단위: -9999 ~ +9999 (정규화 속도)

[CAN RX: 0x124] STM32 → Pi
  4바이트 Big-Endian: [Left_delta(2B)][Right_delta(2B)]
  단위: 엔코더 누산 틱 (STM32 전송 후 0 리셋)
  하드웨어: 우측 모터 역방향 → right_ticks × (-1)

[오도메트리 전략: 하이브리드]
  X축 / Yaw: 2-엔코더 기반 (신뢰도 높음)
    - Left  Enc avg = (v_FL + v_RL)/2 = Vx - Wz  (Vy 항 소거됨)
    - Right Enc avg = (v_FR + v_RR)/2 = Vx + Wz  (Vy 항 소거됨)
    → delta_x   = (dist_L + dist_R) / 2
    → delta_yaw = (dist_R - dist_L) / TRACK_WIDTH

  Y축: 명령 속도 적분 (가상 오도메트리)
    → delta_y_robot = last_commanded_vy × dt

  전역 포즈 갱신:
    x   += delta_x × cos(yaw) - delta_y_robot × sin(yaw)
    y   += delta_x × sin(yaw) + delta_y_robot × cos(yaw)
    yaw += delta_yaw

[오도메트리 출력]
  /odom_motor (nav_msgs/Odometry) — EKF odom1 소스, TF 없음

[메카넘 IK: X-구성]
  l = (TRACK_WIDTH + WHEELBASE) / 2 = 0.505 m
  FL = Vx - Vy - Wz × l
  FR = Vx + Vy + Wz × l
  RL = Vx + Vy - Wz × l
  RR = Vx - Vy + Wz × l

[cmd_vel Mux]
  /cmd_vel_keyboard → MANUAL 모드
  /cmd_vel_nav2     → AUTO   모드

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


# ── QoS ─────────────────────────────────────────────────────────
_CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# ── 물리 상수 ─────────────────────────────────────────────────────
_WHEEL_DIAMETER_M  = 0.12                              # 바퀴 직경 (m)
_WHEEL_CIRCUM_M    = math.pi * _WHEEL_DIAMETER_M       # 바퀴 둘레 (m)
_ENCODER_CPR       = 1404                              # 엔코더 CPR
_TRACK_WIDTH_M     = 0.51                              # 좌우 트랙 폭 (m)
_WHEELBASE_M       = 0.50                              # 전후 휠베이스 (m)
_METER_PER_TICK    = _WHEEL_CIRCUM_M / _ENCODER_CPR    # ≈ 0.0002685 m/tick

# 메카넘 IK 거리 상수: l = (track_width + wheelbase) / 2
_MECANUM_L         = (_TRACK_WIDTH_M + _WHEELBASE_M) / 2.0   # 0.505 m

# 속도 한계 (CAN 정규화 기준)
_MAX_SPEED         = 9999   # CAN 최대값

# CAN ID
_CAN_TX_ID         = 0x123
_CAN_RX_FB_ID      = 0x124


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      _CAN_TX_ID)
        self.declare_parameter('max_speed',   _MAX_SPEED)
        self.declare_parameter('odom_frame',  'odom')
        self.declare_parameter('base_frame',  'base_footprint')
        self.declare_parameter('odom_topic',  '/odom_motor')

        # [하드웨어 스펙] 모터가 9999(100%) 출력일 때 나오는 로봇의 물리적 최대 속도
        # Nav2의 m/s 명령을 0~9999 스케일로 번역하는 역할을 수행합니다.
        self.declare_parameter('hw_max_vx', 0.95)  # m/s
        self.declare_parameter('hw_max_vy', 0.95)  # m/s
        self.declare_parameter('hw_max_wz', 1.88)  # rad/s (0.95 / 0.505)

        channel          = self.get_parameter('can_channel').value
        self._can_id     = self.get_parameter('can_id').value
        self._max_speed  = self.get_parameter('max_speed').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        odom_topic       = self.get_parameter('odom_topic').value

        self._hw_max_vx  = self.get_parameter('hw_max_vx').value
        self._hw_max_vy  = self.get_parameter('hw_max_vy').value
        self._hw_max_wz  = self.get_parameter('hw_max_wz').value

        # ── 모드 / E-Stop ─────────────────────────────────────────
        self._mode        = 'MANUAL'
        self._mode_lock   = threading.Lock()
        self._is_estopped = False
        self._estop_lock  = threading.Lock()

        # ── 오도메트리 상태 ───────────────────────────────────────
        self._odom_lock      = threading.Lock()
        self._pose_x         = 0.0
        self._pose_y         = 0.0
        self._pose_yaw       = 0.0
        self._vel_x          = 0.0   # 로봇 프레임 선속도 X (m/s)
        self._vel_y          = 0.0   # 로봇 프레임 선속도 Y (명령값)
        self._vel_yaw        = 0.0   # 각속도 (rad/s)
        self._last_fb_time: float | None = None

        # 마지막 Vy 명령값 (하이브리드 오도메트리용)
        self._cmd_vy     = 0.0       # 로봇 프레임 Y 명령속도 (m/s)
        self._cmd_lock   = threading.Lock()

        # ── CAN 버스 ──────────────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(
                f'CAN 초기화 ({channel}) TX=0x{self._can_id:03X} RX=0x{_CAN_RX_FB_ID:03X}'
            )
        except Exception as exc:
            self.get_logger().fatal(f'CAN 초기화 실패: {exc}')
            raise

        self._notifier = can.Notifier(self._bus, [self._can_rx_callback])

        # ── ROS 구독 ──────────────────────────────────────────────
        self._sub_keyboard = self.create_subscription(
            Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS)
        self._sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel_nav2', self._nav2_cb, _CMD_VEL_QOS)
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10)
        self._sub_estop = self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10)

        # ── ROS 게시자 ────────────────────────────────────────────
        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)
        self._pub_odom      = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(
            'MotorNode (Mecanum) 준비 완료\n'
            f'  IK: FL=Vx-Vy-Wz·L, FR=Vx+Vy+Wz·L, RL=Vx+Vy-Wz·L, RR=Vx-Vy+Wz·L\n'
            f'  Odom: 하이브리드 (Enc→X/Yaw, Cmd→Y)\n'
            f'  → {odom_topic}'
        )

    # ══════════════════════════════════════════════════════════════
    # CAN RX: STM32 엔코더 피드백 + 하이브리드 오도메트리
    # ══════════════════════════════════════════════════════════════

    def _can_rx_callback(self, msg: can.Message) -> None:
        """
        CAN 0x124 수신 → 하이브리드 오도메트리 계산 → /odom_motor 게시.

        [하이브리드 오도메트리 수학]
        메카넘 X-구성에서 좌/우 인코더 평균 분석:
          Left  avg = (v_FL + v_RL)/2 = (Vx - Vy - Wz·L)/2 + (Vx + Vy - Wz·L)/2
                    = Vx - Wz  (Vy 항 소거)
          Right avg = (v_FR + v_RR)/2 = (Vx + Vy + Wz·L)/2 + (Vx - Vy + Wz·L)/2
                    = Vx + Wz  (Vy 항 소거)

        → 2엔코더로 X와 Yaw 완벽 측정 가능
        → Y는 마지막 Vy 명령 × dt로 가상 추정
        """
        if msg.arbitration_id != _CAN_RX_FB_ID:
            return
        if len(msg.data) < 4:
            return

        # 엔코더 틱 디코딩 (Big-Endian int16)
        left_ticks, right_ticks = struct.unpack('>hh', msg.data[:4])
        right_ticks = -right_ticks   # 우측 역방향 보정

        # dt 계산
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

        # ── X / Yaw: 엔코더 기반 (신뢰도 높음) ───────────────────
        # 메카넘 평균 인코더: Vy 소거, Vx와 Wz만 남음
        delta_x_robot = (dist_left + dist_right) * 0.5
        delta_yaw     = (dist_right - dist_left) / _TRACK_WIDTH_M

        # ── Y: 마지막 Vy 명령 적분 (가상 오도메트리) ─────────────
        with self._cmd_lock:
            cmd_vy = self._cmd_vy
        delta_y_robot = cmd_vy * dt   # 로봇 프레임 Y 변위

        # ── 전역 포즈 갱신 ────────────────────────────────────────
        with self._odom_lock:
            mid_yaw = self._pose_yaw + delta_yaw * 0.5

            # 로봇 프레임 → 전역 프레임 변환
            self._pose_x   += (delta_x_robot * math.cos(mid_yaw)
                                - delta_y_robot * math.sin(mid_yaw))
            self._pose_y   += (delta_x_robot * math.sin(mid_yaw)
                                + delta_y_robot * math.cos(mid_yaw))
            self._pose_yaw += delta_yaw

            # 속도 추정
            self._vel_x   = delta_x_robot / dt
            self._vel_y   = delta_y_robot / dt
            self._vel_yaw = delta_yaw / dt

            x   = self._pose_x
            y   = self._pose_y
            yaw = self._pose_yaw
            vx  = self._vel_x
            vy  = self._vel_y
            wz  = self._vel_yaw

        stamp = self.get_clock().now().to_msg()
        self._publish_odom(x, y, yaw, vx, vy, wz, stamp)

    # ══════════════════════════════════════════════════════════════
    # 오도메트리 게시
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        h = yaw * 0.5
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def _publish_odom(self, x, y, yaw, vx, vy, wz, stamp) -> None:
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz

        # 공분산 (X, Y, Yaw 대각)
        # Y축은 명령 기반이므로 신뢰도 낮게 설정
        odom.pose.covariance[0]  = 0.05   # x (encoder)
        odom.pose.covariance[7]  = 0.20   # y (command-integrated, 낮은 신뢰도)
        odom.pose.covariance[35] = 0.10   # yaw (encoder)
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.05
        odom.twist.covariance[35] = 0.05

        self._pub_odom.publish(odom)

    # ══════════════════════════════════════════════════════════════
    # 메카넘 IK: cmd_vel → 4바퀴 독립 CAN 명령
    # ══════════════════════════════════════════════════════════════

    def _apply_twist(self, msg: Twist) -> None:
        """
        표준 메카넘 X-구성 역기구학 (IK).

        물리 방정식 (IK):
          v_FL = Vx - Vy - Wz × l      l = (TRACK + WHEELBASE) / 2
          v_FR = Vx + Vy + Wz × l
          v_RL = Vx + Vy - Wz × l
          v_RR = Vx - Vy + Wz × l

        정규화:
          각 속도를 물리 최대값으로 나눠 [-1, 1] 범위로 변환 후
          조합 최대값(>1 가능)을 비례 클램프 → CAN 값 × N

        방향 보정:
          FL/RL: 정상 결선 (양수=전진)
          FR/RR: 역결선 보정 → CAN 음수 부호 적용 (motor.h DIR_INVERT 설정에 따라 조정)
        """
        # 입력 속도를 하드웨어 최대 물리 속도(0.95m/s) 스케일로 번역
        Vx = msg.linear.x  / self._hw_max_vx
        Vy = msg.linear.y  / self._hw_max_vy
        Wz = msg.angular.z / self._hw_max_wz

        # 메카넘 IK 거리 상수 (하드웨어 스펙 기준으로 정규화)
        l_norm = _MECANUM_L / self._hw_max_vx

        v_fl = Vx - Vy - Wz * l_norm
        v_fr = Vx + Vy + Wz * l_norm
        v_rl = Vx + Vy - Wz * l_norm
        v_rr = Vx - Vy + Wz * l_norm

        # 비례 정규화: 최대값이 1.0을 초과하면 전체 스케일 다운
        max_val = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        v_fl /= max_val
        v_fr /= max_val
        v_rl /= max_val
        v_rr /= max_val

        N = self._max_speed

        # CAN 값 변환 [-N, N]
        fl_can = int(max(-N, min(N,  v_fl * N)))
        fr_can = int(max(-N, min(N,  v_fr * N)))
        rl_can = int(max(-N, min(N,  v_rl * N)))
        rr_can = int(max(-N, min(N,  v_rr * N)))

        # 마지막 Vy 명령 저장 (하이브리드 오도메트리용)
        with self._cmd_lock:
            self._cmd_vy = msg.linear.y

        self._send_can(fl_can, fr_can, rl_can, rr_can)
        self.get_logger().debug(
            f'[IK] Vx={msg.linear.x:+.3f} Vy={msg.linear.y:+.3f} Wz={msg.angular.z:+.3f} '
            f'→ FL={fl_can:+6d} FR={fr_can:+6d} RL={rl_can:+6d} RR={rr_can:+6d}'
        )

    # ══════════════════════════════════════════════════════════════
    # cmd_vel Mux / E-Stop
    # ══════════════════════════════════════════════════════════════

    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old, self._mode = self._mode, msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')
            self._send_can(0, 0, 0, 0)
            with self._cmd_lock:
                self._cmd_vy = 0.0

    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev, self._is_estopped = self._is_estopped, msg.data
        if msg.data and not prev:
            self.get_logger().warn('[E-STOP] 발동!')
            self._send_can(0, 0, 0, 0)
            with self._cmd_lock:
                self._cmd_vy = 0.0
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

    # ══════════════════════════════════════════════════════════════
    # CAN TX
    # ══════════════════════════════════════════════════════════════

    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8바이트 Big-Endian CAN 프레임 전송 [FL][FR][RL][RR]."""
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