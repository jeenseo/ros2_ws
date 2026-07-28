"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK + Hybrid Encoder/Command Odometry
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
_MECANUM_L         = (_TRACK_WIDTH_M + _WHEELBASE_M) / 2.0  # 0.505 m

# 속도 한계 (CAN 정규화 기준)
_MAX_VX            = 0.95   # m/s (최대 직진)
_MAX_VY            = 0.95   # m/s (최대 스트레이프)
_MAX_WZ            = 1.88   # rad/s (최대 회전)
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

        channel          = self.get_parameter('can_channel').value
        self._can_id     = self.get_parameter('can_id').value
        self._max_speed  = self.get_parameter('max_speed').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        odom_topic       = self.get_parameter('odom_topic').value

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
        self._vel_x          = 0.0   
        self._vel_y          = 0.0   
        self._vel_yaw        = 0.0   
        self._last_fb_time   = None  # 타입 힌팅 에러 방지

        # 마지막 Vy 명령값 (하이브리드 오도메트리용)
        self._cmd_vy     = 0.0       
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

        # ── ROS 구독/게시 ─────────────────────────────────────────
        self._sub_keyboard = self.create_subscription(Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS)
        self._sub_nav2     = self.create_subscription(Twist, '/cmd_vel_nav2', self._nav2_cb, _CMD_VEL_QOS)
        self._sub_mode     = self.create_subscription(String, '/mode', self._mode_cb, 10)
        self._sub_estop    = self.create_subscription(Bool, '/e_stop', self._estop_cb, 10)
        self._pub_estop_ack= self.create_publisher(Bool, '/e_stop_ack', 10)
        self._pub_odom     = self.create_publisher(Odometry, odom_topic, 10)

        self.get_logger().info(f'MotorNode (Mecanum) 준비 완료. Odom -> {odom_topic}')

    # ══════════════════════════════════════════════════════════════
    # CAN RX: STM32 엔코더 피드백 + 하이브리드 오도메트리
    # ══════════════════════════════════════════════════════════════

    def _can_rx_callback(self, msg: can.Message) -> None:
        # 🚀 [핵심] 백그라운드 스레드 에러를 터미널에 띄우기 위한 철벽 안전망
        try:
            if msg.arbitration_id != _CAN_RX_FB_ID:
                return
            if len(msg.data) < 4:
                return

            left_ticks, right_ticks = struct.unpack('>hh', msg.data[:4])
            right_ticks = -right_ticks   

            now = time.monotonic()
            with self._odom_lock:
                if self._last_fb_time is None:
                    self._last_fb_time = now
                    return
                dt = now - self._last_fb_time
                if dt < 0.001:  # 0으로 나누기 방지
                    return
                self._last_fb_time = now

            dist_left  = left_ticks  * _METER_PER_TICK
            dist_right = right_ticks * _METER_PER_TICK

            delta_x_robot = (dist_left + dist_right) * 0.5
            delta_yaw     = (dist_right - dist_left) / (_TRACK_WIDTH_M + _WHEELBASE_M)

            with self._cmd_lock:
                cmd_vy = self._cmd_vy
            delta_y_robot = cmd_vy * dt   

            with self._odom_lock:
                mid_yaw = self._pose_yaw + delta_yaw * 0.5

                self._pose_x   += (delta_x_robot * math.cos(mid_yaw) - delta_y_robot * math.sin(mid_yaw))
                self._pose_y   += (delta_x_robot * math.sin(mid_yaw) + delta_y_robot * math.cos(mid_yaw))
                self._pose_yaw += delta_yaw

                self._vel_x   = delta_x_robot / dt
                self._vel_y   = delta_y_robot / dt
                self._vel_yaw = delta_yaw / dt

                x, y, yaw = self._pose_x, self._pose_y, self._pose_yaw
                vx, vy, wz = self._vel_x, self._vel_y, self._vel_yaw

            stamp = self.get_clock().now().to_msg()
            self._publish_odom(x, y, yaw, vx, vy, wz, stamp)

        except Exception as e:
            # 에러가 발생하면 무조건 빨간 글씨로 띄워라!
            self.get_logger().error(f'[🚨 긴급] CAN RX 스레드 암살됨! 원인: {repr(e)}')

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

        # 🚀 [에러 방어] 리스트를 통째로 할당하여 Tuple 할당 에러(TypeError) 원천 차단
        cov_pose = [0.0] * 36
        cov_pose[0]  = 0.05   
        cov_pose[7]  = 0.20   
        cov_pose[35] = 0.10   
        odom.pose.covariance = cov_pose

        cov_twist = [0.0] * 36
        cov_twist[0]  = 0.01
        cov_twist[7]  = 0.05
        cov_twist[35] = 0.05
        odom.twist.covariance = cov_twist

        self._pub_odom.publish(odom)

    # ══════════════════════════════════════════════════════════════
    # 메카넘 IK: cmd_vel → 4바퀴 독립 CAN 명령
    # ══════════════════════════════════════════════════════════════

    def _apply_twist(self, msg: Twist) -> None:
        v_fl_ms = msg.linear.x - msg.linear.y - (msg.angular.z * _MECANUM_L)
        v_fr_ms = msg.linear.x + msg.linear.y + (msg.angular.z * _MECANUM_L)
        v_rl_ms = msg.linear.x + msg.linear.y - (msg.angular.z * _MECANUM_L)
        v_rr_ms = msg.linear.x - msg.linear.y + (msg.angular.z * _MECANUM_L)

        v_fl = v_fl_ms / _MAX_VX
        v_fr = v_fr_ms / _MAX_VX
        v_rl = v_rl_ms / _MAX_VX
        v_rr = v_rr_ms / _MAX_VX

        max_val = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        v_fl /= max_val
        v_fr /= max_val
        v_rl /= max_val
        v_rr /= max_val

        N = self._max_speed
        fl_can = int(max(-N, min(N,  v_fl * N)))
        fr_can = int(max(-N, min(N,  v_fr * N)))
        rl_can = int(max(-N, min(N,  v_rl * N)))
        rr_can = int(max(-N, min(N,  v_rr * N)))

        with self._cmd_lock:
            self._cmd_vy = msg.linear.y

        self._send_can(fl_can, fr_can, rl_can, rr_can)

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
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        try:
            self._bus.send(can.Message(arbitration_id=self._can_id, data=data, is_extended_id=False))
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