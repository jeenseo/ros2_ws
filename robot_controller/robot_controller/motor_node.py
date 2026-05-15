"""
motor_node.py
=============
ROS 2 Node: cmd_vel → CAN 8-바이트 4-휠 독립 모터 명령 변환기

[수정] cmd_vel Mux 구현 — Manual/Auto 엄격한 제어권 분리
  - /cmd_vel_keyboard  구독 → MANUAL 모드에서만 CAN 전송
  - /cmd_vel_nav2      구독 → AUTO 모드에서만 CAN 전송
    (launch 파일 remapping: /cmd_vel_nav2 → /cmd_vel (Nav2 출력))
  - 모드 전환 즉시 정지 명령 전송 → 잔류 속도 제거

[수정] Sudden Surge 방지 — QoS depth=1
  - 구독 큐를 depth=1로 제한 → 항상 최신 명령만 처리
  - 경로 계획 중 쌓인 stale 메시지 즉시 폐기

E-Stop (Buzzing 방지):
  - /e_stop (Bool) True → CAN 즉시 정지 + 모든 cmd_vel 무시
  - /e_stop_ack (Bool) 게시 → Nav2에 정지 상태 통보

토픽 구조:
  keyboard_node → /cmd_vel_keyboard ─┐
                                      ├→ [mode-aware mux] → CAN (STM32)
  Nav2 controller → /cmd_vel ────────┘   (launch remapping으로 연결)
                    (/cmd_vel_nav2로 수신)
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import can


# ── cmd_vel QoS: depth=1 (항상 최신 명령만 처리, 큐 쌓임 방지) ──────────
_CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      0x123)
        self.declare_parameter('max_speed',   9999)

        channel         = self.get_parameter('can_channel').value
        self._can_id    = self.get_parameter('can_id').value
        self._max_speed = self.get_parameter('max_speed').value

        # ── 모드 & E-Stop 상태 ────────────────────────────────────
        self._mode        = 'MANUAL'
        self._mode_lock   = threading.Lock()
        self._is_estopped = False
        self._estop_lock  = threading.Lock()

        # ── CAN 버스 초기화 ───────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(
                f'CAN 버스 초기화 완료 ({channel}, ID=0x{self._can_id:03X})'
            )
        except Exception as exc:
            self.get_logger().fatal(f'CAN 버스 초기화 실패: {exc}')
            raise

        # ── 구독: /cmd_vel_keyboard (keyboard_node 출력) ─────────
        # MANUAL 모드에서만 CAN으로 전달
        self._sub_keyboard = self.create_subscription(
            Twist,
            '/cmd_vel_keyboard',
            self._keyboard_cb,
            _CMD_VEL_QOS,
        )

        # ── 구독: /cmd_vel_nav2 (Nav2 controller 출력) ───────────
        # AUTO 모드에서만 CAN으로 전달
        # launch 파일에서 remappings=[('/cmd_vel_nav2', '/cmd_vel')] 설정
        # → Nav2가 /cmd_vel에 게시 → motor_node는 /cmd_vel_nav2로 수신
        self._sub_nav2 = self.create_subscription(
            Twist,
            '/cmd_vel_nav2',
            self._nav2_cb,
            _CMD_VEL_QOS,
        )

        # ── 구독: /mode, /e_stop ──────────────────────────────────
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )
        self._sub_estop = self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10
        )

        # ── 게시자 ───────────────────────────────────────────────
        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)

        self.get_logger().info(
            'MotorNode 준비 완료 (8바이트 CAN + E-Stop + cmd_vel Mux)\n'
            '  /cmd_vel_keyboard → MANUAL 모드에서만 처리\n'
            '  /cmd_vel_nav2     → AUTO   모드에서만 처리 (Nav2 /cmd_vel remapped)'
        )

    # ──────────────────────────────────────────────────────────────
    # ── /mode 콜백 ────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old        = self._mode
            self._mode = msg.data

        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')
            # 모드 전환 순간 즉시 정지 → 잔류 속도 제거
            self._send_can(0, 0, 0, 0)

    # ──────────────────────────────────────────────────────────────
    # ── /e_stop 콜백 — Buzzing 방지 ──────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev_state        = self._is_estopped
            self._is_estopped = msg.data

        if msg.data and not prev_state:
            self.get_logger().warn('[E-STOP] 발동! CAN 즉시 정지')
            self._send_can(0, 0, 0, 0)
            ack = Bool()
            ack.data = True
            self._pub_estop_ack.publish(ack)

        elif not msg.data and prev_state:
            self.get_logger().info('[E-STOP] 해제. 정상 모드 복귀')
            ack = Bool()
            ack.data = False
            self._pub_estop_ack.publish(ack)

    # ──────────────────────────────────────────────────────────────
    # ── /cmd_vel_keyboard 콜백 (MANUAL 전용) ─────────────────────
    # ──────────────────────────────────────────────────────────────
    def _keyboard_cb(self, msg: Twist) -> None:
        # Rule A: MANUAL 모드에서만 처리 — Auto 명령 완전 차단
        with self._mode_lock:
            if self._mode != 'MANUAL':
                return

        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return

        self._apply_twist(msg)

    # ──────────────────────────────────────────────────────────────
    # ── /cmd_vel_nav2 콜백 (AUTO 전용) ───────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _nav2_cb(self, msg: Twist) -> None:
        # Rule B: AUTO 모드에서만 처리 — Manual 조작 중 Nav2 명령 완전 차단
        with self._mode_lock:
            if self._mode != 'AUTO':
                return

        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return

        self._apply_twist(msg)

    # ──────────────────────────────────────────────────────────────
    # ── Twist → 4-휠 스키드-스티어 CAN 변환 ──────────────────────
    # ──────────────────────────────────────────────────────────────
    def _apply_twist(self, msg: Twist) -> None:
        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))

        linear_v  = linear  * self._max_speed
        angular_v = angular * self._max_speed

        # 스키드-스티어 역기구학
        # 좌측 바퀴: linear + angular, 우측 바퀴: linear - angular
        fl = int(linear_v + angular_v)
        fr = int(-linear_v + angular_v)
        rl = int(linear_v - angular_v)
        rr = int(-linear_v - angular_v)

        N  = self._max_speed
        fl = max(-N, min(N, fl))
        fr = max(-N, min(N, fr))
        rl = max(-N, min(N, rl))
        rr = max(-N, min(N, rr))

        self._send_can(fl, fr, rl, rr)
        self.get_logger().debug(
            f'[CAN TX] FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

    # ──────────────────────────────────────────────────────────────
    # ── CAN 전송 ──────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송."""
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        msg  = can.Message(
            arbitration_id=self._can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg)
        except can.CanError as exc:
            self.get_logger().error(f'[CAN ERROR] {exc}')

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        try:
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
