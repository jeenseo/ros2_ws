"""
motor_node.py
=============
ROS 2 Node: /cmd_vel → CAN 8-바이트 4-휠 독립 모터 명령 변환기

E-Stop 추가 (Buzzing 방지):
  - /e_stop (Bool) 구독: True → CAN 즉시 정지 + /cmd_vel 무시
  - /e_stop_ack (Bool) 게시: Nav2에 정지 상태 통보
  - E-Stop 해제 시 자동으로 정상 모드 복귀

Buzzing 근본 원인:
  Nav2 DWB ─────→ /cmd_vel (전진 명령) ─┐
                                          ├→ 충돌 → PID Windup → 버즈
  avoidance_node → /cmd_vel (정지 명령) ─┘

해결책:
  avoidance_node가 /e_stop=True 게시
  → motor_node가 CAN 정지 + /cmd_vel 처리 중단
  → Nav2 명령도 무효화 → 충돌 해소
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import can


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
        self._mode          = 'MANUAL'
        self._mode_lock     = threading.Lock()
        self._is_estopped   = False
        self._estop_lock    = threading.Lock()

        # ── CAN 버스 초기화 ───────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(
                f'CAN 버스 초기화 완료 ({channel}, ID=0x{self._can_id:03X})'
            )
        except Exception as exc:
            self.get_logger().fatal(f'CAN 버스 초기화 실패: {exc}')
            raise

        # ── 구독 ─────────────────────────────────────────────────
        self._sub_cmd  = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )
        # E-Stop 토픽 구독 (avoidance_node 또는 안전 노드에서 게시)
        self._sub_estop = self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10
        )

        # ── 게시자 ───────────────────────────────────────────────
        # E-Stop 확인 토픽 (Nav2가 구독하여 명령 전송 중단 신호로 사용)
        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)
        # Zero velocity 역발행 (Nav2 속도 명령 초기화용)
        self._pub_cmd_flush = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('MotorNode 준비 완료 (8바이트 CAN + E-Stop)')

    # ── /mode 콜백 ────────────────────────────────────────────────
    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old        = self._mode
            self._mode = msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')

    # ── /e_stop 콜백 — Buzzing 방지 핵심 ─────────────────────────
    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev_state       = self._is_estopped
            self._is_estopped = msg.data

        if msg.data and not prev_state:
            # ── E-Stop 발동 ───────────────────────────────────────
            self.get_logger().warn('[E-STOP] 발동! CAN 즉시 정지, /cmd_vel 무효화')

            # 1. CAN 즉시 정지 신호
            self._send_can(0, 0, 0, 0)

            # 2. /cmd_vel에 zero Twist 역발행 → Nav2 속도 플러시
            zero_twist = Twist()
            self._pub_cmd_flush.publish(zero_twist)

            # 3. E-Stop ACK 게시 (Nav2 등 상위 노드에 통보)
            ack_msg      = Bool()
            ack_msg.data = True
            self._pub_estop_ack.publish(ack_msg)

        elif not msg.data and prev_state:
            # ── E-Stop 해제 ───────────────────────────────────────
            self.get_logger().info('[E-STOP] 해제. 정상 모드 복귀')
            ack_msg      = Bool()
            ack_msg.data = False
            self._pub_estop_ack.publish(ack_msg)

    # ── /cmd_vel 콜백 ─────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist) -> None:
        # ── E-Stop 활성 시 모든 cmd_vel 무시 ─────────────────────
        with self._estop_lock:
            if self._is_estopped:
                # 정지 유지 (zero 재전송)
                self._send_can(0, 0, 0, 0)
                return

        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))

        linear_v  = linear  * self._max_speed
        angular_v = angular * self._max_speed

        with self._mode_lock:
            bias = 0.3 if self._mode == 'AUTO' else 1.0

        # ── 하드웨어 교차 배선 보정 맵핑 ─────────────────────────
        fl = int(linear_v + angular_v)
        fr = int(-linear_v + (angular_v * bias))
        rl = int(linear_v - angular_v)
        rr = int(-linear_v - (angular_v * bias))

        # 클램프
        N  = self._max_speed
        fl = max(-N, min(N, fl))
        fr = max(-N, min(N, fr))
        rl = max(-N, min(N, rl))
        rr = max(-N, min(N, rr))

        self._send_can(fl, fr, rl, rr)
        self.get_logger().debug(
            f'[CAN TX] FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

    # ── CAN 전송 ──────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송."""
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        msg  = can.Message(
            arbitration_id=self._can_id,
            data=data,
            is_extended_id=False
        )
        try:
            self._bus.send(msg)
        except can.CanError as exc:
            self.get_logger().error(f'[CAN ERROR] {exc}')

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._send_can(0, 0, 0, 0)   # 긴급 정지
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
