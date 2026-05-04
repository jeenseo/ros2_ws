"""
motor_node.py
=============
ROS 2 Node: /cmd_vel → CAN 8-바이트 4-휠 독립 모터 명령 변환기
"""

import struct
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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

        # ── 모드 상태 ─────────────────────────────────────────────
        self._mode      = 'MANUAL'   # 'MANUAL' | 'AUTO'
        self._mode_lock = threading.Lock()

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
        self.get_logger().info('MotorNode 준비 완료 (8바이트 CAN 프로토콜)')

    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old         = self._mode
            self._mode  = msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')

    # 주의: 여기서부터 들여쓰기(4칸)가 클래스 안쪽으로 정확히 맞아야 합니다.
    def _cmd_vel_cb(self, msg: Twist) -> None:
        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))

        linear_v  = linear  * self._max_speed
        angular_v = angular * self._max_speed

        with self._mode_lock:
            # AUTO 모드일 때만 회전 성분(angular)에 30% 감속 적용
            bias = 0.3 if self._mode == 'AUTO' else 1.0

        # ── 하드웨어 교차 배선 보정 맵핑 (Magic Formula) ─────────────
        # 물리적으로 꼬인 선을 파이썬에서 미리 꼬아서 보내 정상 작동 유도
        
        # fl: 원래 M_FL -> 실제 N_FL (역방향) 구동
        fl = int(linear_v + angular_v)
        
        # fr: 원래 M_FR -> 실제 N_RR 구동 (bias 적용)
        fr = int(-linear_v + (angular_v * bias))
        
        # rl: 원래 M_RL -> 실제 N_FR 구동 (bias 미적용)
        rl = int(linear_v - angular_v)
        
        # rr: 원래 M_RR -> 실제 N_RL (역방향) 구동 (bias 적용)
        rr = int(-linear_v - (angular_v * bias))
        # ─────────────────────────────────────────────────────────────

        # ── 클램프 (범위 제한) ───────────────────────────────────────
        N  = self._max_speed
        fl = max(-N, min(N, fl))
        fr = max(-N, min(N, fr))
        rl = max(-N, min(N, rl))
        rr = max(-N, min(N, rr))

        self._send_can(fl, fr, rl, rr)
        
        # 디버그용 출력: 터미널에 이 로그가 찍혀야 정상적으로 데이터가 나가는 것입니다.
        self.get_logger().debug(
            f'[CAN TX] FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

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
