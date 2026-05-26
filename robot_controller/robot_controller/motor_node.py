"""
motor_node.py
=============
ROS 2 Node: Skid-Steer IK → CAN 8-바이트 4-휠 모터 명령 변환기

[스키드-스티어 역기구학 (IK)]

  STM32 motor.c는 이제 투명한 패스스루(Transparent Pass-Through).
  전륜(htim1)은 양수=전진으로 정상 결선.
  후륜(htim2)은 하드웨어 역결선 → Python이 부호 반전을 담당.

  ROS 2 Twist 입력:
    linear.x  > 0 → 전진
    linear.x  < 0 → 후진
    angular.z > 0 → 좌회전 (CCW)
    angular.z < 0 → 우회전 (CW)
    linear.y  = 무시 (스키드-스티어는 스트레이핑 불가)

  4-휠 IK 계산:
    v_left  = linear - angular   (angular>0 = 좌회전 = 좌측 후진)
    v_right = linear + angular   (angular>0 = 좌회전 = 우측 전진)

    FL = +v_left   × MAX_SPEED  (전륜-좌: 정상 결선)
    FR = +v_right  × MAX_SPEED  (전륜-우: 정상 결선)
    RL = -v_left   × MAX_SPEED  (후륜-좌: 역결선 보정 → 부호 반전)
    RR = -v_right  × MAX_SPEED  (후륜-우: 역결선 보정 → 부호 반전)

  검증:
    전진 (linear=+1):  FL=+MAX, FR=+MAX, RL=-MAX, RR=-MAX ✓
    좌회전 (angular=+1): FL=-MAX, FR=+MAX, RL=+MAX, RR=-MAX ✓
    우회전 (angular=-1): FL=+MAX, FR=-MAX, RL=-MAX, RR=+MAX ✓

[CAN 프로토콜]
  8바이트 Big-Endian:  [FL(2)] [FR(2)] [RL(2)] [RR(2)]
  STM32 수신 후 Motor_Drive(fl, fr, rl, rr) 호출

[cmd_vel Mux]
  /cmd_vel_keyboard → MANUAL 모드에서만 CAN 전송
  /cmd_vel_nav2     → AUTO   모드에서만 CAN 전송
    (launch remapping: /cmd_vel_nav2 ← /cmd_vel from Nav2)

[E-Stop]
  /e_stop (Bool) True → CAN 즉시 정지 + 모든 cmd_vel 무시
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
        self._sub_keyboard = self.create_subscription(
            Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS,
        )

        # ── 구독: /cmd_vel_nav2 (Nav2 출력, launch remapping) ─────
        self._sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel_nav2', self._nav2_cb, _CMD_VEL_QOS,
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
            'MotorNode 준비 완료 (Skid-Steer IK + 후륜 역결선 보정)\n'
            '  IK: v_left=linear-angular, v_right=linear+angular\n'
            '  RL/RR 부호 반전 (하드웨어 역결선 보정)\n'
            '  /cmd_vel_keyboard → MANUAL 모드\n'
            '  /cmd_vel_nav2     → AUTO   모드'
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
    # ── /e_stop 콜백 ──────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev              = self._is_estopped
            self._is_estopped = msg.data

        if msg.data and not prev:
            self.get_logger().warn('[E-STOP] 발동! CAN 즉시 정지')
            self._send_can(0, 0, 0, 0)
            ack = Bool(); ack.data = True
            self._pub_estop_ack.publish(ack)
        elif not msg.data and prev:
            self.get_logger().info('[E-STOP] 해제')
            ack = Bool(); ack.data = False
            self._pub_estop_ack.publish(ack)

    # ──────────────────────────────────────────────────────────────
    # ── /cmd_vel_keyboard 콜백 (MANUAL 전용) ─────────────────────
    # ──────────────────────────────────────────────────────────────
    def _keyboard_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'MANUAL':
                return   # AUTO 모드: 키보드 완전 차단
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return
        self._apply_twist(msg)

    # ──────────────────────────────────────────────────────────────
    # ── /cmd_vel_nav2 콜백 (AUTO 전용) ───────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _nav2_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'AUTO':
                return   # MANUAL 모드: Nav2 완전 차단
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return
        self._apply_twist(msg)

    # ──────────────────────────────────────────────────────────────
    # ── Skid-Steer IK → 4-휠 CAN 명령 변환 ──────────────────────
    # ──────────────────────────────────────────────────────────────
    def _apply_twist(self, msg: Twist) -> None:
        """
        ROS 2 Twist → 4-휠 스키드-스티어 CAN 속도값 변환.

        입력:
          msg.linear.x  : 전진/후진 속도 (-1.0 ~ +1.0 정규화)
          msg.angular.z : 회전 속도   (-1.0 ~ +1.0 정규화)
          msg.linear.y  : 무시 (스키드-스티어 불가)

        스키드-스티어 IK:
          v_left  = linear - angular
          v_right = linear + angular

        STM32 Motor_Drive CAN 값:
          FL = +v_left  * MAX_SPEED  (전륜-좌: 정상 결선)
          FR = +v_right * MAX_SPEED  (전륜-우: 정상 결선)
          RL = -v_left  * MAX_SPEED  (후륜-좌: 역결선 → 부호 반전)
          RR = -v_right * MAX_SPEED  (후륜-우: 역결선 → 부호 반전)
        """
        # 입력 클램프 [-1.0, +1.0]
        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))
        # linear.y 는 완전 무시 (스키드-스티어)

        # ── 좌/우 측 속도 계산 (정규화) ──────────────────────────
        v_left  = linear - angular   # angular > 0 (좌회전) → 좌측 속도 감소
        v_right = linear + angular   # angular > 0 (좌회전) → 우측 속도 증가

        # 합산 후 클램프 (|v| > 1.0 가능)
        v_left  = max(-1.0, min(1.0, v_left))
        v_right = max(-1.0, min(1.0, v_right))

        N = self._max_speed   # 9999

        # ── 4-휠 CAN 속도값 계산 ─────────────────────────────────
        fl = int( v_left  * N)   # 전륜-좌: 그대로 적용
        fr = int( v_right * N)   # 전륜-우: 그대로 적용
        rl = int(-v_left  * N)   # 후륜-좌: 역결선 보정 (부호 반전)
        rr = int(-v_right * N)   # 후륜-우: 역결선 보정 (부호 반전)

        # 최종 클램프 (정수 범위 보장)
        fl = max(-N, min(N, fl))
        fr = max(-N, min(N, fr))
        rl = max(-N, min(N, rl))
        rr = max(-N, min(N, rr))

        self._send_can(fl, fr, rl, rr)
        self.get_logger().debug(
            f'[IK] lin={linear:+.3f} ang={angular:+.3f} | '
            f'FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

    # ──────────────────────────────────────────────────────────────
    # ── CAN 전송 ──────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송.
        형식: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]
        STM32 Motor_Drive(fl, fr, rl, rr) 호출됨.
        """
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
