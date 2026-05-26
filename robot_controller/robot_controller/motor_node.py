"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK → CAN 8-바이트 4-휠 모터 명령 변환기

[하드웨어 구성]
  바퀴 유형: 메카넘 X-구성 (Mecanum X-configuration)
  STM32 firmware: 투명 패스스루 (Transparent Pass-Through)
    → 내부 부호 보정 없음. 모든 IK 및 방향 보정은 Python 담당.

[메카넘 X-구성 IK 공식]
  표준 메카넘 X-구성 역기구학 행렬:
    FL = Vx + Vy - Wz
    FR = Vx - Vy + Wz
    RL = Vx - Vy - Wz
    RR = Vx + Vy + Wz

  Vy (스트레이핑) 강제 0 적용:
    FL = Vx + 0 - Wz = Vx - Wz
    FR = Vx - 0 + Wz = Vx + Wz
    RL = Vx - 0 - Wz = Vx - Wz
    RR = Vx + 0 + Wz = Vx + Wz

[하드웨어 역결선 보정 (STM32 투명 패스스루)]
  전륜 (htim1): 양수 = 전진 (정상 결선)
  후륜 (htim2): 역결선 → 양수 = 후진 → Python이 부호 반전 담당

  STM32 CAN 전송 최종값:
    FL_can = +(Vx - Wz) × MAX_SPEED  (전륜-좌: 보정 불필요)
    FR_can = +(Vx + Wz) × MAX_SPEED  (전륜-우: 보정 불필요)
    RL_can = -(Vx - Wz) × MAX_SPEED  (후륜-좌: 역결선 보정 → 부호 반전)
    RR_can = -(Vx + Wz) × MAX_SPEED  (후륜-우: 역결선 보정 → 부호 반전)

[동작 검증 (Vy=0)]
  W 전진 (Vx=+1): FL=+N  FR=+N  RL=-N  RR=-N → 4바퀴 모두 전진 방향 ✓
  A 좌회전 (Wz=+1): FL=-N FR=+N  RL=+N  RR=-N → 좌측 후진, 우측 전진 ✓
  D 우회전 (Wz=-1): FL=+N FR=-N  RL=-N  RR=+N → 좌측 전진, 우측 후진 ✓

[CAN 프로토콜]
  8바이트 Big-Endian: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]

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

        # ── 구독 ─────────────────────────────────────────────────
        self._sub_keyboard = self.create_subscription(
            Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS,
        )
        self._sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel_nav2', self._nav2_cb, _CMD_VEL_QOS,
        )
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )
        self._sub_estop = self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10
        )

        # ── 게시자 ───────────────────────────────────────────────
        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)

        self.get_logger().info(
            'MotorNode 준비 완료 (Mecanum X-config IK + 후륜 역결선 보정)\n'
            '  IK: FL=Vx-Wz, FR=Vx+Wz (Vy=0 강제)\n'
            '  CAN: FL_can=+FL, FR_can=+FR, RL_can=-FL, RR_can=-FR\n'
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
            self._send_can(0, 0, 0, 0)   # 모드 전환 즉시 정지

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
        with self._mode_lock:
            if self._mode != 'AUTO':
                return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0)
                return
        self._apply_twist(msg)

    # ──────────────────────────────────────────────────────────────
    # ── Mecanum O-구성 IK → 4-휠 CAN 명령 변환 ───────────────────
    # ──────────────────────────────────────────────────────────────
    def _apply_twist(self, msg: Twist) -> None:
        """
        ROS 2 Twist → 메카넘 O-구성 IK → STM32 CAN 속도값 변환.

        [하드웨어 분석 — 실험 데이터 기반]

        Test 1: CAN[1,1,1,1] → 제자리 회전
          → 이 로봇은 Mecanum O-구성 (O-config) 동작
          → 표준 X-config에서 [1,1,1,1] = 전진이지만, 이 로봇은 회전 발생

        Test 2: CAN[FL=+, RL=-, FR=-, RR=+] → 직진 전진
          → 하드웨어 역결선 확인:
               FL: 양수 = 전진 (정상 결선)
               FR: 음수 = 전진 (역결선)
               RL: 음수 = 전진 (역결선)
               RR: 양수 = 전진 (정상 결선)

        [최종 IK 공식 — O-구성 + 역결선 보정 + ROS2 부호 규칙]

          fl_can = +(Vx - Wz) × N
          fr_can = -(Vx + Wz) × N   ← FR 역결선: 부호 반전
          rl_can = -(Vx - Wz) × N   ← RL 역결선: 부호 반전
          rr_can = +(Vx + Wz) × N

        [동작 검증]
          W 전진  (Vx=+1): FL=+N, FR=-N, RL=-N, RR=+N → 4바퀴 전진 ✓
          A 좌회전(Wz=+1): FL=-N, FR=-N, RL=+N, RR=+N → 좌측 후진, 우측 전진(CCW) ✓
          D 우회전(Wz=-1): FL=+N, FR=+N, RL=-N, RR=-N → 좌측 전진, 우측 후진(CW) ✓
          S 후진  (Vx=-1): FL=-N, FR=+N, RL=+N, RR=-N → 4바퀴 후진 ✓
        """
        # ── 입력 파싱 및 클램프 [-1.0, +1.0] ─────────────────────
        Vx = max(-1.0, min(1.0, float(msg.linear.x)))
        Vy = 0.0   # 스트레이핑 강제 비활성화 (linear.y 완전 무시)
        Wz = max(-1.0, min(1.0, float(msg.angular.z)))

        N = self._max_speed   # 9999

        # ── O-구성 IK + 하드웨어 역결선 보정 ─────────────────────
        # FL/RR: 정상 결선 → 부호 그대로
        # FR/RL: 역결선 → 부호 반전 (STM32 투명 패스스루이므로 Python 담당)
        fl_can = int( (Vx - Wz) * N)   # 전륜-좌: 정상 결선
        fr_can = int(-(Vx + Wz) * N)   # 전륜-우: 역결선 보정
        rl_can = int(-(Vx - Wz) * N)   # 후륜-좌: 역결선 보정
        rr_can = int( (Vx + Wz) * N)   # 후륜-우: 정상 결선

        # 최종 클램프 (정수 범위 보장)
        fl_can = max(-N, min(N, fl_can))
        fr_can = max(-N, min(N, fr_can))
        rl_can = max(-N, min(N, rl_can))
        rr_can = max(-N, min(N, rr_can))

        self._send_can(fl_can, fr_can, rl_can, rr_can)
        self.get_logger().debug(
            f'[IK] Vx={Vx:+.3f} Wz={Wz:+.3f} | '
            f'FL={fl_can:+6d} FR={fr_can:+6d} RL={rl_can:+6d} RR={rr_can:+6d}'
        )

    # ──────────────────────────────────────────────────────────────
    # ── CAN 전송 ──────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송.
        형식: [FL(2B)][FR(2B)][RL(2B)][RR(2B)]
        STM32 Motor_Drive(fl, fr, rl, rr) 직접 호출.
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
