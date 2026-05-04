"""
keyboard_node.py
================
ROS 2 Node: 터미널 키보드 제어 (Wayland 호환, 비동기 멀티키 지원)

개선 사항:
  - WASD 동시 입력 지원 (W+D = 전진+우회전 등)
  - 버퍼 드레인: 한 폴에서 모든 가용 바이트 소비 → 멀티키 집합 구성
  - 'b' 토글: Normal (2000) ↔ Boost (4000) 속도 전환
  - 방향키(↑↓←→)도 WASD와 동일하게 지원

키 바인딩:
  m / M   : MANUAL ↔ AUTO 모드 전환
  w / ↑   : 전진 기여 (+linear)
  s / ↓   : 후진 기여 (-linear)
  a / ←   : 좌회전 기여 (+angular)
  d / →   : 우회전 기여 (-angular)
  b / B   : 속도 Normal(2000) ↔ Boost(4000) 토글
  Ctrl+C  : 종료

복합 키 예:
  W + D   → linear=+spd, angular=-spd  (전진 + 우회전)
  W + A   → linear=+spd, angular=+spd  (전진 + 좌회전)

토픽 출력:
  /cmd_vel  (geometry_msgs/Twist) — MANUAL 모드일 때만 게시
  /mode     (std_msgs/String)     — "MANUAL" | "AUTO"

파라미터:
  normal_speed  float  0.2002  (= 2000/9999)
  boost_speed   float  0.4001  (= 4000/9999)
"""

import atexit
import os
import select
import signal
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── 속도 레벨 ─────────────────────────────────────────────────────
SPEED_NORMAL = 2000 / 9999   # ≈ 0.2002
SPEED_BOOST  = 4000 / 9999   # ≈ 0.4001


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('normal_speed', SPEED_NORMAL)
        self.declare_parameter('boost_speed',  SPEED_BOOST)

        self._spd_normal = self.get_parameter('normal_speed').value
        self._spd_boost  = self.get_parameter('boost_speed').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist,  '/cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, '/mode',    10)

        # ── 공유 상태 ─────────────────────────────────────────────
        self._mode       = 'MANUAL'
        self._speed_mode = 'normal'              # 'normal' | 'boost'
        self._keys       = set()                 # 현재 활성 키 집합
        self._key_lock   = threading.Lock()
        self._orig_term  = None

        # ── 키보드 백그라운드 스레드 ──────────────────────────────
        kb_t = threading.Thread(target=self._keyboard_loop, daemon=True)
        kb_t.start()

        # ── 20Hz 명령 게시 타이머 ─────────────────────────────────
        self._timer = self.create_timer(1.0 / 20.0, self._publish_cmd)

        self.get_logger().info(
            'KeyboardNode 준비 완료\n'
            '  WASD/방향키: 이동  |  m: MANUAL↔AUTO  |  b: Normal↔Boost  |  Ctrl+C: 종료'
        )
        self._publish_mode()

    # ── 터미널 복원 ────────────────────────────────────────────────
    def _restore_terminal(self) -> None:
        if self._orig_term is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term
                )
            except Exception:
                pass
            self._orig_term = None

    # ── 모드 게시 헬퍼 ───────────────────────────────────────────
    def _publish_mode(self) -> None:
        msg      = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    # ── 키보드 루프 (백그라운드 데몬 스레드) ─────────────────────
    def _keyboard_loop(self) -> None:
        """
        비동기 멀티키 입력 처리:
          1. select.select(timeout=0.05) — 50ms 폴
          2. 입력 있으면 버퍼 드레인: while 즉시폴 → 모든 가용 바이트 읽기
          3. 이번 프레임 키 집합(current_frame) 구성
          4. timeout이면 키 집합 비움 (릴리즈)
        """
        ESCAPE_MAP = {
            '[A': 'W',   # ↑ = 전진
            '[B': 'S',   # ↓ = 후진
            '[D': 'A',   # ← = 좌회전
            '[C': 'D',   # → = 우회전
        }

        fd = sys.stdin.fileno()
        self._orig_term = termios.tcgetattr(fd)
        atexit.register(self._restore_terminal)
        tty.setcbreak(fd)

        while rclpy.ok():
            try:
                # ── 50ms 대기 ─────────────────────────────────────
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)

                if not ready:
                    with self._key_lock:
                        self._keys.clear()
                    continue

                # ── 버퍼 드레인: 이번 프레임의 모든 바이트 읽기 ──
                current_frame: set = set()
                esc_pending = False

                while True:
                    avail, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if not avail:
                        break

                    ch = sys.stdin.read(1)

                    if ch == '\x03':   # Ctrl+C
                        os.kill(os.getpid(), signal.SIGINT)
                        return

                    elif ch == '\x1b':
                        esc_pending = True

                    elif esc_pending:
                        if ch == '[':
                            avail2, _, _ = select.select([sys.stdin], [], [], 0.02)
                            if avail2:
                                third = sys.stdin.read(1)
                                mapped = ESCAPE_MAP.get('[' + third)
                                if mapped:
                                    current_frame.add(mapped)
                        esc_pending = False

                    elif ch in ('m', 'M'):
                        self._mode = 'AUTO' if self._mode == 'MANUAL' else 'MANUAL'
                        self.get_logger().info(f'[MODE] → {self._mode}')
                        self._publish_mode()
                        current_frame.clear()

                    elif ch in ('b', 'B'):
                        self._speed_mode = (
                            'boost' if self._speed_mode == 'normal' else 'normal'
                        )
                        val = self._spd_boost if self._speed_mode == 'boost' else self._spd_normal
                        self.get_logger().info(
                            f'[SPEED] → {self._speed_mode.upper()} ({int(val * 9999)} / 9999)'
                        )

                    elif ch in ('w', 'W'):
                        current_frame.add('W')
                    elif ch in ('s', 'S'):
                        current_frame.add('S')
                    elif ch in ('a', 'A'):
                        current_frame.add('A')
                    elif ch in ('d', 'D'):
                        current_frame.add('D')

                with self._key_lock:
                    self._keys = current_frame

            except Exception as exc:
                self.get_logger().error(f'[KB] 오류: {exc}')
                break

    # ── 20Hz 타이머: MANUAL 모드 명령 게시 ───────────────────────
    def _publish_cmd(self) -> None:
        if self._mode != 'MANUAL':
            return

        with self._key_lock:
            keys = set(self._keys)

        spd = self._spd_boost if self._speed_mode == 'boost' else self._spd_normal

        linear  = 0.0
        angular = 0.0

        if 'W' in keys:
            linear += spd
        if 'S' in keys:
            linear -= spd
        if 'A' in keys:
            angular += spd    # 좌회전 (+angular)
        if 'D' in keys:
            angular -= spd    # 우회전 (-angular)

        linear  = max(-1.0, min(1.0, linear))
        angular = max(-1.0, min(1.0, angular))

        msg           = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        self._restore_terminal()
        self._cmd_pub.publish(Twist())   # 긴급 정지
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
