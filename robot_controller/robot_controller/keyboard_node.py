"""
keyboard_node.py
================
ROS 2 Node: 터미널 키보드 제어 — 메카넘/스키드-스티어 공용 (Vy=0 고정)
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


# ── 속도 레벨 (m/s 기준 계산: (목표PWM / 9999) * MAX_VX(0.95)) ──────
SPEED_NORMAL    = (2000 / 9999.0) * 0.95  # 약 0.19 m/s -> 2000 PWM
SPEED_BOOST     = (4000 / 9999.0) * 0.95  # 약 0.38 m/s -> 4000 PWM
SPEED_OVERDRIVE = (8000 / 9999.0) * 0.95  # 약 0.76 m/s -> 8000 PWM ('o' 키 전용)

# 메카넘 휠 회전 보정 비율 (MAX_WZ / MAX_VX) = 1.88 / 0.95
# IK 계산 시 반토막 나는 회전 휠 속도를 직진 휠 속도와 동일하게 맞춰줍니다.
TURN_MULTIPLIER = 1.978 


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('normal_speed', SPEED_NORMAL)
        self.declare_parameter('boost_speed',  SPEED_BOOST)
        self.declare_parameter('overdrive_speed', SPEED_OVERDRIVE)

        self._spd_normal    = self.get_parameter('normal_speed').value
        self._spd_boost     = self.get_parameter('boost_speed').value
        self._spd_overdrive = self.get_parameter('overdrive_speed').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist,  '/cmd_vel_keyboard', 10)
        self._mode_pub = self.create_publisher(String, '/mode',             10)

        # ── 공유 상태 ─────────────────────────────────────────────
        self._mode       = 'MANUAL'
        self._speed_mode = 'normal'
        self._keys       = set()
        self._key_lock   = threading.Lock()
        self._orig_term  = None

        # ── 키보드 백그라운드 스레드 ──────────────────────────────
        kb_t = threading.Thread(target=self._keyboard_loop, daemon=True)
        kb_t.start()

        # ── 20Hz 명령 게시 타이머 ─────────────────────────────────
        self._timer = self.create_timer(1.0 / 20.0, self._publish_cmd)

        self.get_logger().info(
            'KeyboardNode 준비 완료\n'
            '  W/↑=전진  S/↓=후진  A/←=좌회전  D/→=우회전\n'
            '  Q=좌측 게걸음  E=우측 게걸음\n'
            '  m=MANUAL↔AUTO  b=Boost(4000)  o=Overdrive(8000)  Ctrl+C=종료'
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
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)

                if not ready:
                    with self._key_lock:
                        self._keys.clear()
                    continue

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
                                third  = sys.stdin.read(1)
                                mapped = ESCAPE_MAP.get('[' + third)
                                if mapped:
                                    current_frame.add(mapped)
                        esc_pending = False

                    elif ch in ('m', 'M'):
                        self._mode = 'AUTO' if self._mode == 'MANUAL' else 'MANUAL'
                        self.get_logger().info(f'[MODE] → {self._mode}')
                        self._publish_mode()
                        current_frame.clear()

                    # ── 속도 모드 변경 ───────────────────────────────────
                    elif ch in ('b', 'B'):
                        self._speed_mode = 'boost' if self._speed_mode != 'boost' else 'normal'
                        val = 4000 if self._speed_mode == 'boost' else 2000
                        self.get_logger().info(f'[SPEED] → {self._speed_mode.upper()} ({val} PWM)')

                    elif ch in ('o', 'O'):
                        self._speed_mode = 'overdrive' if self._speed_mode != 'overdrive' else 'normal'
                        val = 8000 if self._speed_mode == 'overdrive' else 2000
                        self.get_logger().info(f'[SPEED] → {self._speed_mode.upper()} ({val} PWM)')

                    # ── 이동 키 ───────────────────────────────────
                    elif ch in ('w', 'W'):
                        current_frame.add('W')   # linear.x +
                    elif ch in ('s', 'S'):
                        current_frame.add('S')   # linear.x -
                    elif ch in ('a', 'A'):
                        current_frame.add('A')   # angular.z +
                    elif ch in ('d', 'D'):
                        current_frame.add('D')   # angular.z -
                    elif ch in ('q', 'Q'):
                        current_frame.add('Q')   # linear.y + (좌측 스트레이프)
                    elif ch in ('e', 'E'):
                        current_frame.add('E')   # linear.y - (우측 스트레이프)
                    # 🚀 [추가] 대각선 주행 키 (메카넘 전용: 전진/후진 + 좌/우 스트레이프)
                    elif ch in ('t', 'T'):
                        current_frame.add('W')
                        current_frame.add('Q')   # T = 전진 + 좌측 게걸음 (11시 방향)
                    elif ch in ('y', 'Y'):
                        current_frame.add('W')
                        current_frame.add('E')   # Y = 전진 + 우측 게걸음 (1시 방향)
                    elif ch in ('g', 'G'):
                        current_frame.add('S')
                        current_frame.add('Q')   # G = 후진 + 좌측 게걸음 (7시 방향)
                    elif ch in ('h', 'H'):
                        current_frame.add('S')
                        current_frame.add('E')   # H = 후진 + 우측 게걸음 (5시 방향)

                with self._key_lock:
                    self._keys = current_frame

            except Exception as exc:
                self.get_logger().error(f'[KB] 오류: {exc}')
                break

    # ── 20Hz 타이머: MANUAL 모드 Twist 게시 ──────────────────────
    def _publish_cmd(self) -> None:
        if self._mode != 'MANUAL':
            return

        with self._key_lock:
            keys = set(self._keys)

        # 현재 설정된 속도 모드에 맞춰 spd 값(m/s) 가져오기
        if self._speed_mode == 'overdrive':
            spd = self._spd_overdrive
        elif self._speed_mode == 'boost':
            spd = self._spd_boost
        else:
            spd = self._spd_normal

        # ── W/S → linear.x (전진/후진) ───────────────────────────
        linear_x = 0.0
        if 'W' in keys:
            linear_x += spd
        if 'S' in keys:
            linear_x -= spd

        # ── Q/E → linear.y (좌/우 스트레이프) ─────────────────────
        linear_y = 0.0
        if 'Q' in keys:
            linear_y += spd
        if 'E' in keys:
            linear_y -= spd

        # ── A/D → angular.z (좌/우 회전) ─────────────────────────
        angular_z = 0.0
        if 'A' in keys:
            angular_z += (spd * TURN_MULTIPLIER)   # CCW (+) 보정 적용
        if 'D' in keys:
            angular_z -= (spd * TURN_MULTIPLIER)   # CW  (-) 보정 적용

        # ── Twist 메시지 구성 ─────────────────────────────────────
        msg = Twist()
        # Overdrive 시 0.76이 들어가므로 1.0 캡에 걸리지 않습니다.
        # 회전 값은 1.5 rad/s 이상 올라갈 수 있으므로 캡을 2.0으로 상향 조정했습니다.
        msg.linear.x  = max(-1.0, min(1.0, linear_x))
        msg.linear.y  = max(-1.0, min(1.0, linear_y))
        msg.linear.z  = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = max(-2.0, min(2.0, angular_z))

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

if __name__ == '__main__':
    main()