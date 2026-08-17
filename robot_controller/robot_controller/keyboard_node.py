#!/usr/bin/env python3
"""
keyboard_node.py  —  v2 (안전 패치판)
================================================================================
ROS 2 Node: 터미널 키보드 제어 — 메카넘 (Vy 포함)

═══════════════════════════════════════════════════════════════════════════════
★ v2 가 막는 것 — 워치독을 '우회하는' 폭주 경로
═══════════════════════════════════════════════════════════════════════════════

  기존 코드:
      except Exception as exc:
          self.get_logger().error(f'[KB] 오류: {exc}')
          break                       # ← 스레드만 죽고 노드는 살아있음

  스레드가 죽으면 self._keys 에 마지막 키 조합이 그대로 남고,
  20 Hz 타이머가 그 명령을 영원히 재발행합니다.

  ⚠ 여기가 핵심입니다.
    STM32 의 명령 워치독과 motor_node 의 하트비트는 "cmd_vel 이 *도착하는가*" 만 봅니다.
    이 경우 cmd_vel 은 20 Hz 로 멀쩡히 도착하므로 **모든 안전 계층이 정상으로 판단합니다.**
    즉 '의도(intent)의 부재' 는 '도착(arrival)의 부재' 와 다르고,
    기존 워치독 어느 것도 전자를 감지하지 못합니다.
    => 그 감지는 명령의 '출처'인 이 노드에서만 할 수 있습니다.

  v2 대응 3가지
    1. 스레드 예외 → _keys 비우기 + 즉시 0 발행 + _input_alive=False 로 영구 잠금
    2. ★ 스레드 하트비트 — 예외 없이 '멈추기만' 해도(블로킹/데드락) 감지
       스레드가 매 루프 타임스탬프를 갱신하고, 타이머가 그 신선도를 검사합니다.
       예외 처리만으로는 '조용히 멈춘' 스레드를 잡을 수 없습니다.
    3. 종료 시 0 을 여러 번 발행 + ExternalShutdownException 포착

  그 밖에
    - 스페이스바 → /e_stop 발행 (기존에 E-Stop 을 누를 방법이 아예 없었음)
    - /mode 를 TRANSIENT_LOCAL + 1 Hz 재발행 (늦게 뜬 노드도 모드를 알 수 있게)
    - stdin 이 TTY 가 아니면(ros2 launch 로 띄운 경우) 조용히 죽지 않고 명확히 실패
"""

import atexit
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:                                   # 구버전 호환
    class ExternalShutdownException(Exception):
        pass


SPEED_NORMAL    = (2000 / 9999.0) * 0.95   # 약 0.19 m/s
SPEED_BOOST     = (4000 / 9999.0) * 0.95   # 약 0.38 m/s
SPEED_OVERDRIVE = (8000 / 9999.0) * 0.95   # 약 0.76 m/s
TURN_MULTIPLIER = 1.978                    # MAX_WZ / MAX_VX = 1.88 / 0.95

# /mode 는 '늦게 뜬 구독자'도 반드시 받아야 하는 상태성 토픽입니다.
# VOLATILE + 토글 시에만 발행이면, 나중에 뜬 motor_node 는 모드를 영영 모릅니다.
_MODE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self.declare_parameter('normal_speed',    SPEED_NORMAL)
        self.declare_parameter('boost_speed',     SPEED_BOOST)
        self.declare_parameter('overdrive_speed', SPEED_OVERDRIVE)
        self.declare_parameter('publish_hz',      20.0)
        # 스레드가 이 시간 넘게 살아있음을 알리지 않으면 '사망'으로 간주
        self.declare_parameter('input_watchdog',  0.5)
        # 키를 뗀 것으로 판정하기까지의 유예 (터미널 auto-repeat 초기 지연 흡수)
        self.declare_parameter('key_release_grace', 0.15)

        gp = self.get_parameter
        self._spd_normal    = gp('normal_speed').value
        self._spd_boost     = gp('boost_speed').value
        self._spd_overdrive = gp('overdrive_speed').value
        publish_hz          = float(gp('publish_hz').value)
        self._input_wd      = float(gp('input_watchdog').value)
        self._key_grace     = float(gp('key_release_grace').value)

        # ── 게시자 ────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,  '/cmd_vel_keyboard', 10)
        self._mode_pub  = self.create_publisher(String, '/mode', _MODE_QOS)
        self._estop_pub = self.create_publisher(Bool,   '/e_stop', 10)

        # ── 공유 상태 ─────────────────────────────────────────────
        self._mode       = 'MANUAL'
        self._speed_mode = 'normal'
        self._keys       = set()
        self._key_stamp  = 0.0            # 마지막으로 키가 관측된 시각
        self._key_lock   = threading.Lock()
        self._orig_term  = None
        self._estopped   = False

        # ★ 안전 상태
        self._input_alive = True          # False 가 되면 영구히 0 만 발행
        self._kb_heartbeat = time.monotonic()

        # ── 터미널 준비: 실패하면 '조용히' 죽지 않도록 여기서 확인 ──
        try:
            fd = sys.stdin.fileno()
            self._orig_term = termios.tcgetattr(fd)
            atexit.register(self._restore_terminal)
        except Exception as exc:
            self.get_logger().fatal(
                f'stdin 이 TTY 가 아닙니다({exc}). keyboard_node 는 반드시 터미널에서 '
                '직접 실행해야 합니다. ros2 launch 로는 키 입력을 받을 수 없습니다.')
            raise

        kb_t = threading.Thread(target=self._keyboard_loop, daemon=True)
        kb_t.start()

        self._timer = self.create_timer(1.0 / publish_hz, self._publish_cmd)
        self._mode_timer = self.create_timer(1.0, self._publish_mode)  # 1 Hz 재발행

        self.get_logger().info(
            'KeyboardNode v2 준비 완료\n'
            '  W/↑=전진  S/↓=후진  A/←=좌회전  D/→=우회전\n'
            '  Q=좌측 게걸음  E=우측 게걸음  T/Y/G/H=대각\n'
            '  ★ SPACE = 비상정지 토글(/e_stop)\n'
            '  m=MANUAL↔AUTO  b=Boost(4000)  o=Overdrive(8000)  Ctrl+C=종료')
        self._publish_mode()

    # ══════════════════════════════════════════════════════════════
    # 안전
    # ══════════════════════════════════════════════════════════════

    def _publish_zero(self, times: int = 1) -> None:
        for _ in range(times):
            try:
                self._cmd_pub.publish(Twist())
            except Exception:
                return

    def _kill_input(self, reason: str) -> None:
        """
        입력 경로를 영구 차단한다. 되살리려면 노드를 재시작해야 한다.

        의도적으로 '복구 불가'로 만든 이유: 입력 스레드가 한 번 이상 동작을
        의심받았다면, 그 상태에서 자동 복구시켜 다시 주행하게 두는 것이
        사람이 재시작하는 것보다 훨씬 위험합니다.
        """
        if not self._input_alive:
            return
        self._input_alive = False
        with self._key_lock:
            self._keys.clear()
        self._publish_zero(3)
        self.get_logger().fatal(
            f'[KB] 입력 경로 차단: {reason}\n'
            f'     이후 0 속도만 발행합니다. 노드를 재시작하십시오.')

    # ══════════════════════════════════════════════════════════════
    # 터미널 / 모드
    # ══════════════════════════════════════════════════════════════

    def _restore_terminal(self) -> None:
        if self._orig_term is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(),
                                  termios.TCSADRAIN, self._orig_term)
            except Exception:
                pass
            self._orig_term = None

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _publish_estop(self) -> None:
        msg = Bool()
        msg.data = self._estopped
        self._estop_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════
    # 키보드 스레드
    # ══════════════════════════════════════════════════════════════

    def _keyboard_loop(self) -> None:
        ESCAPE_MAP = {'[A': 'W', '[B': 'S', '[D': 'A', '[C': 'D'}

        try:
            tty.setcbreak(sys.stdin.fileno())
        except Exception as exc:
            self._kill_input(f'cbreak 설정 실패: {exc}')
            return

        while rclpy.ok() and self._input_alive:
            # ★ 하트비트: 예외 없이 '멈추기만' 해도 타이머가 알아채도록
            self._kb_heartbeat = time.monotonic()
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue          # ★ 여기서 _keys.clear() 하지 않습니다.
                                      #   유예 시간(_key_grace)으로 판정하므로
                                      #   auto-repeat 지연에도 덜컹거리지 않습니다.

                current_frame: set = set()
                esc_pending = False

                while True:
                    avail, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if not avail:
                        break
                    ch = sys.stdin.read(1)

                    if ch == '\x03':                       # Ctrl+C
                        os.kill(os.getpid(), signal.SIGINT)
                        return

                    elif ch == ' ':                        # ★ 스페이스 = E-Stop 토글
                        self._estopped = not self._estopped
                        self._publish_estop()
                        self._publish_zero(2)
                        self.get_logger().warn(
                            f'[E-STOP] {"발동" if self._estopped else "해제"}')
                        current_frame.clear()

                    elif ch == '\x1b':
                        esc_pending = True

                    elif esc_pending:
                        if ch == '[':
                            avail2, _, _ = select.select([sys.stdin], [], [], 0.02)
                            if avail2:
                                mapped = ESCAPE_MAP.get('[' + sys.stdin.read(1))
                                if mapped:
                                    current_frame.add(mapped)
                        esc_pending = False

                    elif ch in ('m', 'M'):
                        self._mode = 'AUTO' if self._mode == 'MANUAL' else 'MANUAL'
                        self.get_logger().info(f'[MODE] → {self._mode}')
                        self._publish_mode()
                        self._publish_zero(2)      # 모드 전환 시 반드시 정지
                        current_frame.clear()

                    elif ch in ('b', 'B'):
                        self._speed_mode = 'boost' if self._speed_mode != 'boost' else 'normal'
                        self.get_logger().info(f'[SPEED] → {self._speed_mode.upper()}')
                    elif ch in ('o', 'O'):
                        self._speed_mode = 'overdrive' if self._speed_mode != 'overdrive' else 'normal'
                        self.get_logger().info(f'[SPEED] → {self._speed_mode.upper()}')

                    elif ch in ('w', 'W'): current_frame.add('W')
                    elif ch in ('s', 'S'): current_frame.add('S')
                    elif ch in ('a', 'A'): current_frame.add('A')
                    elif ch in ('d', 'D'): current_frame.add('D')
                    elif ch in ('q', 'Q'): current_frame.add('Q')
                    elif ch in ('e', 'E'): current_frame.add('E')
                    elif ch in ('t', 'T'): current_frame.update(('W', 'Q'))
                    elif ch in ('y', 'Y'): current_frame.update(('W', 'E'))
                    elif ch in ('g', 'G'): current_frame.update(('S', 'Q'))
                    elif ch in ('h', 'H'): current_frame.update(('S', 'E'))

                if current_frame:
                    with self._key_lock:
                        self._keys = current_frame
                        self._key_stamp = time.monotonic()

            except Exception as exc:
                # ★ 기존: break (키 잔존 → 20Hz 무한 재발행 → 폭주)
                #   변경: 명령을 비우고 0 을 쏜 뒤 영구 차단
                self._kill_input(f'입력 스레드 예외: {exc!r}')
                return

        self._kill_input('입력 스레드 정상 종료')

    # ══════════════════════════════════════════════════════════════
    # 20 Hz 발행
    # ══════════════════════════════════════════════════════════════

    def _publish_cmd(self) -> None:
        now = time.monotonic()

        # ── 안전 게이트 1: 입력 경로 차단 상태 ────────────────────
        if not self._input_alive:
            self._publish_zero()
            return

        # ── 안전 게이트 2: ★ 스레드 하트비트 (조용히 멈춘 경우 감지) ──
        if (now - self._kb_heartbeat) > self._input_wd:
            self._kill_input(
                f'입력 스레드 무응답 {now - self._kb_heartbeat:.2f}s '
                f'(한계 {self._input_wd:.2f}s)')
            self._publish_zero()
            return

        # ── 안전 게이트 3: E-Stop ────────────────────────────────
        if self._estopped:
            self._publish_zero()
            return

        if self._mode != 'MANUAL':
            self._publish_zero()          # AUTO 에서도 0 을 계속 흘려 하트비트 유지
            return

        # ── 키 유예 판정 ─────────────────────────────────────────
        with self._key_lock:
            keys = set(self._keys)
            stamp = self._key_stamp
        if (now - stamp) > self._key_grace:
            keys = set()                  # 유예 초과 = 키를 뗀 것

        spd = (self._spd_overdrive if self._speed_mode == 'overdrive'
               else self._spd_boost if self._speed_mode == 'boost'
               else self._spd_normal)

        linear_x = (spd if 'W' in keys else 0.0) - (spd if 'S' in keys else 0.0)
        linear_y = (spd if 'Q' in keys else 0.0) - (spd if 'E' in keys else 0.0)
        angular_z = ((spd * TURN_MULTIPLIER if 'A' in keys else 0.0)
                     - (spd * TURN_MULTIPLIER if 'D' in keys else 0.0))

        msg = Twist()
        msg.linear.x  = max(-1.0, min(1.0, linear_x))
        msg.linear.y  = max(-1.0, min(1.0, linear_y))
        msg.angular.z = max(-2.0, min(2.0, angular_z))
        self._cmd_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════

    def destroy_node(self):
        self._input_alive = False
        try:
            self._timer.cancel()
            self._mode_timer.cancel()
        except Exception:
            pass
        self._restore_terminal()
        # 단발 발행은 DDS 큐에서 유실될 수 있으므로 여러 번 + 짧은 대기
        for _ in range(5):
            self._publish_zero()
            time.sleep(0.02)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()