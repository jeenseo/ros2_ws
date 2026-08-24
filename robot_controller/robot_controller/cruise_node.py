#!/usr/bin/env python3
"""
cruise_node.py — 캘리브레이션 전용 정속 주행 + 자동 측정
================================================================================

[왜 키보드로 캘리브레이션을 하면 안 되는가]
  터미널 auto-repeat 는 "첫 문자 → 약 0.5초 침묵 → 30Hz 연타" 구조입니다.
  즉 키를 계속 누르고 있어도 명령이 균일하게 나가지 않습니다.
  거기에 사람의 손 떨림, 키를 뗀 시점의 불확실성까지 겹치면
  가감속 프로파일이 매 시도마다 달라집니다.

  스케일 캘리브레이션은 "같은 조건을 반복 재현"하는 것이 전부인데,
  키보드는 그 전제를 깨뜨립니다. 사람을 루프에서 빼는 것이 정답입니다.

[이 노드가 하는 일]
  1. 카운트다운 → 램프업 → 정속 유지 → 램프다운 → 0
     (스티션 점프를 피하고 매번 동일한 가감속 프로파일을 재현)
  2. 시작/종료 시점의 /odom_motor, IMU yaw 를 자동 캡처
  3. 종료 후 요약 출력 — 줄자로 잰 실측값과 바로 비교 가능

[사용 예]
  # 직진 캘리브레이션 (PWM 4000 상당 = 0.377 m/s, 12초)
  ros2 run robot_controller cruise_node --ros-args \
      -p linear_x:=0.377 -p duration:=12.0

  # ★ 자이로 스케일 검증: 제자리 360도 회전
  ros2 run robot_controller cruise_node --ros-args \
      -p angular_z:=0.5 -p duration:=12.566
      # 0.5 rad/s x 12.566 s = 2pi rad = 정확히 1바퀴

  # 후진 (전/후진 쏠림 비대칭 확인용)
  ros2 run robot_controller cruise_node --ros-args \
      -p linear_x:=-0.377 -p duration:=12.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# ★ [수정] 구독 QoS 를 BEST_EFFORT 로
#
#   [무엇이 문제였나]
#     imu_gyro_bias_node 가 /imu/data_unbiased 를 BEST_EFFORT 로 발행하는데
#     이 노드는 기본값(RELIABLE)으로 구독했습니다.
#     DDS 호환 규칙: 발행자가 구독자보다 '약하면' 연결되지 않습니다.
#         RELIABLE  발행 + BEST_EFFORT 구독  -> 호환 O
#         BEST_EFFORT 발행 + RELIABLE 구독   -> 호환 X  ← 이 경우
#     그래서 "incompatible QoS ... Last incompatible policy: RELIABILITY" 경고가 뜨고
#     메시지가 한 건도 오지 않았습니다.
#
#   [해결 원칙]
#     구독자는 BEST_EFFORT 로 두는 것이 항상 안전합니다.
#     발행자가 RELIABLE 이든 BEST_EFFORT 든 양쪽 다 받을 수 있기 때문입니다.
_SUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a: float) -> float:
    """한 스텝 변화량을 (-pi, pi] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class CruiseNode(Node):

    def __init__(self):
        super().__init__('cruise_node')

        self.declare_parameter('cmd_topic',  '/cmd_vel_keyboard')
        self.declare_parameter('odom_topic', '/odom_motor')
        self.declare_parameter('imu_topic',  '/imu/data_unbiased')

        self.declare_parameter('linear_x',  0.0)     # [m/s]
        self.declare_parameter('linear_y',  0.0)
        self.declare_parameter('angular_z', 0.0)     # [rad/s]
        self.declare_parameter('duration',  10.0)    # [s] 정속 구간 길이
        self.declare_parameter('ramp',      1.0)     # [s] 램프업/다운 각각
        self.declare_parameter('countdown', 3.0)     # [s] 출발 전 카운트다운
        self.declare_parameter('rate_hz',   50.0)

        gp = self.get_parameter
        self._vx = float(gp('linear_x').value)
        self._vy = float(gp('linear_y').value)
        self._wz = float(gp('angular_z').value)
        self._dur = float(gp('duration').value)
        self._ramp = float(gp('ramp').value)
        self._cd = float(gp('countdown').value)
        rate = float(gp('rate_hz').value)

        self._pub = self.create_publisher(Twist, gp('cmd_topic').value, 10)
        self.create_subscription(Odometry, gp('odom_topic').value,
                                 self._odom_cb, _SUB_QOS)
        self.create_subscription(Imu, gp('imu_topic').value,
                                 self._imu_cb, _SUB_QOS)

        # ── 자동 측정 상태 ────────────────────────────────────────
        # ★ [수정] 총 회전량은 '언랩(unwrap)' 해서 누적해야 합니다.
        #
        #   [무엇이 틀렸나]
        #     기존 리포트는 끝점 yaw 와 시작점 yaw 를 빼고 (-180,180] 로 정규화했습니다.
        #     그런데 총 회전이 180°를 넘으면 그 정보가 통째로 사라집니다.
        #         실제 261.3° 회전  ->  261.3 - 360 = -98.7°  로 보고
        #     360° 시험처럼 한 바퀴 이상 도는 측정에서는 치명적입니다.
        #
        #   [해결]
        #     매 콜백마다 '직전 값과의 차이'만 정규화해서 누적합니다.
        #     한 스텝 변화량은 항상 작으므로(50Hz 에서 최대 몇 도) 랩핑이 안 생깁니다.
        self._odom = None            # (x, y, yaw_raw)
        self._odom_yaw_acc = 0.0     # 언랩 누적 yaw [rad]
        self._odom_yaw_prev = None
        self._imu_yaw_acc = 0.0      # 언랩 누적 IMU yaw [rad]
        self._imu_yaw_prev = None
        self._imu_seen = False
        self._start = None           # 시작 스냅샷
        self._t0 = time.monotonic()
        self._phase = 'COUNTDOWN'
        self._last_log = 0.0
        self._peak_wz = 0.0

        self._timer = self.create_timer(1.0 / rate, self._tick)

        total = self._cd + self._ramp * 2 + self._dur
        dist = self._vx * (self._dur + self._ramp)     # 램프는 평균 절반
        self.get_logger().info(
            f'\n{"="*60}\n'
            f' CRUISE  vx={self._vx:+.3f} vy={self._vy:+.3f} wz={self._wz:+.3f}\n'
            f'   램프 {self._ramp:.1f}s → 정속 {self._dur:.1f}s → 램프 {self._ramp:.1f}s\n'
            f'   총 {total:.1f}s,  예상 이동거리 ≈ {dist:.2f} m\n'
            f'   예상 회전량 ≈ {math.degrees(self._wz*(self._dur+self._ramp)):.1f}°\n'
            f' ★ 출발 전 바닥에 시작 위치를 표시하십시오 (차체 중심 + 정면 방향)\n'
            f'{"="*60}')

    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry) -> None:
        y = yaw_of(msg.pose.pose.orientation)
        if self._odom_yaw_prev is not None:
            self._odom_yaw_acc += wrap_pi(y - self._odom_yaw_prev)
        else:
            self._odom_yaw_acc = y
        self._odom_yaw_prev = y
        self._odom = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y, y)

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_seen = True
        y = yaw_of(msg.orientation)
        if self._imu_yaw_prev is not None:
            self._imu_yaw_acc += wrap_pi(y - self._imu_yaw_prev)
        else:
            self._imu_yaw_acc = y
        self._imu_yaw_prev = y
        self._peak_wz = max(self._peak_wz, abs(msg.angular_velocity.z))

    # ------------------------------------------------------------------ #
    def _profile(self, t: float):
        """카운트다운/램프/정속/램프다운 스케일 계수 [0..1] 과 상태 이름."""
        if t < self._cd:
            return 0.0, 'COUNTDOWN'
        t -= self._cd
        if t < self._ramp:
            return t / self._ramp, 'RAMP_UP'
        t -= self._ramp
        if t < self._dur:
            return 1.0, 'CRUISE'
        t -= self._dur
        if t < self._ramp:
            return 1.0 - t / self._ramp, 'RAMP_DOWN'
        return 0.0, 'DONE'

    def _tick(self) -> None:
        t = time.monotonic() - self._t0
        k, phase = self._profile(t)

        if phase != self._phase:
            if phase == 'RAMP_UP':
                # 언랩 누적값도 함께 스냅샷 (총 회전이 180°를 넘어도 정확)
                self._start = (self._odom, self._odom_yaw_acc,
                               self._imu_yaw_acc if self._imu_seen else None)
                self.get_logger().info(
                    f'▶ 출발!  시작 odom = {self._fmt(self._odom)}'
                    f'{"" if self._imu_seen else "   ⚠ IMU 미수신"}')
            self._phase = phase

        msg = Twist()
        msg.linear.x  = self._vx * k
        msg.linear.y  = self._vy * k
        msg.angular.z = self._wz * k
        self._pub.publish(msg)

        if t - self._last_log >= 1.0:
            self._last_log = t
            if phase == 'COUNTDOWN':
                self.get_logger().info(f'  {self._cd - t:.0f} …')
            else:
                self.get_logger().info(
                    f'  [{phase}] t={t:5.1f}s  cmd={self._vx*k:+.3f} m/s  '
                    f'odom={self._fmt(self._odom)}')

        if phase == 'DONE':
            for _ in range(10):
                self._pub.publish(Twist())
            self._report()
            self._timer.cancel()
            rclpy.shutdown()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _fmt(o):
        if o is None:
            return '(수신 없음)'
        return f'({o[0]:+.3f}, {o[1]:+.3f}, {math.degrees(o[2]):+.1f}°)'

    def _report(self) -> None:
        if self._start:
            s_odom, s_yaw_acc, s_imu_acc = self._start
        else:
            s_odom, s_yaw_acc, s_imu_acc = None, None, None
        e_odom = self._odom

        cmd_rot = math.degrees(self._wz * (self._dur + self._ramp))
        cmd_dist = self._vx * (self._dur + self._ramp)

        print('\n' + '=' * 68)
        print(' 주행 완료 — 아래 값을 그대로 기록하십시오')
        print('=' * 68)
        print(f'  명령      : vx={self._vx:+.3f} m/s  wz={self._wz:+.3f} rad/s')
        print(f'  정속 시간 : {self._dur:.1f} s  (램프 {self._ramp:.1f}s x2 별도)')
        print(f'  명령 기준 : 이동 {cmd_dist:+.3f} m,  회전 {cmd_rot:+.1f}°')

        if s_odom and e_odom:
            dx, dy = e_odom[0] - s_odom[0], e_odom[1] - s_odom[1]
            # ★ 언랩 누적 차이 — 180°를 넘어도 정확 (기존 버그 수정)
            dyaw = self._odom_yaw_acc - s_yaw_acc
            chord = math.hypot(dx, dy)
            if abs(dyaw) > 1e-3:
                # 호/현 변환은 반바퀴 이하일 때만 의미가 있습니다
                if abs(dyaw) < math.pi:
                    arc = chord * (abs(dyaw) / 2) / math.sin(abs(dyaw) / 2)
                else:
                    arc = None
            else:
                arc = chord
            print(f'\n  [odom_motor 변화량]')
            print(f'    Δx = {dx:+.4f} m,  Δy = {dy:+.4f} m')
            print(f'    ★ Δyaw(누적) = {math.degrees(dyaw):+.2f}°'
                  f'   ← 180° 넘어도 정확하게 누적됩니다')
            print(f'    현(chord) = {chord:.4f} m')
            if arc is not None:
                print(f'    ★ 호(arc)  = {arc:.4f} m   ← 엔코더가 잰 실제 이동거리')
            else:
                print(f'    호(arc)  = (회전이 180°를 넘어 등곡률 변환 불가 — '
                      f'제자리 회전 시험에서는 정상)')
            if abs(cmd_rot) > 1.0:
                print(f'    ▶ 명령 대비 실제 회전 = {math.degrees(dyaw)/cmd_rot*100:.1f} %')
            if abs(cmd_dist) > 0.01 and arc:
                print(f'    ▶ 명령 대비 실제 이동 = {arc/abs(cmd_dist)*100:.1f} %')
        else:
            print('\n  [odom_motor] 수신되지 않았습니다. motor_node 가 떠 있는지 확인')

        if s_imu_acc is not None and self._imu_seen:
            d = self._imu_yaw_acc - s_imu_acc
            print(f'\n  [IMU yaw 변화량(누적)] {math.degrees(d):+.2f}°   '
                  f'(피크 |wz| = {self._peak_wz:.3f} rad/s)')
        else:
            print('\n  [IMU] 수신되지 않았습니다.')
            print('       imu_gyro_bias_node 가 떠 있는지, 토픽 이름이 맞는지 확인하십시오.')
            print('       ros2 topic info /imu/data_unbiased --verbose')

        print(f'\n  ── 이제 줄자로 재서 아래를 채우십시오 ──')
        print(f'    실측 Δx (차체 중심 기준) = ______ m')
        print(f'    실측 Δy (차체 중심 기준) = ______ m   (+ = 좌측)')
        print(f'    실측 회전각 (바닥 각도기) = ______ °')
        print('=' * 68 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = CruiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._pub.publish(Twist())
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()