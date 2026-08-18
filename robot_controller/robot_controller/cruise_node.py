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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
        self.create_subscription(Odometry, gp('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(Imu, gp('imu_topic').value, self._imu_cb, 10)

        # ── 자동 측정 상태 ────────────────────────────────────────
        self._odom = None            # (x, y, yaw)
        self._imu_yaw = None         # IMU 적분 yaw (있으면)
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
        self._odom = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw_of(msg.pose.pose.orientation))

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_yaw = yaw_of(msg.orientation)
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
                self._start = (self._odom, self._imu_yaw)
                self.get_logger().info(
                    f'▶ 출발!  시작 odom = {self._fmt(self._odom)}')
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
        s_odom, s_imu = self._start if self._start else (None, None)
        e_odom, e_imu = self._odom, self._imu_yaw

        print('\n' + '=' * 68)
        print(' 주행 완료 — 아래 값을 그대로 기록하십시오')
        print('=' * 68)
        print(f'  명령      : vx={self._vx:+.3f} m/s  wz={self._wz:+.3f} rad/s')
        print(f'  정속 시간 : {self._dur:.1f} s  (램프 {self._ramp:.1f}s x2 별도)')

        if s_odom and e_odom:
            dx, dy = e_odom[0] - s_odom[0], e_odom[1] - s_odom[1]
            dyaw = math.atan2(math.sin(e_odom[2] - s_odom[2]),
                              math.cos(e_odom[2] - s_odom[2]))
            chord = math.hypot(dx, dy)
            if abs(dyaw) > 1e-3:
                arc = chord * (abs(dyaw) / 2) / math.sin(abs(dyaw) / 2)
            else:
                arc = chord
            print(f'\n  [odom_motor 변화량]')
            print(f'    Δx = {dx:+.4f} m,  Δy = {dy:+.4f} m')
            print(f'    Δyaw = {math.degrees(dyaw):+.2f}°')
            print(f'    현(chord) = {chord:.4f} m')
            print(f'    ★ 호(arc)  = {arc:.4f} m   ← 엔코더가 잰 실제 이동거리')
        else:
            print('\n  [odom_motor] 수신되지 않았습니다. motor_node 가 떠 있는지 확인')

        if s_imu is not None and e_imu is not None:
            d = math.atan2(math.sin(e_imu - s_imu), math.cos(e_imu - s_imu))
            print(f'\n  [IMU yaw 변화량] {math.degrees(d):+.2f}°   '
                  f'(피크 |wz| = {self._peak_wz:.3f} rad/s)')

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