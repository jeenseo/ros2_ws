#!/usr/bin/env python3
"""
imu_gyro_bias_node.py
=====================
/imu/data  ->  /imu/data_unbiased  자이로 Z 바이어스 추정 및 제거

[왜 이 노드가 Stage 1b에서 필수가 되었는가]
전륜 2채널 엔코더로는 메카넘 요레이트를 복원할 수 없다는 결론에 따라,
이제 요(Yaw)의 **유일한 고주파 소스가 IMU 자이로 하나**가 되었습니다.
소스가 하나뿐이면 그 소스의 오차를 교차 검증할 방법이 없습니다.

그리고 MPU6050의 지배적 오차는 백색 노이즈가 아니라 **바이어스**입니다.
  - 데이터시트 노이즈 밀도 기준 sigma ≈ 0.003 rad/s (기존 ekf.yaml의 1e-5 가정)
  - 그러나 실제 바이어스 안정도는 온도/시간에 따라 0.01~0.05 rad/s 수준으로 변동
  - 0.01 rad/s 바이어스를 60초 적분하면 0.6 rad = 34도의 가짜 방위각

여기서도 Stage 1의 원칙이 그대로 적용됩니다:
  **편향은 공분산(R)으로 못 없앤다. 추정해서 빼야 한다.**

[동작]
1. IMU 데이터만으로 정지 상태를 자체 판정 (외부 토픽 의존 없음)
     - 자이로 3축 크기가 임계값 이하
     - 가속도 크기가 |g| 근방이고, 슬라이딩 윈도우 분산이 작음
2. 정지 확정 시 자이로 3축 바이어스를 느린 EMA로 갱신
3. 바이어스를 뺀 값과 **현실적인 공분산**을 실어 재발행

[실행]
  ros2 run <your_pkg> imu_gyro_bias_node.py --ros-args \
      -p output_gyro_z_variance:=4.0e-4
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_G = 9.80665


class ImuGyroBiasNode(Node):

    def __init__(self):
        super().__init__('imu_gyro_bias_node')

        self.declare_parameter('input_topic',  '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_unbiased')

        # ── 정지 판정 ────────────────────────────────────────────────
        self.declare_parameter('still_gyro_thresh', 0.03)   # [rad/s] 자이로 크기 임계
        self.declare_parameter('still_accel_band',  0.35)   # [m/s^2] ||a|| 와 g 의 허용 편차
        self.declare_parameter('still_accel_var',   0.05)   # [m/s^2]^2 윈도우 분산 임계
        self.declare_parameter('window_sec',        0.5)    # [s] 판정 윈도우
        self.declare_parameter('still_hold_sec',    1.0)    # [s] 이만큼 지속돼야 확정

        # ── 바이어스 추정 ────────────────────────────────────────────
        #   tau 가 길수록 안정적이지만 온도 드리프트 추종이 느려집니다.
        #   정지 상태에서만 갱신되므로, 실제 벽시계 시간이 아니라 '정지 누적 시간' 기준입니다.
        self.declare_parameter('bias_tau',       20.0)      # [s]
        self.declare_parameter('bias_limit',     0.30)      # [rad/s] 안전 상한
        self.declare_parameter('warmup_sec',     3.0)       # [s] 부팅 후 이만큼은 빠르게 수렴

        # ── ★ 출력 공분산 (EKF가 읽는 R) ─────────────────────────────
        #   기존 1e-5 (sigma 0.0032 rad/s) 는 데이터시트 백색노이즈만 반영한 낙관값입니다.
        #   바이어스 추정 후에도 잔류 바이어스 불안정도가 남으므로 4e-4 (sigma 0.02 rad/s)를
        #   권장합니다. 이 값이 rf2o Vyaw(var 3.6e-3) 대비 9:1 이 되어,
        #   단기는 IMU가 지배하되 장기 방위 드리프트는 라이다가 끌어당길 수 있게 됩니다.
        #   (1e-5 로 두면 비율이 360:1 이라 라이다가 자이로 드리프트를 전혀 못 잡습니다)
        self.declare_parameter('output_gyro_z_variance', 4.0e-4)
        self.declare_parameter('output_gyro_xy_variance', 4.0e-4)

        gp = self.get_parameter
        self._th_gyro  = float(gp('still_gyro_thresh').value)
        self._th_aband = float(gp('still_accel_band').value)
        self._th_avar  = float(gp('still_accel_var').value)
        self._win_sec  = float(gp('window_sec').value)
        self._hold_sec = float(gp('still_hold_sec').value)
        self._tau      = float(gp('bias_tau').value)
        self._limit    = float(gp('bias_limit').value)
        self._warmup   = float(gp('warmup_sec').value)
        self._var_z    = float(gp('output_gyro_z_variance').value)
        self._var_xy   = float(gp('output_gyro_xy_variance').value)

        self._bias = [0.0, 0.0, 0.0]
        self._win: deque = deque()        # (t, ax, ay, az)
        self._still_since: float | None = None
        self._t0: float | None = None
        self._last_t: float | None = None
        self._still_accum = 0.0           # 정지 누적 시간 [s]
        self._converged = False

        self._sub = self.create_subscription(Imu, gp('input_topic').value,
                                             self._cb, _SENSOR_QOS)
        self._pub = self.create_publisher(Imu, gp('output_topic').value, _SENSOR_QOS)
        self._pub_diag = self.create_publisher(Float32MultiArray, '~/bias_diagnostics', 10)

        self.get_logger().info(
            f"자이로 바이어스 노드: {gp('input_topic').value} -> {gp('output_topic').value}\n"
            f'  출력 gyro_z 분산 = {self._var_z:.2e} (sigma {math.sqrt(self._var_z):.4f} rad/s)\n'
            f'  ※ 부팅 직후 {self._warmup:.0f}초는 로봇을 완전히 정지시켜 두십시오.')

    # ------------------------------------------------------------------ #
    def _cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        dt = 0.01 if self._last_t is None else max(min(t - self._last_t, 0.2), 1e-4)
        self._last_t = t

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        still = self._is_still(t, gx, gy, gz, ax, ay, az)

        if still:
            self._still_accum += dt
            # 부팅 warmup 구간에서는 시상수를 짧게 잡아 빠르게 수렴시킵니다.
            in_warmup = (t - self._t0) < self._warmup
            tau = 0.5 if in_warmup else self._tau
            a = 1.0 - math.exp(-dt / tau)
            for i, g in enumerate((gx, gy, gz)):
                b = self._bias[i] + a * (g - self._bias[i])
                self._bias[i] = max(-self._limit, min(self._limit, b))
            if not self._converged and self._still_accum > self._warmup:
                self._converged = True
                self.get_logger().info(
                    f'자이로 바이어스 초기 수렴: '
                    f'[{self._bias[0]:+.5f}, {self._bias[1]:+.5f}, {self._bias[2]:+.5f}] rad/s '
                    f'(z축 {math.degrees(self._bias[2]):+.3f} deg/s)')

        # ── 보정 후 재발행 ──────────────────────────────────────────
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        out.angular_velocity.x = gx - self._bias[0]
        out.angular_velocity.y = gy - self._bias[1]
        out.angular_velocity.z = gz - self._bias[2]

        # 3x3 row-major: [0]=xx, [4]=yy, [8]=zz
        cov = [0.0] * 9
        cov[0] = self._var_xy
        cov[4] = self._var_xy
        cov[8] = self._var_z
        out.angular_velocity_covariance = cov

        self._pub.publish(out)

        d = Float32MultiArray()
        # 0:bias_z[rad/s] 1:bias_z[deg/s] 2:still 3:still_accum[s] 4:gz_raw 5:gz_corrected
        d.data = [float(self._bias[2]), float(math.degrees(self._bias[2])),
                  1.0 if still else 0.0, float(self._still_accum),
                  float(gz), float(gz - self._bias[2])]
        self._pub_diag.publish(d)

    # ------------------------------------------------------------------ #
    def _is_still(self, t, gx, gy, gz, ax, ay, az) -> bool:
        """
        IMU 자체 정보만으로 정지 판정.
        외부 토픽(cmd_vel, 엔코더)에 의존하지 않는 이유: 명령이 0이어도 로봇이
        관성으로 밀리거나 사람이 밀 수 있고, 그때 바이어스를 갱신하면 오염됩니다.

        조건 3가지를 모두 만족해야 정지:
          1) 자이로 크기가 작다                      (회전 없음)
          2) 가속도 크기가 |g| 근방이다              (선가속 없음, 중력만 받는 중)
          3) 가속도 크기의 윈도우 분산이 작다        (진동/충격 없음)
        3번이 중요합니다. 일정 속도로 미끄러지는 중에도 1,2는 만족할 수 있지만
        실제 바닥 위 주행이면 미세 진동이 반드시 잡힙니다.
        """
        gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
        amag = math.sqrt(ax * ax + ay * ay + az * az)

        self._win.append((t, amag))
        while self._win and (t - self._win[0][0]) > self._win_sec:
            self._win.popleft()

        if len(self._win) < 5:
            return False
        vals = [v for _, v in self._win]
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)

        ok = (gmag < self._th_gyro
              and abs(amag - _G) < self._th_aband
              and var < self._th_avar)

        if not ok:
            self._still_since = None
            return False
        if self._still_since is None:
            self._still_since = t
            return False
        return (t - self._still_since) >= self._hold_sec


def main(args=None):
    rclpy.init(args=args)
    node = ImuGyroBiasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()