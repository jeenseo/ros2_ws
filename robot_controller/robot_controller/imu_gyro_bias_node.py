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

# ── 구독: BEST_EFFORT (발행자가 RELIABLE 이든 BEST_EFFORT 든 모두 수신 가능) ──
_SUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# ★ [수정] 발행: BEST_EFFORT -> RELIABLE
#
#   [무엇이 문제였나]
#     이 노드는 /imu/data 를 대체하는 '드롭인 교체품'이어야 합니다.
#     그런데 원본 mpu6050_node 는 /imu/data 를 RELIABLE(기본값)로 발행하는데,
#     이 노드가 /imu/data_unbiased 를 BEST_EFFORT 로 발행하고 있었습니다.
#
#     DDS 호환 규칙:  BEST_EFFORT 발행 + RELIABLE 구독 = 연결 안 됨
#     따라서 기본 QoS(RELIABLE)로 구독하는 모든 노드가 데이터를 못 받습니다.
#       - cruise_node          -> "incompatible QoS" 경고 (관측됨)
#       - ★ ekf_node (robot_localization) -> 경고도 없이 IMU 데이터 0건
#
#   [해결]
#     원본과 동일한 RELIABLE 로 발행합니다. 그러면 RELIABLE 구독자도,
#     BEST_EFFORT 구독자(motor_node 등)도 모두 정상 수신합니다.
_PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_G = 9.80665


class ImuGyroBiasNode(Node):

    def __init__(self):
        super().__init__('imu_gyro_bias_node')

        self.declare_parameter('input_topic',  '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_unbiased')

        # ══ 정지 판정 (v2 재설계) ════════════════════════════════════
        #
        # ★ v1 의 논리적 결함
        #   정지 판정에 자이로의 '절대 크기'를 썼는데, 그 크기에는 바로 우리가
        #   추정하려는 바이어스가 들어 있습니다.
        #       "바이어스 때문에 정지로 인식 안 됨 -> 그래서 바이어스를 못 뺌"
        #   순환 논리입니다. 실측에서도 |gyro| = 0.0413 rad/s 였는데
        #   그중 0.0394 가 gx,gy 의 바이어스였습니다(gz 는 0.0124뿐).
        #
        # ★ v2 의 해법 — 주 지표를 '분산(variance)' 으로 교체
        #   분산은 평균을 빼고 계산하므로 바이어스에 영향받지 않습니다.
        #       정지: 바이어스가 얼마든 값이 일정 -> 분산 ≈ 0
        #       이동: 값이 흔들림              -> 분산 큼
        #   절대 크기 검사는 '명백한 회전'을 거르는 느슨한 보조 역할만 합니다.
        #
        # ★ 가속도 임계도 완화
        #   실측 |a| = 10.34 (g 대비 +55 mg). MPU6050 zero-g offset 규격이
        #   ±50~80 mg 이므로 정상 개체입니다. v1 의 0.35 m/s²(=36 mg) 는
        #   무보정 MPU6050 에 애초에 성립할 수 없는 기준이었습니다.
        # ── 정지 판정 임계값 (v3: 크기 기반 -> 분산 기반으로 전환) ──────
        #   [기각 마진 계산]  정지 시 gvar ~ 1e-6..1e-5,  주행 시 gvar ~ 1e-4..1e-3.
        #   두 분포의 기하평균 부근인 5e-5 를 임계로 잡으면 양쪽에 각각 5~10배의
        #   마진이 생깁니다. 5e-5 는 sigma = sqrt(5e-5) = 7.1e-3 rad/s = 0.41 deg/s
        #   에 해당하며, 이는 "손대지 않은 로봇의 자이로 요동" 수준입니다.
        #   현장에서 그래도 실패하면 diagnostics data[10] (실측 gvar)을 보고
        #   그 값의 3배로 올리십시오. data[10] 이 곧 정답을 알려줍니다.
        self.declare_parameter('still_gyro_thresh', 0.15)   # [rad/s] 느슨한 보조 검사 (was 0.03)
        self.declare_parameter('still_gyro_var',    5.0e-5) # ★ 주 지표: 자이로 분산 (바이어스 불변)
        self.declare_parameter('still_accel_band',  1.5)    # [m/s^2] 바이어스 허용 (was 0.35)
        self.declare_parameter('still_accel_var',   0.05)   # [m/s^2]^2 진동 검출
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
        self._th_gvar  = float(gp('still_gyro_var').value)
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
        self._win: deque = deque()        # (t, amag, gx, gy, gz)
        self._still_since: float | None = None
        self._t0: float | None = None
        self._last_t: float | None = None
        self._still_accum = 0.0           # 정지 누적 시간 [s]
        self._converged = False

        self._sub = self.create_subscription(Imu, gp('input_topic').value,
                                             self._cb, _SUB_QOS)
        self._pub = self.create_publisher(Imu, gp('output_topic').value, _PUB_QOS)
        self._pub_diag = self.create_publisher(Float32MultiArray, '~/bias_diagnostics', 10)

        self.get_logger().info(
            f"자이로 바이어스 노드: {gp('input_topic').value} -> {gp('output_topic').value}\n"
            f'  출력 gyro_z 분산 = {self._var_z:.2e} (sigma {math.sqrt(self._var_z):.4f} rad/s)\n'
            f'  정지판정 임계 [v3 분산기반]: gvar<{self._th_gvar:.1e}  '
            f'|g|<{self._th_gyro}  ||a|-9.807|<{self._th_aband}  avar<{self._th_avar}\n'
            f'  ※ 부팅 직후 {self._warmup:.0f}초는 로봇을 완전히 정지시켜 두십시오.\n'
            f'  ※ 진단: ros2 topic echo /imu_gyro_bias_node/bias_diagnostics\n'
            f'     data[2]=1 이면 정지판정 성공, data[9]=실패코드, data[10]=실측 자이로분산')

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
        # 0:bias_z[rad/s]  1:bias_z[deg/s]  2:still  3:still_accum[s]
        # 4:gz_raw  5:gz_corrected
        # ★ 아래는 "왜 정지 판정이 안 되는가" 진단용 (v2 추가)
        # 6:|gyro|  7:|accel|  8:accel분산  9:실패원인코드  10:★자이로분산(v3)
        #   실패원인: 0=없음(정지OK) 1=자이로크기 2=가속도크기 3=진동 4=★자이로분산
        fail_code = {'': 0.0, 'gyro': 1.0, 'accel': 2.0,
                     'vibration': 3.0, 'gyro_var': 4.0}.get(
            getattr(self, '_d_fail', ''), 0.0)
        d.data = [float(self._bias[2]), float(math.degrees(self._bias[2])),
                  1.0 if still else 0.0, float(self._still_accum),
                  float(gz), float(gz - self._bias[2]),
                  float(getattr(self, '_d_gmag', 0.0)),
                  float(getattr(self, '_d_amag', 0.0)),
                  float(getattr(self, '_d_var', 0.0)),
                  fail_code,
                  float(getattr(self, '_d_gvar', 0.0))]
        self._pub_diag.publish(d)

        # 정지 판정이 한 번도 안 됐으면 원인을 로그로 알려줍니다.
        if self._still_accum < 0.1 and getattr(self, '_d_fail', ''):
            reason = {'gyro_var': f'자이로 분산 {getattr(self,"_d_gvar",0.0):.3e} '
                                  f'>= 임계 {self._th_gvar:.3e}  '
                                  f'(로봇이 실제로 흔들리는 중입니다)',
                      'gyro':     f'자이로 크기 {self._d_gmag:.4f} >= 임계 {self._th_gyro}',
                      'accel':    f'|가속도| {self._d_amag:.3f} 가 g(9.807)에서 '
                                  f'{abs(self._d_amag-_G):.3f} 벗어남 >= 임계 {self._th_aband}',
                      'vibration':f'가속도 분산 {self._d_var:.4f} >= 임계 {self._th_avar}'
                      }[self._d_fail]
            self.get_logger().warn(
                f'[바이어스] 정지 판정 실패 — {reason}\n'
                f'   로봇이 실제로 정지해 있는데도 이러면 해당 임계값을 완화하십시오.',
                throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ #
    def _is_still(self, t, gx, gy, gz, ax, ay, az) -> bool:
        """
        IMU 자체 정보만으로 정지 판정.
        외부 토픽(cmd_vel, 엔코더)에 의존하지 않는 이유: 명령이 0이어도 로봇이
        관성으로 밀리거나 사람이 밀 수 있고, 그때 바이어스를 갱신하면 오염됩니다.

        ★ v3 핵심 변경: "자이로 크기"를 주 판정에서 내리고 "자이로 분산"을 올렸습니다.

        [왜 v2 가 절대 정지 판정을 못 했는가 — 순환 논리]
          v2 는 |gyro| < 0.03 을 요구했습니다. 그런데 |gyro| 안에는 지금 우리가
          추정하려는 바로 그 바이어스가 들어 있습니다. 실측값:
              gz = 0.01239,  |gyro| = 0.041283  ->  gx,gy 가 0.03938 을 기여
              (= 2.26 deg/s. MPU6050 초기 바이어스로 지극히 정상 범위)
          즉 "바이어스를 없애려면 먼저 바이어스가 없어야 한다"를 요구한 셈입니다.
          바이어스는 상수이므로 분산에는 전혀 기여하지 않습니다(Var[g+b] = Var[g]).
          따라서 분산은 바이어스에 불변(bias-invariant)이며, 정지 판정의
          올바른 통계량입니다.

        [가속도 임계도 같은 이유로 완화]
          실측 |accel| = 10.341 -> |g| 에서 0.534 m/s^2 (약 55 mg) 벗어남.
          MPU6050 zero-g offset 스펙이 ±50~80 mg 이므로 이것도 정상입니다.
          v2 의 0.35 (36 mg) 임계는 물리적으로 만족 불가능한 값이었습니다.

        조건 4가지를 모두 만족해야 정지:
          1) ★ 자이로 윈도우 분산이 작다   (주 지표. 바이어스 불변)
          2) 자이로 크기가 지나치게 크지 않다 (보조. 대회전 즉시 차단용, 느슨함)
          3) 가속도 크기가 |g| 근방이다      (선가속 없음 + 바이어스 여유)
          4) 가속도 크기의 윈도우 분산이 작다 (진동/충격 없음)
        1번과 4번이 실질적인 판정자입니다. 등속 활주 중에도 2,3 은 만족할 수
        있지만, 실제 바닥 위 주행이면 미세 진동이 1,4 에 반드시 잡힙니다.
        """
        gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
        amag = math.sqrt(ax * ax + ay * ay + az * az)

        self._win.append((t, amag, gx, gy, gz))
        while self._win and (t - self._win[0][0]) > self._win_sec:
            self._win.popleft()

        n = len(self._win)
        if n < 5:
            return False

        # ── 가속도 크기 분산 ────────────────────────────────────────
        avals = [w[1] for w in self._win]
        amean = sum(avals) / n
        var = sum((v - amean) ** 2 for v in avals) / n

        # ── ★ 자이로 3축 분산 (바이어스 불변 통계량) ────────────────
        #   각 축의 표본분산을 따로 구해 그중 최댓값을 씁니다.
        #     Var[g_i] = E[(g_i - mean_i)^2]
        #   상수 바이어스 b 는 mean 에 그대로 흡수되어 (g+b) - (mean+b) = g - mean
        #   이 되므로 분산에서 완전히 소거됩니다. 이것이 바이어스 불변의 근거입니다.
        #   최댓값(sum 이 아니라 max)을 쓰는 이유: 한 축만 흔들려도 정지가 아닙니다.
        gvar = 0.0
        for axis in (2, 3, 4):            # 튜플 인덱스 2,3,4 = gx,gy,gz
            gvals = [w[axis] for w in self._win]
            gmean = sum(gvals) / n
            v_ax = sum((v - gmean) ** 2 for v in gvals) / n
            if v_ax > gvar:
                gvar = v_ax

        # ★ [추가] 어느 조건이 막고 있는지 진단으로 노출합니다.
        #   "정지인데 bias 가 0" 일 때 넷 중 무엇이 실패했는지 알아야 고칩니다.
        self._d_gmag, self._d_amag, self._d_var = gmag, amag, var
        self._d_gvar = gvar
        self._d_fail = ('gyro_var' if gvar >= self._th_gvar else
                        'gyro' if gmag >= self._th_gyro else
                        'accel' if abs(amag - _G) >= self._th_aband else
                        'vibration' if var >= self._th_avar else '')

        ok = (gvar < self._th_gvar
              and gmag < self._th_gyro
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