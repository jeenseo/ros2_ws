#!/usr/bin/env python3
"""
motor_node.py
=============
ROS 2 Node: Mecanum Wheel IK + Hybrid Encoder/Command Odometry
            + 모듈화된 적응형 공분산(Adaptive Covariance) 엔진

[Stage 1 : Baseline Stabilization]
  이 버전은 3단계 로드맵 중 1단계에 해당합니다.
    Stage 1 (현재) : 정적 스케일 보정 + 선형(Linear) 동적 공분산
    Stage 2        : LiDAR 퇴화(Degeneracy) 감지 연동  -> rf2o_covariance_relay.py
    Stage 3        : 퍼지 추론 엔진(Fuzzy Inference)    -> FuzzyCovarianceModel 교체만 하면 됨

[이번 리팩터링의 핵심 변경점]
  1. 공분산 계산 로직을 ROS 의존성이 전혀 없는 순수 파이썬 모듈로 분리
     (CovarianceModel ABC -> LinearCovarianceModel). 3단계에서 FuzzyCovarianceModel을
     끼워 넣기만 하면 되고, ROS 없이 pytest로 단위 테스트가 가능합니다.
  2. [중요] EKF가 실제로 읽는 것은 twist.covariance 입니다.
     ekf.yaml의 odom1_config는 pose(X,Y,Yaw)가 전부 false이므로, 기존 코드가 팽창시키던
     pose.covariance[0] 은 EKF에 아무런 영향을 주지 못하고 있었습니다.
     -> 이번 버전은 twist.covariance 를 1차 타깃으로 삼고, pose.covariance는
        (진단용 + 향후 pose 융합 재활성화 대비) 속도 불확실성의 시간 적분으로 계산합니다.
  3. [중요] 슬립 감지 지표를 '명령-엔코더 잔차' -> 'IMU 자이로-엔코더 잔차'로 교체.
     STM32가 엔코더 기반 PID 폐루프를 돌리고 있으면 바퀴가 헛돌아도 엔코더는 명령을
     그대로 따라갑니다. 즉 |cmd_vx - enc_vx| 는 슬립이 심할수록 오히려 0에 수렴합니다.
     실제로 관측 가능한 슬립 증거는 IMU(자이로/가속도)와 엔코더의 불일치뿐입니다.
  4. 정적 스케일 보정 파라미터(encoder_scale_x / _y / _yaw) 추가.
     편향(bias)은 공분산 튜닝으로 제거할 수 없습니다. 오직 보정만이 답입니다.
  5. ZUPT(Zero-Velocity Update), cmd_vel 워치독, 틱 손실 버그, Yaw 정규화 수정.


═══════════════════════════════════════════════════════════════════════════════
[Stage 1b : 하드웨어 결함 대응 — RL(좌후륜) 부상 + 전륜 2개만 엔코더 장착]
═══════════════════════════════════════════════════════════════════════════════

▣ 문제의 성격 규정 : 이것은 '노이즈'가 아니라 '관측성(observability) 결손'이다
  메카넘의 요레이트는 4륜 전체에서만 복원됩니다.
      wz = (r / (4(lx+ly))) * (-w_fl + w_fr - w_rl + w_rr)
  전륜 2개만 있으면 위 식의 절반이 비어 있어, 어떤 계수를 곱해도 wz를 복원할 수 없습니다.
  더구나 메카넘 롤러는 측면 미끄러짐을 '허용'하도록 설계되어 있으므로, 차체가 회전해도
  전륜 두 개의 롤링 엔코더는 얼마든지 동일한 값을 낼 수 있습니다.
  => 즉 랭크 결손(rank deficiency)이며, R 행렬로는 절대 해결되지 않습니다.
     (Stage 1에서 "R로는 편향을 못 없앤다"고 했던 것과 정확히 같은 종류의 문제)

▣ 물리 신호 방향 확인 (RL 부상 시)
  구동력 f_i 로부터의 차체 합력 (사용 중인 IK의 전치행렬):
      Fx = f_fl + f_fr + f_rl + f_rr
      Fy = -f_fl + f_fr + f_rl - f_rr
      Mz = (-f_fl + f_fr - f_rl + f_rr) * L
  직진 명령(모두 f)에서 f_rl = 0 이면:
      Fx = 3f,  Fy = -f (차체 우측),  Mz = +fL (반시계 = 좌회전)
  => 요 모멘트가 지배적으로 나타나 경로가 '좌측으로 휘는' 현상. 보고된 증상과 일치합니다.

▣ 대응 3축
  1. [추정] 요 소스를 엔코더 -> 자이로로 전환 (use_gyro_for_yaw).
     엔코더가 정직하게 측정할 수 있는 것은 '전진 롤링 거리' 딱 하나뿐입니다.
  2. [보정] 단일 스칼라 -> (encoder_scale_x, encoder_balance) 2파라미터로 분해.
     단, balance는 현 결함 상태에서 '관측 불가'이므로 1.0 고정이 정답입니다(아래 상세).
  3. [제어] YawRateCompensator — IMU 자이로 폐루프 PI로 쏠림 자체를 상쇄.
     추정으로 증상을 가리는 대신 원인을 눌러버리는 접근. 기본 OFF.

▣ 그러나 근본 해결은 소프트웨어가 아닙니다
  바퀴가 떠 있는 것은 기구 문제입니다. 심(shim)/서스펜션/프레임 수평을 먼저 잡으십시오.
  소프트웨어 보상은 배터리 잔량, 적재 하중, 바닥재가 바뀌면 그대로 무너집니다.
  아래 코드는 '기구를 고칠 때까지 주행과 캘리브레이션을 가능하게 하는 임시 버팀목'입니다.


═══════════════════════════════════════════════════════════════════════════════
[Stage 1c : STM32 펌웨어(motor.c/h) 실물 분석 결과 반영]
═══════════════════════════════════════════════════════════════════════════════

★ 발견 1 — 좌측 PID 가 FL 과 RL 을 '같은 PWM' 으로 묶어 구동한다
    _drive_side_closed(&s_pid_left, meas_left,
                       GPIO_PIN_2, htim1, CH1, FL_DIR_INVERT,   /* FL */
                       GPIO_PIN_0, htim2, CH1, RL_DIR_INVERT);  /* RL */
  즉 FL 과 RL 은 하나의 명령을 공유하고, 피드백은 TIM3(=FL) 하나뿐입니다.
  RL 이 떠 있으면:
    - PID 는 접지된 FL 을 목표 RPM 까지 끌어올리려 PWM 을 올린다
    - 그 PWM 을 무부하 RL 이 그대로 받아 공중에서 과속 회전한다
    - 좌측 추진은 FL 1개, 우측 추진은 FR+RR 2개 => 좌우 추진력 비대칭 = 쏠림의 정체
  ⚠ 주행 중 RL 이 순간 접지하면 과속 상태의 바퀴가 바닥을 때립니다. 저속으로 시험하십시오.

★ 발견 2 — 엔코더 요(yaw)는 Vy 에 오염되어 있다 (펌웨어 주석의 전제가 깨짐)
  motor.c 주석은 "좌측 엔코더 = (FL+RL)/2 이므로 Vy 가 소거된다"를 전제합니다.
  그러나 실제 엔코더는 FL 하나에만 달려 있습니다. 메카넘 IK 를 그대로 대입하면
      FL = Vx − Vy − Wz·l ,  FR = Vx + Vy + Wz·l
      (FL + FR)/2 = Vx                      <- ★ Vx 는 정확하다 (Vy, Wz 가 완전 소거)
      (FR − FL)/(2l) = Wz + Vy/l            <- ★ 요는 Vy 로 오염된다
  l = 0.505 이므로 계수는 1/l = 1.98 rad/s per m/s.
  게걸음 0.2 m/s 만 해도 0.396 rad/s 의 가짜 요레이트가 얹힙니다.
  => (a) EKF 에 엔코더 Vx 만 넣기로 한 Stage 1b 결정이 기하학적으로도 옳았음이 확인됨
     (b) 아래 코드에서 delta_yaw_enc 에 −cmd_vy·dt/l 보정을 넣어, 게걸음이
         가짜 슬립 증거(r_gyro)를 만들지 않도록 막습니다.

★ 발견 3 — PID 는 사실상 무력하다 (피드포워드 지배)
  MOTOR_KP=2.0 인데 Zone2 FF 기울기는 (9999−2000)/(150−31.8) = 67.7 PWM/RPM 입니다.
  즉 1 RPM 오차를 메우는 데 FF 는 67.7 PWM 을 쓰는데 P 항은 2 PWM 만 냅니다 (약 1/34).
  적분도 INTEGRAL_LIMIT=300, KI=1.0 이라 최대 기여가 300 PWM 에 불과합니다.
  => 실질적으로 '개루프 PWM→RPM 룩업'으로 동작하며, 부하가 걸리면 목표 RPM 에 못 미칩니다.
     이것이 슬립이 속도에 비례해 커지는 이유이고, 아래 속도 의존 스케일 모델의 근거입니다.
  (권장 펌웨어 수정: MOTOR_KP 를 20~40 수준으로. 단 별도 검증 후 적용하십시오.)

★ 발견 4 — mpu6050_node.py 에 자이로 바이어스 보정이 전혀 없다
  gz = raw/131.0*DEG_TO_RAD 로 끝입니다. MPU6050 출고 바이어스는 통상 1~3 °/s 이고,
  20초 주행이면 20~60° 의 가짜 방위각이 쌓입니다. imu_gyro_bias_node.py 를 반드시
  경유시키십시오. 아울러 publish_hz 가 20 Hz 인데, 요레이트 적분 소스로는 낮습니다
  (회전 1.5 rad/s 에서 한 샘플당 4.3° 양자화). 50~100 Hz 로 올리길 권합니다.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Float32MultiArray

import can


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION A. 적응형 공분산 엔진 (ROS 의존성 없음 / 그대로 covariance_models.py 로 분리 가능)
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MotionEvidence:
    """
    공분산 모델에 넘기는 '증거(Evidence)' 묶음.
    Stage 3의 퍼지 추론 엔진도 정확히 이 구조체만 입력으로 받으면 되므로,
    모델을 교체해도 motor_node 본체는 한 줄도 바뀌지 않습니다.
    """
    dt:        float          # 이번 엔코더 프레임 간격 [s]
    cmd_vx:    float          # 제어 명령 전진 속도 [m/s]
    cmd_vy:    float          # 제어 명령 횡이동 속도 [m/s]
    cmd_wz:    float          # 제어 명령 각속도 [rad/s]
    enc_vx:    float          # 엔코더 측정 전진 속도 [m/s]  (스케일 보정 후)
    enc_vy:    float          # 횡이동 속도 [m/s] (※ 현재는 명령 적분값 = 개루프)
    enc_wz:    float          # 엔코더 측정 각속도 [rad/s]   (스케일 보정 후)
    imu_wz:    float          # IMU 자이로 각속도 [rad/s]
    imu_ax:    float          # IMU 전진축 가속도 [m/s^2]
    imu_valid: bool           # IMU 데이터 신선도 (stale이면 False)
    cmd_fresh: bool           # cmd_vel 워치독 통과 여부
    stationary: bool          # ZUPT 판정 (정지 상태)


@dataclass
class CovarianceOutput:
    """모델이 돌려주는 결과. 분산(variance, sigma^2) 단위입니다."""
    var_vx:  float
    var_vy:  float
    var_wz:  float
    var_x:   float
    var_y:   float
    var_yaw: float
    slip_index: float = 0.0   # 0.0(정상) ~ 1.0(극심한 슬립). 진단/퍼지 연동용
    debug: dict = field(default_factory=dict)


class CovarianceModel(ABC):
    """
    동적 공분산 모델 인터페이스.

    Stage 3에서 퍼지 엔진을 붙일 때는
        class FuzzyCovarianceModel(CovarianceModel):
            def evaluate(self, ev): ...
    를 구현하고 MotorNode의 `covariance_model` 파라미터만 'fuzzy'로 바꾸면 됩니다.
    """

    @abstractmethod
    def evaluate(self, ev: MotionEvidence) -> CovarianceOutput:
        ...

    def reset(self) -> None:
        """오도메트리 리셋 시 내부 누적 상태 초기화."""
        return None


class LinearCovarianceModel(CovarianceModel):
    """
    ── 선형(비퍼지) 적응형 공분산 모델 ─────────────────────────────────────

    ▣ 설계 원리 1 : 분산은 '더한다', 표준편차는 '더하지 않는다'
      서로 독립인 오차 요인들은 분산(sigma^2) 레벨에서 선형 합성됩니다.
      기존 코드의 `0.05 + slip_error*0.2` 는 단위가 섞인 임의 합이었으므로,
      각 오차 요인을 '표준편차 기여분 [m/s]'으로 환산한 뒤 제곱합으로 바꿉니다.

          sigma_vx^2 = sigma_0^2                (센서 바닥 노이즈)
                     + (k_v * |v|)^2           (속도 비례 슬립: 빠를수록 미끄러짐)
                     + (k_w * |w|)^2           (회전 유발 슬립: 메카넘 롤러 측면 미끄럼)
                     + (k_c * r_cmd)^2         (명령-엔코더 잔차: 약한 증거)
                     + (k_g * r_gyro)^2        (엔코더-자이로 잔차: 강한 증거)
                     + (k_a * r_acc)^2         (엔코더 가속도-IMU 가속도 잔차: Stage 3 훅)

      각 계수는 '해당 요인이 1단위일 때 실제로 발생하는 속도 오차[m/s]'로 읽힙니다.
      즉 k_w = 0.8 은 "1 rad/s로 돌 때 전진 속도 추정이 0.8 m/s 만큼 틀어진다"는 의미이고,
      이는 실측(회전 주행 시 5m -> 10m, 즉 100% 과대)과 정합됩니다.

    ▣ 설계 원리 2 : 비대칭 엔벨로프 (Fast Attack / Slow Release)
      기존의 `min(0.5, ...)` 하드 클램프는 프레임마다 값이 튀어 EKF 이득이 채터링합니다.
      슬립은 '순간적으로 시작되고 서서히 회복'되므로, 1차 저역통과를 비대칭으로 겁니다.

          alpha = 1 - exp(-dt / tau)          (지수 이동평균의 정확한 이산화)
          tau   = tau_attack (증가 시, 빠름)  /  tau_release (감소 시, 느림)

      EKF 입장에서 이것은 "한 번 신뢰를 잃은 센서는 천천히 신뢰를 회복한다"는
      보수적(conservative) 필터링이며, 발산을 막는 방향으로 안전합니다.

    ▣ 설계 원리 3 : 위치 공분산은 속도 공분산의 시간 적분
      슬립 오차는 백색잡음이 아니라 '편향(bias)'에 가깝습니다. 백색잡음이면
      sigma_x^2 += sigma_v^2 * dt^2 이지만, 완전 상관된 편향은 표준편차가 선형 누적합니다.

          sigma_x  <- sigma_x + sigma_vx * dt        (worst-case, 상관 오차)
          var_x     = sigma_x^2

      이것이 Foxglove에서 /odom_motor.pose.covariance[0] 이 주행할수록 단조 증가하는
      물리적으로 올바른 모양입니다. (기존 코드는 순간값이라 평평했습니다.)
    """

    def __init__(
        self,
        # ── 바닥(floor) 노이즈: 슬립이 전혀 없어도 남는 양자화/측정 노이즈 ──
        sigma0_vx: float = 0.02,      # [m/s]
        # [Stage 1b] 0.10 -> 0.25 상향. Vy는 실측이 아니라 '명령 적분값'인데,
        #   RL 부상으로 인해 직진 명령(cmd_vy=0) 중에도 차체는 실제로 옆으로 밀립니다.
        #   즉 이 채널은 단순히 노이즈가 큰 게 아니라 '상시 거짓말'을 합니다.
        #   ekf.yaml에서 odom1의 Vy 융합을 아예 껐고(Stage 1b), 이 값은 2차 방어선입니다.
        sigma0_vy: float = 0.25,      # [m/s]
        sigma0_wz: float = 0.05,      # [rad/s]
        # ── 오차 요인별 이득 ──
        # [Stage 1c] 0.10 -> 0.15 상향. 실측상 슬립이 속도에 거의 정비례하고(계수 0.91),
        #   그 모델을 2점으로만 적합했기 때문에 잔차 불확실성이 큽니다.
        #   권장값:  slip 모델 ON + 운용 속도에서 캘리브 -> 0.10
        #           단일 정적 k 로 2배 속도 범위를 커버   -> 0.25
        k_v: float = 0.15,            # 속도 비례 슬립 [m/s per m/s]
        # ★ k_w : 메카넘 핵심 항. verify_tuning.py 스윕 결과(회전 시나리오 최종 오차)
        #     k_w=0.50 -> +9.7% | 0.65 -> +6.7% | 0.80 -> +4.5% | 1.00 -> +2.5% | 1.20 -> +1.1%
        #   값이 클수록 회전 중 엔코더를 더 과감히 폐기합니다. 다만 복도에서 완만한 곡선
        #   주행 시 rf2o도 함께 나쁘면 의지할 소스가 사라지므로 무한정 키우면 안 됩니다.
        #   1.00을 기본값으로 채택(정확도/견고성 균형). 곡선 주행이 불안하면 0.80으로 낮추십시오.
        k_w: float = 1.00,            # 회전 유발 슬립 [m/s per rad/s]
        k_c: float = 0.30,            # 명령-엔코더 잔차 [m/s per m/s] (폐루프면 거의 0)
        k_g: float = 1.20,            # 엔코더-자이로 각속도 잔차 [m/s per rad/s] ★ 주 증거
        k_a: float = 0.00,            # 가속도 잔차 [m/s per m/s^2] (Stage 3에서 활성화)
        k_vy_cmd: float = 0.50,       # Vy 개루프 불확실성 [m/s per m/s]
        k_wz_gyro: float = 0.60,      # Wz 잔차 -> Wz 분산 [rad/s per rad/s]
        # ── 엔벨로프 시상수 ──
        tau_attack: float = 0.05,     # [s] 슬립 감지 시 즉시 팽창
        tau_release: float = 0.80,    # [s] 회복은 천천히
        # ── 클램프 ──
        var_min_v: float = 1e-4,      # sigma = 0.01 m/s
        var_max_v: float = 4.0,       # sigma = 2.0  m/s (사실상 "이 센서 무시")
        var_min_w: float = 1e-4,
        var_max_w: float = 2.0,
        # ── 위치 공분산 누적 ──
        sigma_x_max: float = 3.0,     # [m] 누적 상한
        sigma_yaw_max: float = 1.5,   # [rad] 누적 상한
        # ── ZUPT ──
        zupt_var_v: float = 1e-5,     # 정지 확신 시 분산 (sigma ~ 3mm/s)
        zupt_var_w: float = 1e-5,
        # ── slip_index 정규화 기준 ──
        slip_ref_var: float = 0.25,   # sigma = 0.5 m/s 를 slip_index ~ 0.5 로 본다
        # ── [Stage 1b] 만성 요 편향 추정기 ──────────────────────────────
        #   RL 부상 같은 기구 결함은 직진 명령 중에도 (enc_wz - imu_wz) 에 '상시 오프셋'을
        #   만듭니다. 이를 그대로 슬립 증거로 쓰면 var_vx 가 영구 팽창해서, 정작 엔코더를
        #   믿어야 하는 직진 구간에서 엔코더가 통째로 죽어버립니다.
        #     예) 상시 오프셋 0.12 rad/s -> (1.2*0.12)^2 = 0.0207
        #         직진 기본 var_vx 0.0029 대비 8배 팽창 = 치명적 부작용
        #   따라서 '만성 오프셋'을 느리게 추정해 빼고, 그 잔차만 슬립 증거로 씁니다.
        #   => r_gyro 가 "고질병 대비 얼마나 더 어긋났는가"를 재게 되어 의미가 정확해집니다.
        bias_tau: float = 10.0,       # [s] 편향 추정 시상수 (과도 현상은 절대 못 쫓아오게 길게)
        bias_limit: float = 0.50,     # [rad/s] 추정 편향 절대 상한 (안전장치)
        bias_gate_wz: float = 0.05,   # [rad/s] |cmd_wz| 가 이보다 작을 때만 = 직진 중일 때만 갱신
        bias_gate_vx: float = 0.05,   # [m/s]   |cmd_vx| 가 이보다 클 때만 = 실제 주행 중일 때만
    ) -> None:
        self.sigma0_vx, self.sigma0_vy, self.sigma0_wz = sigma0_vx, sigma0_vy, sigma0_wz
        self.k_v, self.k_w, self.k_c, self.k_g, self.k_a = k_v, k_w, k_c, k_g, k_a
        self.k_vy_cmd, self.k_wz_gyro = k_vy_cmd, k_wz_gyro
        self.tau_attack, self.tau_release = tau_attack, tau_release
        self.var_min_v, self.var_max_v = var_min_v, var_max_v
        self.var_min_w, self.var_max_w = var_min_w, var_max_w
        self.sigma_x_max, self.sigma_yaw_max = sigma_x_max, sigma_yaw_max
        self.zupt_var_v, self.zupt_var_w = zupt_var_v, zupt_var_w
        self.slip_ref_var = slip_ref_var
        self.bias_tau, self.bias_limit = bias_tau, bias_limit
        self.bias_gate_wz, self.bias_gate_vx = bias_gate_wz, bias_gate_vx
        self._yaw_bias = 0.0          # 만성 (enc_wz - imu_wz) 오프셋 추정치 [rad/s]

        # ── 내부 상태 ──
        self._s_vx = math.sqrt(var_min_v)    # 엔벨로프 필터가 들고 있는 표준편차 [m/s]
        self._s_vy = math.sqrt(var_min_v)
        self._s_wz = math.sqrt(var_min_w)
        self._sig_x = 1e-3                   # 누적 위치 표준편차 [m]
        self._sig_y = 1e-3
        self._sig_yaw = 1e-3                 # 누적 방위 표준편차 [rad]
        self._prev_enc_vx: float | None = None

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self._s_vx = math.sqrt(self.var_min_v)
        self._s_vy = math.sqrt(self.var_min_v)
        self._s_wz = math.sqrt(self.var_min_w)
        self._sig_x = self._sig_y = self._sig_yaw = 1e-3
        self._prev_enc_vx = None
        # 주의: _yaw_bias 는 리셋하지 않습니다. 기구 결함은 오도메트리 리셋과 무관하게
        # 계속 존재하므로, 애써 수렴시킨 추정치를 버릴 이유가 없습니다.

    @property
    def yaw_bias(self) -> float:
        """추정된 만성 요 편향 [rad/s]. 진단 퍼블리시 및 제어 피드포워드용."""
        return self._yaw_bias

    # ------------------------------------------------------------------ #
    @staticmethod
    def _envelope(prev: float, target: float, dt: float,
                  tau_attack: float, tau_release: float) -> float:
        """
        비대칭 1차 저역통과 (Attack/Release Envelope Follower).

        연속시간 1차 시스템 dy/dt = (u - y)/tau 의 정확한 이산 해:
            y[k] = y[k-1] + (1 - exp(-dt/tau)) * (u[k] - y[k-1])
        dt가 불규칙해도(엔코더 CAN 프레임은 지터가 있음) 시상수가 보존되는 것이
        단순 `y += 0.1*(u-y)` 대비 이 형태의 장점입니다.
        """
        tau = tau_attack if target > prev else tau_release
        alpha = 1.0 - math.exp(-dt / max(tau, 1e-6))
        return prev + alpha * (target - prev)

    # ------------------------------------------------------------------ #
    def evaluate(self, ev: MotionEvidence) -> CovarianceOutput:
        dt = max(ev.dt, 1e-3)

        # ── (0) 엔코더 가속도 (Stage 3 퍼지 엔진의 주 입력이 될 항) ────────
        #     "명령 속도는 큰데 IMU 가속도가 작고 엔코더 속도가 크다" = 극심한 슬립
        #     이를 선형화한 형태가 r_acc = |a_encoder - a_imu| 입니다.
        if self._prev_enc_vx is None:
            enc_ax = 0.0
        else:
            enc_ax = (ev.enc_vx - self._prev_enc_vx) / dt
        self._prev_enc_vx = ev.enc_vx
        r_acc = abs(enc_ax - ev.imu_ax) if ev.imu_valid else 0.0

        # ── (1) 슬립 증거(잔차) 채널 ─────────────────────────────────────
        r_cmd = abs(ev.cmd_vx - ev.enc_vx) if ev.cmd_fresh else 0.0

        # ★ 가장 신뢰도 높은 슬립 증거: 바퀴가 말하는 회전 vs 자이로가 말하는 회전
        #   [Stage 1b] 단, 만성 기구 결함(RL 부상)이 만드는 '상시 오프셋'은 먼저 제거합니다.
        raw_gyro_res = (ev.enc_wz - ev.imu_wz) if ev.imu_valid else 0.0

        #   편향 갱신 게이트: '직진 명령 중 + 실제 주행 중' 일 때만.
        #   직진 명령 중에 나타나는 요 불일치는 정의상 명령된 회전이 아니므로
        #   전부 기구 결함(또는 정렬 오차)입니다. 회전 중에는 진짜 슬립이 섞이므로 갱신 금지.
        if (ev.imu_valid and not ev.stationary and ev.cmd_fresh
                and abs(ev.cmd_wz) < self.bias_gate_wz
                and abs(ev.cmd_vx) > self.bias_gate_vx):
            a_bias = 1.0 - math.exp(-dt / self.bias_tau)     # tau=10s -> 과도 현상 추종 불가
            self._yaw_bias += a_bias * (raw_gyro_res - self._yaw_bias)
            self._yaw_bias = max(-self.bias_limit, min(self.bias_limit, self._yaw_bias))

        #   최종 슬립 증거 = '고질병 대비 추가 이탈량'
        r_gyro = abs(raw_gyro_res - self._yaw_bias) if ev.imu_valid else 0.0
        # ── 회전량 : ★ [Stage 1b] imu_wz 를 이 항에서 제외했습니다 ──────────
        #   k_w 항이 모델링하는 것은 '구동계가 만들어내는 롤러 측면 스크럽'입니다.
        #   그 원인은 바퀴 간 속도 차이(=명령된 회전 또는 엔코더가 보고한 회전)이지,
        #   차체가 회전한다는 사실 자체가 아닙니다.
        #
        #   [왜 중요한가] RL 부상으로 차체가 상시 0.15 rad/s 로 휘는 상황에서
        #   imu_wz 를 포함시키면 (1.0*0.15)^2 = 0.0225 가 항상 더해져, 직진 기본값
        #   0.0029 대비 var_vx 가 9배로 영구 팽창합니다. 정작 엔코더를 믿어야 하는
        #   직진 구간에서 엔코더 가중치가 95% -> 70% 로 떨어져 복도 거리가 15% 나빠집니다.
        #
        #   [물리적으로도 이쪽이 옳다] 만성 쏠림 중에도 전륜은 접지 상태로 정상 구름을 합니다.
        #   차체 요가 전륜 축에 만드는 성분은 '횡방향'이라 전진 롤링 거리를 오염시키지 않습니다.
        #   즉 쏠림은 '방위'를 망가뜨리지 '전진 거리'를 망가뜨리지 않습니다.
        #   명령 없이 갑자기 차체가 도는 진짜 이상 상황은 아래 r_gyro 항이 잡습니다(상보 관계).
        omega = max(abs(ev.enc_wz), abs(ev.cmd_wz))
        speed = abs(ev.enc_vx)

        # ── (2) 분산 합성 (독립 오차원의 제곱합) ─────────────────────────
        var_vx_raw = (
            self.sigma0_vx ** 2
            + (self.k_v * speed) ** 2
            + (self.k_w * omega) ** 2       # ★ 회전 시 엔코더 Vx를 사실상 폐기시키는 항
            + (self.k_c * r_cmd) ** 2
            + (self.k_g * r_gyro) ** 2      # ★ 측정 기반 슬립 증거
            + (self.k_a * r_acc) ** 2
        )
        var_vy_raw = (
            self.sigma0_vy ** 2
            + (self.k_vy_cmd * abs(ev.cmd_vy)) ** 2   # Vy는 개루프이므로 명령 크기에 비례
            + (self.k_w * omega) ** 2
            + (self.k_g * r_gyro) ** 2
        )
        var_wz_raw = (
            self.sigma0_wz ** 2
            + (self.k_wz_gyro * r_gyro) ** 2
            + (0.10 * omega) ** 2
        )

        # ── (3) 비대칭 엔벨로프 (표준편차 도메인에서 적용) ──────────────
        self._s_vx = self._envelope(self._s_vx, math.sqrt(var_vx_raw), dt,
                                    self.tau_attack, self.tau_release)
        self._s_vy = self._envelope(self._s_vy, math.sqrt(var_vy_raw), dt,
                                    self.tau_attack, self.tau_release)
        self._s_wz = self._envelope(self._s_wz, math.sqrt(var_wz_raw), dt,
                                    self.tau_attack, self.tau_release)

        var_vx = min(max(self._s_vx ** 2, self.var_min_v), self.var_max_v)
        var_vy = min(max(self._s_vy ** 2, self.var_min_v), self.var_max_v)
        var_wz = min(max(self._s_wz ** 2, self.var_min_w), self.var_max_w)

        # ── (4) ZUPT : 정지 확신 시 속도 분산을 극단적으로 축소 ─────────
        #     EKF가 "속도는 정확히 0"이라는 강한 측정을 받게 되어 정지 중 드리프트가 멈춥니다.
        if ev.stationary:
            var_vx = self.zupt_var_v
            var_vy = self.zupt_var_v
            var_wz = self.zupt_var_w

        # ── (5) 위치 공분산 : 속도 표준편차의 시간 적분 (편향성 누적) ────
        if not ev.stationary:
            self._sig_x = min(self._sig_x + math.sqrt(var_vx) * dt, self.sigma_x_max)
            self._sig_y = min(self._sig_y + math.sqrt(var_vy) * dt, self.sigma_x_max)
            self._sig_yaw = min(self._sig_yaw + math.sqrt(var_wz) * dt, self.sigma_yaw_max)

        # ── (6) slip_index : 0~1 정규화 (진단 및 Stage 3 퍼지 출력과 호환) ──
        slip_index = var_vx / (var_vx + self.slip_ref_var)

        return CovarianceOutput(
            var_vx=var_vx, var_vy=var_vy, var_wz=var_wz,
            var_x=self._sig_x ** 2, var_y=self._sig_y ** 2, var_yaw=self._sig_yaw ** 2,
            slip_index=slip_index,
            debug={'r_cmd': r_cmd, 'r_gyro': r_gyro, 'r_acc': r_acc, 'omega': omega,
                   'yaw_bias': self._yaw_bias, 'raw_gyro_res': raw_gyro_res},
        )


# ═══════════════════════════════════════════════════════════════════════════
#  [Stage 1b] 요레이트 폐루프 보상기 (제어 측 대응)
# ═══════════════════════════════════════════════════════════════════════════

class YawRateCompensator:
    """
    IMU 자이로를 피드백으로 쓰는 요레이트 PI 제어기.

    ▣ 왜 이게 정답에 가까운가
      RL 부상으로 인한 좌측 쏠림은 '추정 오차'가 아니라 '실제 물리 현상'입니다.
      아무리 EKF가 쏠림을 정확히 추정해도 로봇은 여전히 목표 경로를 벗어납니다.
      원인을 없애려면 제어 측에서 눌러야 하고, 그 도구가 이미 있습니다 — 자이로.

      개루프 힘 재분배(3륜 IK 역산)보다 이쪽이 우월한 이유:
        - 힘 모델은 RL의 접지 하중 비율(0~1)을 알아야 하는데 "살짝 떠 있다"는 미지수입니다.
        - 배터리 소모, 적재, 바닥 마찰이 바뀌면 힘 모델은 즉시 틀립니다.
        - PI는 원인이 무엇이든(부상, 타이어 마모, 바닥 기울기) 결과만 보고 상쇄합니다.

    ▣ Nav2와의 간섭
      Nav2(외루프, ~20Hz, 자세 기준) / 본 보상기(내루프, IMU 100Hz+, 요레이트 기준)로
      전형적인 캐스케이드 구조입니다. 내루프가 외루프보다 5배 이상 빠르므로 안정하며,
      오히려 플랜트를 Nav2의 내부 모델(명령대로 도는 로봇)에 가깝게 만들어 도움이 됩니다.

    ▣ 안전장치
      - 권한 제한(limit): 보상량을 ±0.35 rad/s 로 묶어 폭주 시에도 회전이 완만
      - 안티 와인드업: 적분항 자체를 limit 안으로 클램프 (조건부 적분)
      - 정지/E-Stop/모드 전환 시 reset() 필수
      - 기본 OFF. 반드시 넓은 공간에서 Kp만 먼저 올려보고 Ki를 나중에 넣으십시오.
    """

    def __init__(self, kp: float = 1.0, ki: float = 1.5,
                 limit: float = 0.35, deadband: float = 0.01) -> None:
        self.kp, self.ki = kp, ki
        self.limit = limit
        self.deadband = deadband     # [rad/s] 자이로 노이즈에 적분기가 끌려가지 않도록
        self._integral = 0.0
        self._last_out = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._last_out = 0.0

    @property
    def output(self) -> float:
        return self._last_out

    def update(self, target_wz: float, measured_wz: float,
               dt: float, active: bool) -> float:
        """보상량 [rad/s] 반환. cmd_wz 에 '더해서' IK로 넘기십시오."""
        if not active:
            # 정지 중에도 적분기를 살려두면 재출발 순간 킥이 발생합니다.
            self._integral *= math.exp(-dt / 0.5)   # 0.5s 시상수로 부드럽게 방전
            self._last_out = 0.0
            return 0.0

        err = target_wz - measured_wz
        if abs(err) < self.deadband:
            err = 0.0

        # 조건부 적분(anti-windup): 적분 기여분이 권한 한계를 넘으면 누적을 멈춥니다.
        cand = self._integral + err * dt
        if self.ki > 1e-9 and abs(self.ki * cand) <= self.limit:
            self._integral = cand
        # (한계에 걸린 상태에서 오차 부호가 뒤집히면 위 조건이 다시 참이 되어 자동 해제됨)

        out = self.kp * err + self.ki * self._integral
        out = max(-self.limit, min(self.limit, out))
        self._last_out = out
        return out


class FuzzyCovarianceModel(LinearCovarianceModel):
    """
    [Stage 3 자리표시자 — 아직 사용하지 마십시오]

    퍼지 추론 엔진을 붙일 때 이 클래스만 채우면 됩니다. 권장 구조:

      입력 멤버십 함수 (3개 입력, 각 3라벨: LOW / MED / HIGH)
        mu_cmd(ev.cmd_vx), mu_acc(ev.imu_ax), mu_enc(ev.enc_vx)
      규칙 베이스 (프로젝트 지침의 예시 규칙)
        R1: cmd=HIGH AND acc=LOW AND enc=HIGH  ->  slip=SEVERE
        R2: cmd=HIGH AND acc=HIGH AND enc=HIGH ->  slip=NONE
        ...
      역퍼지화(무게중심법) 결과 slip in [0,1] 을 아래처럼 곱하기만 하면 됩니다.
        out = super().evaluate(ev)
        gain = 1.0 + self.fuzzy_gain * slip**2
        out.var_vx *= gain ; out.var_wz *= gain
        return out

    선형 모델을 부모로 두었기 때문에 퍼지 규칙이 침묵(slip=0)해도
    Stage 1의 물리 기반 하한선은 그대로 유지됩니다 = 안전한 폴백.
    """

    def evaluate(self, ev: MotionEvidence) -> CovarianceOutput:  # pragma: no cover
        raise NotImplementedError('Stage 3에서 구현 예정. 현재는 LinearCovarianceModel 사용.')


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION B. ROS 2 노드
# ═══════════════════════════════════════════════════════════════════════════

_CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_IMU_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,   # sensor_data QoS 호환
    durability=DurabilityPolicy.VOLATILE,
)

# ── 물리 상수 ─────────────────────────────────────────────────────────────
_WHEEL_DIAMETER_M = 0.12
_WHEEL_CIRCUM_M   = math.pi * _WHEEL_DIAMETER_M
_ENCODER_CPR      = 1404                               # 13 PPR * 4체배 * 27 감속
_TRACK_WIDTH_M    = 0.51
_WHEELBASE_M      = 0.50
_METER_PER_TICK   = _WHEEL_CIRCUM_M / _ENCODER_CPR     # ≈ 2.6853e-4 m/tick

# 메카넘 IK 거리 상수 l = (track + wheelbase)/2, 요레이트 지레팔 = 2l = track + wheelbase
_MECANUM_L        = (_TRACK_WIDTH_M + _WHEELBASE_M) / 2.0   # 0.505 m
_YAW_LEVER        = _TRACK_WIDTH_M + _WHEELBASE_M           # 1.010 m

_MAX_VX, _MAX_VY, _MAX_WZ = 0.95, 0.95, 1.88
_MAX_SPEED   = 9999
_CAN_TX_ID    = 0x123
_CAN_RX_FB_ID = 0x124

# [Stage 1e] 한 프레임에 가능한 최대 엔코더 틱
#   150 RPM = 2.5 rev/s -> 2.5 * 1404 = 3510 ticks/s. 20ms 프레임 = 70 ticks.
#   여유 2배. 이보다 크면 노이즈이거나 프레임 유실 누적이므로 신뢰할 수 없습니다.
_MAX_TICKS_PER_FRAME = 140

# [Stage 1e] SocketCAN 에러 클래스 (linux/can/error.h)
#   ★ 이 비트들의 조합이 arbitration_id 로 올라오며, 0x123/0x124 와 겹칠 수 있습니다.
_CAN_ERR_CLASS = {
    0x001: 'TX_TIMEOUT', 0x002: 'LOSTARB',  0x004: 'CRTL',      0x008: 'PROT',
    0x010: 'TRX',        0x020: 'ACK',      0x040: 'BUSOFF',    0x080: 'BUSERROR',
    0x100: 'RESTARTED',  0x200: 'CNT',
}

# 진단 배열 인덱스 (Foxglove Plot에서 바로 꽂아 쓰십시오)
#   0:slip_index  1:sigma_vx  2:sigma_vy  3:sigma_wz  4:r_gyro(편향보상후)  5:r_cmd
#   6:r_acc       7:enc_wz    8:imu_wz    9:omega    10:stationary
#  [Stage 1b 추가]
#  11:yaw_bias    - 추정된 만성 요 편향 [rad/s]. ★ RL 부상 심각도의 정량 지표
#  12:raw_gyro_res- 편향 보상 전 원시 잔차. 11번과의 차이가 곧 순간 슬립
#  13:yaw_comp    - 요레이트 보상기 출력 [rad/s]. 0이면 보상기 OFF 또는 비활성
_DIAG_LAYOUT = ('slip_index', 'sigma_vx', 'sigma_vy', 'sigma_wz', 'r_gyro',
                'r_cmd', 'r_acc', 'enc_wz', 'imu_wz', 'omega', 'stationary',
                'yaw_bias', 'raw_gyro_res', 'yaw_comp')


def _normalize_angle(a: float) -> float:
    """Yaw를 (-pi, pi]로 정규화. 장시간 주행 시 float 정밀도 손실 방지."""
    return math.atan2(math.sin(a), math.cos(a))


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      _CAN_TX_ID)
        self.declare_parameter('max_speed',   _MAX_SPEED)
        self.declare_parameter('odom_frame',  'odom')
        self.declare_parameter('base_frame',  'base_footprint')
        self.declare_parameter('odom_topic',  '/odom_motor')
        self.declare_parameter('imu_topic',   '/imu/data')

        # ══ 스케일 보정 : [Stage 1b] 단일 스칼라 -> (scale, balance) 2파라미터 분해 ══
        #
        # ▣ 왜 스칼라 하나로는 부족한가 — 자유도 계산
        #   전륜 2채널 엔코더가 만들어내는 오차의 자유도는 2입니다.
        #     (a) 공통 스케일  : k_L, k_R 이 함께 커지거나 작아지는 성분 -> 거리 오차
        #     (b) 좌우 밸런스  : k_R / k_L 비율 성분                     -> 보고되는 요 오차
        #   여기에 차체의 실제 요(회전)는 자유도 0 — 전륜 2개로는 아예 관측이 불가능합니다.
        #   따라서 올바른 분해는 [스칼라 1개] 가 아니라 [스칼라 2개 + 센서 교체 1건] 입니다.
        #
        #       k_L = 2*scale / (1 + balance)
        #       k_R = 2*scale*balance / (1 + balance)
        #     검산:  (k_L + k_R)/2 = scale ,  k_R/k_L = balance   (직교 분해)
        #
        # ▣ ★ 그런데 balance 는 지금 '건드리면 안 되는' 파라미터입니다
        #   balance 를 조정하면 '보고되는' 요가 바뀔 뿐, 로봇이 실제로 도는 각도는 그대로입니다.
        #   자이로 값에 맞춰 balance 를 억지로 맞추는 것은 '오도메트리를 고장에 피팅'하는 것이며,
        #   그 계수는 특정 속도/하중/바닥에서만 유효하고 RL이 접지하는 순간 전부 틀립니다.
        #   => balance 는 1.0 으로 두고, 요는 자이로에서 가져옵니다(use_gyro_for_yaw).
        #      기구 결함을 고친 뒤에야 balance 캘리브레이션이 의미를 가집니다(가이드 문서 참조).
        # [Stage 1c] 요청하신 대로 좌/우를 직접 지정하는 형태로 바꿨습니다.
        #   calibrate_scales.py 가 실측 4개 값(X, Y, odom x, odom yaw)에서 계산해 줍니다.
        #   실측 데이터(X=5.0, Y=+0.83, odom x=6.9) 기준 산출값:
        #       encoder_scale_left  = 0.7138
        #       encoder_scale_right = 0.7620   (평균 0.7379, 우측이 +6.75%)
        #   ※ 평균 스케일 0.7379 는 odom yaw 값과 무관하게 거의 불변입니다
        #     (yaw 0.0/0.1/0.2 rad 로 바꿔도 0.7377~0.7379).
        #     반면 좌우 밸런스는 yaw 에 민감합니다(1.068 / 1.052 / 1.037).
        #     => 거리 보정은 지금 바로 신뢰 가능, 밸런스는 odom yaw 실측이 필요합니다.
        self.declare_parameter('encoder_scale_left',  1.0)
        self.declare_parameter('encoder_scale_right', 1.0)
        self.declare_parameter('encoder_scale_y',   1.0)
        self.declare_parameter('encoder_scale_yaw', 1.0)   # 자이로 폴백 경로에서만 사용

        # ══ [Stage 1c] ★ 속도 의존 슬립 모델 ══════════════════════════
        #   실측 2점에서 슬립이 속도에 거의 비례함이 확인되었습니다.
        #     CAN 4000 (바퀴 0.377 m/s): 엔코더 6.90 m / 실제 5.09 m = 1.355 (슬립 35.5%)
        #     CAN 8000 (바퀴 0.754 m/s): 엔코더 8.50 m / 실제 5.00 m = 1.700 (슬립 70.0%)
        #     회귀:  slip(v) = 0.0105 + 0.9144·v      (절편이 1% 수준 = 거의 정비례)
        #
        #   [왜 정비례인가] 위 '발견 3' 참조. PID 가 무력해 실질 개루프이므로 바퀴는
        #   PWM 룩업대로 돌고, 지면은 그만큼 못 따라옵니다. 토크 요구가 커질수록 슬립률↑.
        #
        #   [단일 상수로는 왜 안 되는가]
        #     저속(0.38 m/s)에서 잡은 k=0.738 을 고속(0.75 m/s)에 그대로 쓰면 +25% 거리 오차.
        #     반대로 고속 계수를 저속에 쓰면 -20%. 운용 속도 범위가 2배면 무시 못 합니다.
        #
        #   적용식:  k_side(v) = encoder_scale_side / (1 + slip_offset + slip_speed_coeff·|v_raw|)
        #     v_raw = 스케일 적용 '전' 엔코더 속도 (순환 참조 방지)
        #     기본값 0.0 = 비활성. 아래 값은 실측 2점 회귀 결과이며, 3~4점 확보 후 재적합 권장.
        #         slip_offset      = 0.0105
        #         slip_speed_coeff = 0.9144
        #   ※ encoder_scale_* 를 '어느 속도에서' 쟀는지와 반드시 짝을 맞추십시오.
        #     0.377 m/s 에서 잰 0.7138/0.7620 을 쓰면서 이 모델을 켜면 이중 보정이 됩니다.
        #     -> 이 모델을 켤 때는 encoder_scale_* 를 아래 '정규화 값'으로 교체하십시오.
        #        left  0.7138 × (1 + 0.0105 + 0.9144×0.377) = 0.9670
        #        right 0.7620 × (1 + 0.0105 + 0.9144×0.377) = 1.0323
        self.declare_parameter('slip_offset',      0.0)
        self.declare_parameter('slip_speed_coeff', 0.0)

        # ══ [Stage 1b] 요 소스 전환 ═══════════════════════════════════
        #   메카넘 요레이트는 4륜 전체가 있어야 복원됩니다(랭크 결손).
        #   전륜 2개만으로 계산한 delta_yaw 는 구조적으로 틀린 값이므로,
        #   IMU가 살아 있는 한 자이로 적분으로 대체합니다.
        #   ※ EKF는 odom1의 Yaw/Vyaw를 어차피 융합하지 않으므로(odom1_config),
        #     이 전환의 직접 효과는 '/odom_motor 궤적이 진실을 말하게 되는 것'입니다.
        #     Foxglove의 MOTOR_Only 곡선과 캘리브레이션 판정을 신뢰할 수 있게 됩니다.
        self.declare_parameter('use_gyro_for_yaw', True)

        # ══ [Stage 1d] ★ CAN 하트비트 / 통신 두절 안전장치 ═════════════
        #
        # [무엇을 막는가]
        #   기존 구조는 cmd_vel 콜백이 올 때만 CAN 을 보냈습니다. 따라서
        #   텔레옵이 끊기거나 Nav2 가 멈추면 CAN 송신이 그냥 '없어질' 뿐이고,
        #   STM32 의 PWM 레지스터는 마지막 값을 그대로 유지합니다 = 폭주.
        #
        # [해결 원리 — 송신을 '이벤트'에서 '주기'로 바꾼다]
        #   cmd_vel 도착과 무관하게 고정 주기로 CAN 을 보냅니다.
        #     - 명령이 신선하면: 마지막 바퀴 명령을 재송신 (하트비트)
        #     - 명령이 오래됐으면: 0 을 계속 송신 (능동적 정지 명령)
        #   이렇게 하면 STM32 는 항상 프레임을 받으므로 펌웨어 워치독이
        #   오탐으로 발동하지 않고, 동시에 정지 명령이 확실히 전달됩니다.
        #
        # [주기 선정]
        #   50 Hz(20ms) 송신 + STM32 500ms 타임아웃 = 연속 25 프레임 유실까지 허용.
        #   CAN 500 kbps 에서 8바이트 프레임 50개/초는 대역폭의 0.1% 수준입니다.
        #
        # [★ 한계 — 반드시 인지하십시오]
        #   이 타이머는 rclpy 실행자(executor) 스레드에서 돕니다. Pi 가 과부하이거나
        #   프로세스가 SIGKILL 되거나 전원이 끊기면 이 타이머도 같이 죽습니다.
        #   즉 이것은 '편의 계층'이고, 실제 안전을 보장하는 것은 STM32 의 워치독입니다.
        #   두 계층은 대체 관계가 아니라 직렬 관계입니다.
        self.declare_parameter('can_tx_hz', 50.0)

        # ── 워치독 / ZUPT ────────────────────────────────────────────
        self.declare_parameter('cmd_timeout',   0.30)   # [s] 이 시간 넘게 cmd_vel 없으면 정지 명령으로 간주
        self.declare_parameter('imu_timeout',   0.20)   # [s] IMU 신선도
        self.declare_parameter('zupt_enable',   True)
        self.declare_parameter('zupt_cmd_eps',  0.02)   # [m/s], [rad/s]
        self.declare_parameter('zupt_enc_eps',  0.03)   # [m/s]
        self.declare_parameter('zupt_gyro_eps', 0.05)   # [rad/s]
        self.declare_parameter('zupt_hold',     0.20)   # [s] 이 시간만큼 지속되어야 정지 확정

        # ── 공분산 모델 선택 및 이득 ─────────────────────────────────
        self.declare_parameter('covariance_model', 'linear')   # 'linear' | 'fuzzy'(Stage 3)
        self.declare_parameter('cov_sigma0_vx', 0.02)
        self.declare_parameter('cov_sigma0_vy', 0.10)
        self.declare_parameter('cov_k_v',       0.15)
        self.declare_parameter('cov_k_w',       1.00)
        self.declare_parameter('cov_k_c',       0.30)
        self.declare_parameter('cov_k_g',       1.20)
        self.declare_parameter('cov_k_a',       0.00)
        self.declare_parameter('cov_tau_attack',  0.05)
        self.declare_parameter('cov_tau_release', 0.80)
        self.declare_parameter('cov_var_max_v',   4.0)
        self.declare_parameter('cov_bias_tau',    10.0)   # [Stage 1b] 만성 요 편향 추정 시상수
        self.declare_parameter('publish_diagnostics', True)

        # ══ [Stage 1b] 요레이트 폐루프 보상기 ═════════════════════════
        #   ★ 기본 OFF. 넓은 공간에서 아래 순서로만 켜십시오.
        #     1) yaw_comp_ki:=0.0 으로 두고 kp 를 0.5 -> 1.0 -> 1.5 로 올리며 쏠림 감소 확인
        #     2) 진동이 시작되는 kp 의 절반을 채택
        #     3) ki 를 0.5씩 올려 잔류 쏠림(정상상태 오차)을 0으로
        #   진동이 나면 즉시 yaw_comp_enable:=false 로 되돌리고 limit 부터 줄이십시오.
        self.declare_parameter('yaw_comp_enable', False)
        self.declare_parameter('yaw_comp_kp',     1.0)
        self.declare_parameter('yaw_comp_ki',     1.5)
        self.declare_parameter('yaw_comp_limit',  0.35)   # [rad/s] 보상 권한 상한
        self.declare_parameter('yaw_comp_deadband', 0.01) # [rad/s] 자이로 노이즈 무시 대역

        gp = self.get_parameter
        channel          = gp('can_channel').value
        self._can_id     = gp('can_id').value
        self._max_speed  = gp('max_speed').value
        self._odom_frame = gp('odom_frame').value
        self._base_frame = gp('base_frame').value
        odom_topic       = gp('odom_topic').value
        imu_topic        = gp('imu_topic').value

        self._k_left    = float(gp('encoder_scale_left').value)
        self._k_right   = float(gp('encoder_scale_right').value)
        self._scale_y   = float(gp('encoder_scale_y').value)
        self._scale_yaw = float(gp('encoder_scale_yaw').value)
        self._use_gyro_yaw = bool(gp('use_gyro_for_yaw').value)
        self._slip_a    = float(gp('slip_offset').value)
        self._slip_b    = float(gp('slip_speed_coeff').value)

        if self._k_left <= 0.0 or self._k_right <= 0.0:
            self.get_logger().error('encoder_scale_left/right <= 0 은 불가. 1.0으로 강제합니다.')
            self._k_left = self._k_right = 1.0
        self._scale_x = 0.5 * (self._k_left + self._k_right)          # 파생: 평균
        self._balance = self._k_right / self._k_left                  # 파생: 밸런스

        self._cmd_timeout   = float(gp('cmd_timeout').value)
        self._imu_timeout   = float(gp('imu_timeout').value)
        self._zupt_enable   = bool(gp('zupt_enable').value)
        self._zupt_cmd_eps  = float(gp('zupt_cmd_eps').value)
        self._zupt_enc_eps  = float(gp('zupt_enc_eps').value)
        self._zupt_gyro_eps = float(gp('zupt_gyro_eps').value)
        self._zupt_hold     = float(gp('zupt_hold').value)
        self._pub_diag_on   = bool(gp('publish_diagnostics').value)

        # ── 공분산 모델 인스턴스화 (여기가 유일한 교체 지점) ──────────
        model_name = str(gp('covariance_model').value).lower()
        model_kwargs = dict(
            sigma0_vx=float(gp('cov_sigma0_vx').value),
            sigma0_vy=float(gp('cov_sigma0_vy').value),
            k_v=float(gp('cov_k_v').value),
            k_w=float(gp('cov_k_w').value),
            k_c=float(gp('cov_k_c').value),
            k_g=float(gp('cov_k_g').value),
            k_a=float(gp('cov_k_a').value),
            tau_attack=float(gp('cov_tau_attack').value),
            tau_release=float(gp('cov_tau_release').value),
            var_max_v=float(gp('cov_var_max_v').value),
            bias_tau=float(gp('cov_bias_tau').value),
        )
        if model_name == 'fuzzy':
            self._cov_model: CovarianceModel = FuzzyCovarianceModel(**model_kwargs)
        else:
            self._cov_model = LinearCovarianceModel(**model_kwargs)
        self.get_logger().info(f'Covariance model = {type(self._cov_model).__name__}')

        # ── [Stage 1b] 요레이트 보상기 ────────────────────────────────
        self._yaw_comp_on = bool(gp('yaw_comp_enable').value)
        self._yaw_comp = YawRateCompensator(
            kp=float(gp('yaw_comp_kp').value),
            ki=float(gp('yaw_comp_ki').value),
            limit=float(gp('yaw_comp_limit').value),
            deadband=float(gp('yaw_comp_deadband').value),
        )
        self._yaw_comp_last_t: float | None = None
        if self._yaw_comp_on:
            self.get_logger().warn(
                f'[요레이트 보상기 ON] kp={self._yaw_comp.kp} ki={self._yaw_comp.ki} '
                f'limit=±{self._yaw_comp.limit} rad/s — 넓은 공간에서 시험하십시오.')

        # ── 모드 / E-Stop ────────────────────────────────────────────
        self._mode        = 'MANUAL'
        self._mode_lock   = threading.Lock()
        self._is_estopped = False
        self._estop_lock  = threading.Lock()

        # ── 오도메트리 상태 ──────────────────────────────────────────
        self._odom_lock    = threading.Lock()
        self._pose_x = self._pose_y = self._pose_yaw = 0.0
        self._vel_x = self._vel_y = self._vel_yaw = 0.0
        self._last_fb_time: float | None = None
        # [버그 수정] dt가 너무 짧아 스킵된 프레임의 틱을 버리지 않고 이월 누적
        self._pend_left_ticks = 0
        self._pend_right_ticks = 0

        # ── 명령 상태 ────────────────────────────────────────────────
        self._cmd_lock = threading.Lock()
        self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
        self._cmd_stamp = 0.0
        # [Stage 1d] 하트비트가 재송신할 마지막 바퀴 명령 (CAN 정규화 단위)
        self._last_wheel_can = (0, 0, 0, 0)
        self._comm_lost = True          # 시작은 '두절' 상태 = 페일세이프
        # CAN 버스 접근 직렬화 (하트비트 타이머 / cmd_vel 콜백 / 종료 처리 공용)
        self._can_lock = threading.Lock()
        # [Stage 1e] CAN 건강 상태 카운터
        self._can_err_count = 0
        self._bad_tick_count = 0
        self._bus_off_seen = False
        self._can_err_by_class: dict = {}

        # ── IMU 상태 ─────────────────────────────────────────────────
        self._imu_lock  = threading.Lock()
        self._imu_wz    = 0.0
        self._imu_ax    = 0.0
        self._imu_stamp = 0.0

        # ── ZUPT 상태 ────────────────────────────────────────────────
        self._still_since: float | None = None

        # ── CAN 버스 ─────────────────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan',
                                          receive_own_messages=False)
            # ★ [Stage 1e] 커널 레벨 하드웨어 필터.
            #   유효 데이터 프레임 중 0x124 만 사용자 공간으로 올라옵니다.
            #   에러 프레임은 CAN_RAW_ERR_FILTER 가 따로 관리하므로 이 필터로는
            #   막히지 않습니다 -> 그래서 콜백의 is_error_frame 검사가 반드시 필요합니다.
            #   (필터를 걸어도 방어가 필요한 이유를 여기 남겨둡니다)
            self._bus.set_filters([{'can_id': _CAN_RX_FB_ID,
                                    'can_mask': 0x7FF,
                                    'extended': False}])
            self.get_logger().info(
                f'CAN 초기화 ({channel}) TX=0x{self._can_id:03X} RX=0x{_CAN_RX_FB_ID:03X} '
                f'(필터 적용, 에러 프레임 별도 차단)')
        except Exception as exc:
            self.get_logger().fatal(f'CAN 초기화 실패: {exc}')
            raise

        # ── ROS 구독/게시 ────────────────────────────────────────────
        self._sub_keyboard = self.create_subscription(Twist, '/cmd_vel_keyboard', self._keyboard_cb, _CMD_VEL_QOS)
        self._sub_nav2     = self.create_subscription(Twist, '/cmd_vel_nav2',     self._nav2_cb,     _CMD_VEL_QOS)
        self._sub_mode     = self.create_subscription(String, '/mode',  self._mode_cb,  10)
        self._sub_estop    = self.create_subscription(Bool,  '/e_stop', self._estop_cb, 10)
        self._sub_imu      = self.create_subscription(Imu, imu_topic, self._imu_cb, _IMU_QOS)

        self._pub_estop_ack = self.create_publisher(Bool, '/e_stop_ack', 10)
        self._pub_odom      = self.create_publisher(Odometry, odom_topic, 10)
        self._pub_diag      = self.create_publisher(Float32MultiArray, '~/slip_diagnostics', 10)
        # [Stage 1d] 통신 두절 상태를 외부(UI/Nav2 lifecycle)에서 볼 수 있게 게시
        self._pub_comm_lost = self.create_publisher(Bool, '~/comm_lost', 10)
        # [Stage 1e] CAN 버스 건강 상태 (배선 개선 효과를 정량 비교하는 지표)
        self._pub_can_health = self.create_publisher(Float32MultiArray, '~/can_health', 10)

        # ── [Stage 1d] ★ CAN 하트비트 타이머 ─────────────────────────
        can_tx_hz = float(gp('can_tx_hz').value)
        self._heartbeat_timer = self.create_timer(1.0 / can_tx_hz, self._heartbeat_cb)
        self.get_logger().info(
            f'CAN 하트비트 {can_tx_hz:.0f} Hz | cmd_vel 타임아웃 {self._cmd_timeout:.2f}s\n'
            f'  ※ 이 타이머는 편의 계층입니다. Pi 가 죽으면 함께 죽습니다.\n'
            f'    실제 폭주 방지는 STM32 의 CMD_TIMEOUT_MS(500ms) + IWDG 가 담당합니다.')

        self.get_logger().info(
            f'MotorNode(Mecanum) 준비 완료. Odom -> {odom_topic}, IMU <- {imu_topic}\n'
            f'  scale_left={self._k_left:.4f}  scale_right={self._k_right:.4f}  '
            f'(평균 {self._scale_x:.4f} / 밸런스 {self._balance:.4f})\n'
            f'  slip model: 1/(1 + {self._slip_a:.4f} + {self._slip_b:.4f}·v)'
            f'{"  [비활성]" if self._slip_b == 0.0 else ""}\n'
            f'  yaw source = {"IMU 자이로" if self._use_gyro_yaw else "엔코더(권장하지 않음)"}')
        if abs(self._scale_x - 1.0) < 1e-9:
            self.get_logger().warn(
                '[캘리브레이션] encoder_scale_left/right = 1.0 (무보정) 입니다. '
                'calibrate_scales.py 결과를 launch에 주입해야 편향이 제거됩니다.')
        if abs(self._balance - 1.0) > 1e-9:
            self.get_logger().warn(
                f'[주의] 좌우 밸런스={self._balance:.4f} (1.0 아님). '
                'RL 부상 등 기구 결함이 남아 있다면 이 값은 엔코더 눈금 보정이 아니라 '
                "'현재 결함 상태의 주행 모델 피팅'입니다. 요는 자이로에서 가져오십시오.")
        if self._slip_b != 0.0 and self._scale_x < 0.9:
            self.get_logger().warn(
                f'[이중 보정 의심] slip 모델이 켜져 있는데 scale 평균이 {self._scale_x:.3f}로 '
                '작습니다. scale_* 는 슬립 정규화 값(≈1.0 부근)이어야 합니다. '
                '가이드의 "정규화 값" 항목을 확인하십시오.')
        if not self._use_gyro_yaw:
            self.get_logger().warn(
                '[주의] use_gyro_for_yaw=false. 전륜 2채널 엔코더로는 메카넘 요레이트를 '
                '복원할 수 없습니다(랭크 결손). 차체 회전을 구조적으로 놓치게 됩니다.')

        # 🚀 모든 준비가 끝난 뒤에 CAN 수신 스레드 가동 (Race Condition 방어)
        self._notifier = can.Notifier(self._bus, [self._can_rx_callback])

    # ══════════════════════════════════════════════════════════════════
    # IMU
    # ══════════════════════════════════════════════════════════════════

    def _imu_cb(self, msg: Imu) -> None:
        with self._imu_lock:
            self._imu_wz = msg.angular_velocity.z
            self._imu_ax = msg.linear_acceleration.x
            self._imu_stamp = time.monotonic()

    # ══════════════════════════════════════════════════════════════════
    # CAN RX: STM32 엔코더 피드백 -> 하이브리드 오도메트리
    # ══════════════════════════════════════════════════════════════════

    def _can_rx_callback(self, msg: can.Message) -> None:
        try:
            # ══════════════════════════════════════════════════════════
            # ★★ [Stage 1e] 에러 프레임 배제 — 최우선 검사
            #
            # [버그의 정체]
            #   SocketCAN 은 버스 에러가 나면 '에러 프레임'을 수신 큐에 넣습니다.
            #   python-can 의 socketcan 백엔드는 CAN_RAW_ERR_FILTER 를 켜두므로
            #   이 콜백에 그대로 들어옵니다. 그런데 에러 프레임은
            #     - arbitration_id = 에러 클래스 비트마스크 (can_id & 0x7FF)
            #     - DLC = 항상 8, data[] = 에러 진단 바이트
            #   입니다. 즉 '정상 데이터 프레임처럼 생겼습니다'.
            #
            # [왜 하필 우리 ID 와 겹치는가 — 우연이 아닙니다]
            #   linux/can/error.h 의 에러 클래스 비트 조합:
            #     0x124 = CRTL(0x004) | ACK(0x020) | RESTARTED(0x100)
            #     0x123 = TX_TIMEOUT(0x001) | LOSTARB(0x002) | ACK(0x020) | RESTARTED(0x100)
            #   ACK 없음 + 컨트롤러 상태변화 + 버스오프 복귀 는 '통신 상대가 없을 때'
            #   전형적으로 함께 발생하는 조합입니다.
            #   => 하필 이 프로젝트의 피드백 ID(0x124)와 명령 ID(0x123)가 둘 다
            #      에러 클래스 조합으로 도달 가능한 값입니다.
            #
            # [그래서 무슨 일이 벌어졌나]
            #   STM32 가 꺼져 있으면 CAN 프레임에 ACK 를 줄 상대가 없습니다.
            #   -> 송신 실패 -> 재전송 -> 에러 카운터 상승 -> bus-off -> 자동 복구 -> 반복
            #   -> 에러 프레임 폭풍
            #   -> 그중 id 가 0x124 로 마스킹되는 것이 이 콜백을 통과
            #   -> struct.unpack('>hh', 진단바이트) = 유령 엔코더 틱
            #   -> /odom_motor 에 유령 속도 발행 (수 m/s ~ 수십 m/s 급)
            #   -> mpu6050_node._odom_cb 가 이를 보고 _is_moving = True
            #   -> IMU 각속도 공분산이 1e-5 -> 5e-2 로 점프
            #   즉 "키보드를 누르면 IMU 데이터가 튄다"의 완성된 경로입니다.
            #   물리 센서는 멀쩡했고, 소프트웨어가 만들어낸 유령이었습니다.
            #
            # [3중 방어]
            #   1) is_error_frame 검사 (아래)
            #   2) DLC 를 정확히 4 로 요구  (에러 프레임은 항상 8 이므로 이중 차단)
            #   3) 물리적으로 불가능한 틱 수 폐기 (아래 _MAX_TICKS_PER_FRAME)
            # ══════════════════════════════════════════════════════════
            if msg.is_error_frame:
                self._on_can_error(msg)
                return
            if getattr(msg, 'is_remote_frame', False):
                return
            if msg.arbitration_id != _CAN_RX_FB_ID:
                return
            # STM32 는 정확히 DLC=4 로 보냅니다. 그 외는 우리 프로토콜이 아닙니다.
            if msg.dlc != 4 or len(msg.data) != 4:
                self.get_logger().warn(
                    f'[CAN] 0x{msg.arbitration_id:03X} DLC={msg.dlc} 예상(4)과 불일치 — 폐기',
                    throttle_duration_sec=2.0)
                return

            left_ticks, right_ticks = struct.unpack('>hh', msg.data[:4])

            # 방어 3: 물리적으로 불가능한 틱 수 폐기
            #   최대 150 RPM = 2.5 rev/s = 3510 ticks/s. 20ms 프레임이면 70틱.
            #   여유 2배(140)를 넘으면 노이즈이거나 프레임 유실로 인한 누적입니다.
            if abs(left_ticks) > _MAX_TICKS_PER_FRAME or abs(right_ticks) > _MAX_TICKS_PER_FRAME:
                self._bad_tick_count += 1
                self.get_logger().warn(
                    f'[CAN] 비정상 틱 폐기 L={left_ticks} R={right_ticks} '
                    f'(한계 ±{_MAX_TICKS_PER_FRAME}, 누적 {self._bad_tick_count})',
                    throttle_duration_sec=2.0)
                return
            right_ticks = -right_ticks
            now = time.monotonic()

            with self._odom_lock:
                # [버그 수정] 예전 코드는 dt<1ms 프레임의 틱을 통째로 버려
                #            거리가 과소 적분되었습니다. 이제 다음 프레임으로 이월합니다.
                self._pend_left_ticks += left_ticks
                self._pend_right_ticks += right_ticks

                if self._last_fb_time is None:
                    self._last_fb_time = now
                    self._pend_left_ticks = self._pend_right_ticks = 0
                    return

                dt = now - self._last_fb_time
                if dt < 0.001:
                    return                      # 틱은 이월된 상태로 유지 -> 손실 없음
                self._last_fb_time = now
                lt, rt = self._pend_left_ticks, self._pend_right_ticks
                self._pend_left_ticks = self._pend_right_ticks = 0

            # ── 원시 기하 계산 : ★ 스케일을 '바퀴 단위'로 먼저 적용 ──────
            #    오차가 발생하는 지점(바퀴 접지면)에서 보정하는 것이 구조적으로 옳습니다.
            #    평균에 한 번 곱하는 방식은 좌우 비대칭을 표현할 수 없습니다.
            raw_left  = lt * _METER_PER_TICK
            raw_right = rt * _METER_PER_TICK

            # ★ [Stage 1c] 속도 의존 슬립 보정
            #   v_raw = 스케일 적용 '전' 바퀴 표면속도. 스케일된 값을 쓰면 순환 참조가 됩니다.
            #   slip(v) = a + b·v  ->  보정 계수 = 1/(1 + slip)
            v_raw = abs(raw_left + raw_right) * 0.5 / dt
            slip_div = 1.0 + self._slip_a + self._slip_b * v_raw
            if slip_div < 0.2:            # 이상값 방어 (음수/과소 분모)
                slip_div = 0.2

            dist_left  = raw_left  * self._k_left  / slip_div
            dist_right = raw_right * self._k_right / slip_div

            delta_x_robot = (dist_left + dist_right) * 0.5
            # 엔코더가 '주장하는' 요 변화. 아래에서 Vy 오염을 제거합니다.
            delta_yaw_enc = (dist_right - dist_left) / _YAW_LEVER * self._scale_yaw

            # ── Vy: 실측 불가(좌/우 2채널만 수신) -> 명령 적분 (개루프) ──
            #    ※ 근본 해결책은 STM32가 4륜 틱을 모두 보내주는 것입니다.
            #       그러면 vy = (w_fl - w_fr - w_rl + w_rr)*r/4 로 실측 가능합니다.
            with self._cmd_lock:
                cmd_fresh = (now - self._cmd_stamp) < self._cmd_timeout
                cmd_vx = self._cmd_vx if cmd_fresh else 0.0
                cmd_vy = self._cmd_vy if cmd_fresh else 0.0
                cmd_wz = self._cmd_wz if cmd_fresh else 0.0
            delta_y_robot = cmd_vy * dt * self._scale_y

            with self._imu_lock:
                imu_valid = (now - self._imu_stamp) < self._imu_timeout
                imu_wz = self._imu_wz if imu_valid else 0.0
                imu_ax = self._imu_ax if imu_valid else 0.0

            # ── ★ [Stage 1c] 엔코더 요의 Vy 오염 제거 ─────────────────
            #   엔코더가 FL/FR(전륜)에만 있으므로 메카넘 IK 상
            #       (FR − FL)/(2l) = Wz + Vy/l
            #   즉 게걸음(Vy)이 그대로 가짜 요레이트로 새어 들어옵니다 (계수 1/l = 1.98).
            #   Vy 를 실측할 수단이 없으므로 명령값으로 보정합니다. 완벽하진 않지만,
            #   보정을 안 하면 게걸음 0.2 m/s 마다 0.396 rad/s 의 가짜 요가 r_gyro 로 들어가
            #   슬립이 없는데도 var_vx 가 폭증합니다(오탐).
            delta_yaw_enc -= (cmd_vy * dt) / _MECANUM_L

            enc_vx = delta_x_robot / dt
            enc_vy = delta_y_robot / dt
            enc_wz = delta_yaw_enc / dt        # Vy 보정 후 엔코더 요레이트 (진단/잔차 전용)

            # ── ★ [Stage 1b] 요 소스 선택 ────────────────────────────
            #    전륜 2채널로는 메카넘 요레이트를 복원할 수 없습니다(랭크 결손).
            #    자이로가 살아 있으면 무조건 자이로를 씁니다.
            if self._use_gyro_yaw and imu_valid:
                body_wz = imu_wz
                delta_yaw = imu_wz * dt
            else:
                body_wz = enc_wz
                delta_yaw = delta_yaw_enc

            # ── ZUPT 판정 ────────────────────────────────────────────
            stationary = self._update_zupt(now, cmd_vx, cmd_vy, cmd_wz,
                                           enc_vx, enc_wz, imu_wz, imu_valid)
            if stationary:
                # 정지 확정 시 적분 자체를 중단 -> 엔코더 노이즈 드리프트 제거
                delta_x_robot = delta_y_robot = delta_yaw = 0.0
                enc_vx = enc_vy = enc_wz = body_wz = 0.0

            # ── 포즈 적분 (중점 Yaw 근사: 2차 정확도) ────────────────
            with self._odom_lock:
                mid_yaw = self._pose_yaw + delta_yaw * 0.5
                self._pose_x += (delta_x_robot * math.cos(mid_yaw)
                                 - delta_y_robot * math.sin(mid_yaw))
                self._pose_y += (delta_x_robot * math.sin(mid_yaw)
                                 + delta_y_robot * math.cos(mid_yaw))
                self._pose_yaw = _normalize_angle(self._pose_yaw + delta_yaw)

                self._vel_x, self._vel_y, self._vel_yaw = enc_vx, enc_vy, body_wz
                x, y, yaw = self._pose_x, self._pose_y, self._pose_yaw

            # ── 공분산 모델 평가 (교체 가능한 유일한 지점) ───────────
            evidence = MotionEvidence(
                dt=dt,
                cmd_vx=cmd_vx, cmd_vy=cmd_vy, cmd_wz=cmd_wz,
                enc_vx=enc_vx, enc_vy=enc_vy, enc_wz=enc_wz,
                imu_wz=imu_wz, imu_ax=imu_ax, imu_valid=imu_valid,
                cmd_fresh=cmd_fresh, stationary=stationary,
            )
            cov = self._cov_model.evaluate(evidence)

            stamp = self.get_clock().now().to_msg()
            self._publish_odom(x, y, yaw, enc_vx, enc_vy, body_wz, cov, stamp)
            if self._pub_diag_on:
                self._publish_diagnostics(cov, evidence)

        except Exception as e:
            self.get_logger().error(f'[🚨 긴급] CAN RX 스레드 예외! 원인: {repr(e)}')

    # ------------------------------------------------------------------ #
    def _update_zupt(self, now, cmd_vx, cmd_vy, cmd_wz,
                     enc_vx, enc_wz, imu_wz, imu_valid) -> bool:
        """
        ZUPT(Zero-velocity UPdaTe) 판정.
        명령이 0이고, 엔코더도 0이고, 자이로도 0이어야 '정지'로 인정합니다.
        (셋 중 하나라도 움직이면 정지가 아님 -> 오탐 방지)
        일정 시간(zupt_hold) 유지되어야 확정하여 감속 구간의 채터링을 막습니다.
        """
        if not self._zupt_enable:
            return False
        quiet = (abs(cmd_vx) < self._zupt_cmd_eps
                 and abs(cmd_vy) < self._zupt_cmd_eps
                 and abs(cmd_wz) < self._zupt_cmd_eps
                 and abs(enc_vx) < self._zupt_enc_eps
                 and abs(enc_wz) < self._zupt_gyro_eps
                 and (not imu_valid or abs(imu_wz) < self._zupt_gyro_eps))
        if not quiet:
            self._still_since = None
            return False
        if self._still_since is None:
            self._still_since = now
            return False
        return (now - self._still_since) >= self._zupt_hold

    # ══════════════════════════════════════════════════════════════════
    # 오도메트리 게시
    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        h = yaw * 0.5
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def _publish_odom(self, x, y, yaw, vx, vy, wz,
                      cov: CovarianceOutput, stamp) -> None:
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz

        # ── pose 공분산 (진단용 / pose 융합 재활성화 대비) ──────────────
        #    인덱스: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw (6x6 row-major)
        #    현 ekf.yaml에서 odom1의 pose는 융합되지 않으므로 EKF는 이 값을 읽지 않습니다.
        #    그래도 채워두는 이유: Foxglove에서 추측항법 불확실성 성장을 눈으로 보기 위함.
        cp = [0.0] * 36
        cp[0]  = cov.var_x
        cp[7]  = cov.var_y
        cp[35] = cov.var_yaw
        cp[14] = cp[21] = cp[28] = 1e6      # 2D 주행: 미사용 축은 무한대에 가깝게
        odom.pose.covariance = cp

        # ── ★ twist 공분산 : EKF(odom1_config)가 실제로 읽는 R 행렬 ────
        #    인덱스: 0=vx, 7=vy, 14=vz, 21=vroll, 28=vpitch, 35=vyaw
        ct = [0.0] * 36
        ct[0]  = cov.var_vx
        ct[7]  = cov.var_vy
        ct[35] = cov.var_wz
        ct[14] = ct[21] = ct[28] = 1e6
        odom.twist.covariance = ct

        self._pub_odom.publish(odom)

    def _publish_diagnostics(self, cov: CovarianceOutput, ev: MotionEvidence) -> None:
        m = Float32MultiArray()
        m.data = [
            float(cov.slip_index),
            float(math.sqrt(cov.var_vx)),
            float(math.sqrt(cov.var_vy)),
            float(math.sqrt(cov.var_wz)),
            float(cov.debug.get('r_gyro', 0.0)),
            float(cov.debug.get('r_cmd', 0.0)),
            float(cov.debug.get('r_acc', 0.0)),
            float(ev.enc_wz),
            float(ev.imu_wz),
            float(cov.debug.get('omega', 0.0)),
            1.0 if ev.stationary else 0.0,
            float(cov.debug.get('yaw_bias', 0.0)),
            float(cov.debug.get('raw_gyro_res', 0.0)),
            float(self._yaw_comp.output),
        ]
        self._pub_diag.publish(m)

    # ══════════════════════════════════════════════════════════════════
    # 메카넘 IK: cmd_vel -> 4바퀴 독립 CAN 명령
    # ══════════════════════════════════════════════════════════════════

    def _apply_twist(self, msg: Twist) -> None:
        # ── ★ [Stage 1b] 요레이트 폐루프 보상 ─────────────────────────
        #    RL 부상으로 생기는 원치 않는 요 모멘트를, IMU가 본 실제 회전을 근거로 상쇄합니다.
        #    보상량은 IK 로 들어가는 wz 에만 더하고, 아래 self._cmd_wz(공분산 모델용 '의도')
        #    에는 원본 명령을 그대로 저장합니다. 증거와 액추에이션을 섞으면 안 됩니다.
        now = time.monotonic()
        wz_eff = msg.angular.z
        comp = 0.0
        if self._yaw_comp_on:
            dt_c = 0.02 if self._yaw_comp_last_t is None else \
                min(max(now - self._yaw_comp_last_t, 1e-3), 0.2)
            self._yaw_comp_last_t = now
            with self._imu_lock:
                imu_ok = (now - self._imu_stamp) < self._imu_timeout
                imu_wz_now = self._imu_wz
            moving = (abs(msg.linear.x) > 0.02 or abs(msg.linear.y) > 0.02
                      or abs(msg.angular.z) > 0.02)
            comp = self._yaw_comp.update(msg.angular.z, imu_wz_now, dt_c,
                                         active=(imu_ok and moving))
            wz_eff = msg.angular.z + comp

        v_fl_ms = msg.linear.x - msg.linear.y - (wz_eff * _MECANUM_L)
        v_fr_ms = msg.linear.x + msg.linear.y + (wz_eff * _MECANUM_L)
        v_rl_ms = msg.linear.x + msg.linear.y - (wz_eff * _MECANUM_L)
        v_rr_ms = msg.linear.x - msg.linear.y + (wz_eff * _MECANUM_L)

        v_fl, v_fr = v_fl_ms / _MAX_VX, v_fr_ms / _MAX_VX
        v_rl, v_rr = v_rl_ms / _MAX_VX, v_rr_ms / _MAX_VX

        max_val = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr), 1.0)
        v_fl, v_fr, v_rl, v_rr = v_fl / max_val, v_fr / max_val, v_rl / max_val, v_rr / max_val

        N = self._max_speed
        fl_can = int(max(-N, min(N, v_fl * N)))
        fr_can = int(max(-N, min(N, v_fr * N)))
        rl_can = int(max(-N, min(N, v_rl * N)))
        rr_can = int(max(-N, min(N, v_rr * N)))

        with self._cmd_lock:
            # ★ 원본 명령을 저장 (보상량 comp 를 섞지 않음).
            #   공분산 모델은 '운전자가 무엇을 의도했는가'를 알아야 슬립을 판정할 수 있습니다.
            self._cmd_vx = msg.linear.x
            self._cmd_vy = msg.linear.y
            self._cmd_wz = msg.angular.z
            self._cmd_stamp = now                   # ★ 워치독 타임스탬프
            # [Stage 1d] 하트비트가 재송신할 값 보관
            self._last_wheel_can = (fl_can, fr_can, rl_can, rr_can)

        self._send_can(fl_can, fr_can, rl_can, rr_can)

    def _zero_cmd(self) -> None:
        with self._cmd_lock:
            self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
            self._cmd_stamp = time.monotonic()
            # [Stage 1d] 하트비트가 옛 명령을 되살리지 않도록 반드시 함께 비웁니다.
            # 이걸 빠뜨리면 E-Stop 직후에도 타이머가 마지막 속도를 재송신합니다.
            self._last_wheel_can = (0, 0, 0, 0)
        # 보상기 적분항을 반드시 비웁니다. 남겨두면 재출발 순간 킥이 발생합니다.
        self._yaw_comp.reset()
        self._yaw_comp_last_t = None

    # ══════════════════════════════════════════════════════════════════
    # cmd_vel Mux / E-Stop
    # ══════════════════════════════════════════════════════════════════

    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old, self._mode = self._mode, msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')
            self._send_can(0, 0, 0, 0)
            self._zero_cmd()

    def _estop_cb(self, msg: Bool) -> None:
        with self._estop_lock:
            prev, self._is_estopped = self._is_estopped, msg.data
        if msg.data and not prev:
            self.get_logger().warn('[E-STOP] 발동!')
            self._send_can(0, 0, 0, 0)
            self._zero_cmd()
            ack = Bool(); ack.data = True
            self._pub_estop_ack.publish(ack)
        elif not msg.data and prev:
            self.get_logger().info('[E-STOP] 해제')
            ack = Bool(); ack.data = False
            self._pub_estop_ack.publish(ack)

    def _keyboard_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'MANUAL':
                return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0); self._zero_cmd(); return
        self._apply_twist(msg)

    def _nav2_cb(self, msg: Twist) -> None:
        with self._mode_lock:
            if self._mode != 'AUTO':
                return
        with self._estop_lock:
            if self._is_estopped:
                self._send_can(0, 0, 0, 0); self._zero_cmd(); return
        self._apply_twist(msg)

    # ══════════════════════════════════════════════════════════════════
    # CAN TX
    # ══════════════════════════════════════════════════════════════════

    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        try:
            # 하트비트 타이머와 cmd_vel 콜백이 동시에 들어올 수 있으므로 직렬화
            with self._can_lock:
                self._bus.send(can.Message(arbitration_id=self._can_id,
                                           data=data, is_extended_id=False),
                               timeout=0.05)   # ★ 블로킹 상한. 버스 이상 시 노드가 멎지 않게
        except can.CanError as exc:
            # 여기서 예외를 삼키는 것이 맞습니다. 송신 실패는 곧 '명령 부재'이고,
            # STM32 워치독이 500ms 뒤 알아서 모터를 세웁니다.
            self.get_logger().error(f'[CAN TX ERROR] {exc}', throttle_duration_sec=1.0)

    # ══════════════════════════════════════════════════════════════════
    # [Stage 1d] CAN 하트비트 / 통신 두절 안전장치
    # ══════════════════════════════════════════════════════════════════

    def _heartbeat_cb(self) -> None:
        """
        고정 주기로 CAN 을 송신한다. cmd_vel 이 오든 안 오든 무조건 돈다.

        판정 우선순위:
          1) E-Stop      -> 0 송신 (최우선)
          2) 명령 노후화 -> 0 송신 + 두절 플래그
          3) 정상        -> 마지막 바퀴 명령 재송신 (하트비트)

        ★ 주의: 여기서 self._cmd_stamp 를 갱신하면 안 됩니다.
          그러면 자기 자신이 보낸 하트비트를 '신선한 명령'으로 착각해
          cmd_vel 이 끊겨도 영원히 마지막 속도를 재송신하게 됩니다. 폭주 재현입니다.
        """
        now = time.monotonic()

        with self._estop_lock:
            estopped = self._is_estopped
        with self._cmd_lock:
            age = now - self._cmd_stamp
            wheels = self._last_wheel_can

        stale = age > self._cmd_timeout

        if estopped or stale:
            if stale and not self._comm_lost:
                self._comm_lost = True
                self.get_logger().warn(
                    f'[통신 두절] cmd_vel 이 {age:.2f}s 동안 없습니다. '
                    f'0 속도를 지속 송신합니다.')
                with self._cmd_lock:
                    self._last_wheel_can = (0, 0, 0, 0)
            self._send_can(0, 0, 0, 0)
            self._publish_comm_lost(True)
            return

        if self._comm_lost:
            self._comm_lost = False
            self.get_logger().info('[통신 복구] cmd_vel 수신 재개.')
        self._send_can(*wheels)
        self._publish_comm_lost(False)
        self._publish_can_health()

    def _publish_comm_lost(self, lost: bool) -> None:
        m = Bool()
        m.data = lost
        self._pub_comm_lost.publish(m)

    # ══════════════════════════════════════════════════════════════════
    # [Stage 1e] CAN 버스 건강 상태
    # ══════════════════════════════════════════════════════════════════

    def _on_can_error(self, msg: can.Message) -> None:
        """
        에러 프레임 처리 — 버리기만 하지 말고 '진단 데이터'로 남긴다.

        에러 프레임이 쏟아진다는 것은 대부분 아래 셋 중 하나입니다.
          1) 통신 상대(STM32)가 꺼져 있음        -> ACK 없음
          2) 종단 저항(120Ω) 문제               -> 반사/BUSERROR
          3) ★ 두 노드 사이에 GND 기준이 없음    -> 커먼모드 이탈, 간헐적 ACK 실패
        특히 3번은 "가끔 되다가 모터 돌면 안 됨" 형태로 나타나 원인 추적이 어렵습니다.
        누적 카운터를 남겨두면 배선 개선의 효과를 정량 비교할 수 있습니다.
        """
        self._can_err_count += 1
        cls = msg.arbitration_id & 0x7FF
        names = [n for b, n in _CAN_ERR_CLASS.items() if cls & b]
        for n in names:
            self._can_err_by_class[n] = self._can_err_by_class.get(n, 0) + 1

        if 'BUSOFF' in names and not self._bus_off_seen:
            self._bus_off_seen = True
            self.get_logger().error(
                '[CAN] BUS-OFF 발생! 통신 상대 부재, 종단 저항 미설치, 또는 '
                'GND 기준 부재를 의심하십시오.')

        self.get_logger().warn(
            f'[CAN] 에러 프레임 (누적 {self._can_err_count}) '
            f'class=0x{cls:03X} [{"|".join(names) if names else "?"}] '
            f'— 엔코더 데이터로 오인하지 않고 폐기했습니다.',
            throttle_duration_sec=2.0)

    def _publish_can_health(self) -> None:
        """0:err_total 1:bad_tick 2:bus_off 3:ACK오류 4:BUSERROR 5:RESTARTED"""
        m = Float32MultiArray()
        m.data = [
            float(self._can_err_count),
            float(self._bad_tick_count),
            1.0 if self._bus_off_seen else 0.0,
            float(self._can_err_by_class.get('ACK', 0)),
            float(self._can_err_by_class.get('BUSERROR', 0)),
            float(self._can_err_by_class.get('RESTARTED', 0)),
        ]
        self._pub_can_health.publish(m)

    def destroy_node(self):
        # [Stage 1d] 종료 시 정지 명령을 '여러 번' 보냅니다.
        #   단발 송신은 CAN 버스 순간 오류/중재 실패로 유실될 수 있고,
        #   그러면 종료했는데 로봇이 계속 달리는 최악의 상황이 됩니다.
        #   ※ 이것도 정상 종료 경로에서만 동작합니다. SIGKILL/전원 차단에서는
        #     STM32 워치독만이 유일한 방어선입니다.
        try:
            self._heartbeat_timer.cancel()
        except Exception:
            pass
        try:
            self._notifier.stop()
        except Exception:
            pass
        for _ in range(5):
            try:
                self._send_can(0, 0, 0, 0)
                time.sleep(0.01)
            except Exception:
                break
        try:
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


if __name__ == '__main__':
    main()