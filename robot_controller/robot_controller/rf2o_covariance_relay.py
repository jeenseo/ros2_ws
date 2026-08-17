#!/usr/bin/env python3
"""
rf2o_covariance_relay.py
========================
/odom_rf2o  ->  /odom_rf2o_cov  릴레이 노드

[왜 이 노드가 필요한가]
robot_localization 에는 `odom0_covariance` 같은 R 행렬 오버라이드 파라미터가 없습니다.
EKF는 들어온 메시지의 covariance 필드를 그대로 R로 씁니다. 그런데 rf2o_laser_odometry는
covariance를 0 또는 고정 상수로 채워 보냅니다.
=> 즉, 이 릴레이 없이는 "라이다 X 신뢰도를 낮추고 Y/Yaw 신뢰도를 높인다"는 튜닝을
   ekf.yaml 안에서 표현할 방법이 물리적으로 존재하지 않습니다.

[이 노드가 하는 일]
1. 연속된 두 rf2o pose로부터 body-frame 속도(Vx, Vy, Wz)를 직접 계산합니다.
   - rf2o의 twist 필드를 신뢰하지 않고 직접 계산하는 이유: 버전에 따라 비어 있거나
     world-frame으로 채워져 있는 경우가 있어 재현성이 떨어집니다.
   - rf2o는 본질적으로 '스캔 정합으로 변위를 추정하는' 속도 센서이고, 그 pose는 적분값
     (드리프트 누적)입니다. 따라서 EKF에는 속도로 넣는 것이 이론적으로 옳습니다.
2. 파라미터로 지정한 이방성(anisotropic) 공분산을 주입합니다.
   복도 퇴화 대응: sigma_vx 크게, sigma_vy / sigma_wz 작게.
3. [Stage 2 훅] 퇴화 감지 게인 자리를 미리 뚫어 두었습니다 (기본 비활성).

[실행]
  ros2 run <your_pkg> rf2o_covariance_relay.py --ros-args \
      -p sigma_vx:=0.245 -p sigma_vy:=0.05 -p sigma_wz:=0.10
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


_ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _yaw_from_quat(q) -> float:
    """2D 주행 가정 하의 Yaw 추출 (roll/pitch 무시)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Rf2oCovarianceRelay(Node):

    def __init__(self):
        super().__init__('rf2o_covariance_relay')

        self.declare_parameter('input_topic',  '/odom_rf2o')
        self.declare_parameter('output_topic', '/odom_rf2o_cov')
        self.declare_parameter('child_frame_id', 'base_footprint')  # EKF base_link_frame과 일치시킬 것

        # ── R 행렬 (표준편차 단위로 입력받아 제곱해서 사용) ─────────────
        #    ▣ sigma_vx 근거
        #      복도 5m 주행에서 rf2o가 2.5m만 인식 = 전진 속도를 평균 50% 과소 추정.
        #      순항 0.5 m/s 기준 오차 약 0.25 m/s -> sigma_vx = 0.245 (var ≈ 0.06)
        #    ▣ sigma_vy 근거
        #      복도에서 좌우 벽면은 항상 관측되므로 횡방향 정합은 매우 잘 구속됩니다.
        #      -> sigma_vy = 0.05 (var = 0.0025). X 대비 약 24배 신뢰.
        #    ▣ sigma_wz 근거  ★ [Stage 1b] 0.10 -> 0.06 하향
        #      전륜 2채널 엔코더로는 메카넘 요를 복원할 수 없어(랭크 결손) 자이로가 유일한
        #      고주파 요 소스가 되었습니다. 그러면 자이로 바이어스를 잡아줄 주체가 필요한데,
        #      rf2o Vyaw 는 '정지한 벽면' 기준 스캔정합이라 무편향(bias-free)입니다.
        #      정상상태 가중치 w_imu = R_rf2o/(R_imu + R_rf2o):
        #        0.10 -> var 0.01,   R_imu 4e-4 -> w_imu = 0.962 (라이다 권한 3.8%)
        #        0.06 -> var 0.0036, R_imu 4e-4 -> w_imu = 0.900 (라이다 권한 10%)
        #      단기 회전은 IMU 가 90% 지배하되, 장기 방위 드리프트는 라이다가 끌어당깁니다.
        self.declare_parameter('sigma_vx', 0.245)   # [m/s]   ← 크게 (복도 퇴화)
        self.declare_parameter('sigma_vy', 0.05)    # [m/s]   ← 작게 (측면 잘 구속됨)
        self.declare_parameter('sigma_wz', 0.06)    # [rad/s] ← 작게 (벽면 방위 구속 + 무편향)

        self.declare_parameter('max_dt', 0.5)       # [s] 이보다 큰 간격은 속도 계산 폐기
        self.declare_parameter('max_speed', 3.0)    # [m/s] 스캔매칭 발산 시 아웃라이어 차단
        self.declare_parameter('max_omega', 6.0)    # [rad/s]

        # ── [Stage 2] 퇴화 감지 : 기본 비활성 ─────────────────────────
        #    베이스라인(Stage 1) 측정을 오염시키지 않도록 반드시 false로 시작하십시오.
        self.declare_parameter('enable_degeneracy_boost', False)
        self.declare_parameter('degeneracy_stall_eps', 0.005)  # [m/s] 이보다 작으면 '정지'로 간주
        self.declare_parameter('degeneracy_stall_count', 3)    # 연속 프레임 수
        self.declare_parameter('degeneracy_gain', 25.0)        # 퇴화 시 var_vx 배율

        gp = self.get_parameter
        in_topic  = gp('input_topic').value
        out_topic = gp('output_topic').value
        self._child_frame = gp('child_frame_id').value

        self._var_vx = float(gp('sigma_vx').value) ** 2
        self._var_vy = float(gp('sigma_vy').value) ** 2
        self._var_wz = float(gp('sigma_wz').value) ** 2

        self._max_dt    = float(gp('max_dt').value)
        self._max_speed = float(gp('max_speed').value)
        self._max_omega = float(gp('max_omega').value)

        self._deg_on    = bool(gp('enable_degeneracy_boost').value)
        self._deg_eps   = float(gp('degeneracy_stall_eps').value)
        self._deg_count = int(gp('degeneracy_stall_count').value)
        self._deg_gain  = float(gp('degeneracy_gain').value)
        self._stall_run = 0

        self._prev = None    # (t, x, y, yaw)

        self._sub = self.create_subscription(Odometry, in_topic, self._cb, _ODOM_QOS)
        self._pub = self.create_publisher(Odometry, out_topic, _ODOM_QOS)
        self._pub_diag = self.create_publisher(Float32MultiArray, '~/lidar_diagnostics', 10)

        self.get_logger().info(
            f'rf2o 공분산 릴레이: {in_topic} -> {out_topic} | '
            f'sigma(vx,vy,wz) = ({math.sqrt(self._var_vx):.3f}, '
            f'{math.sqrt(self._var_vy):.3f}, {math.sqrt(self._var_wz):.3f}) | '
            f'퇴화감지 = {"ON(Stage2)" if self._deg_on else "OFF(Stage1 베이스라인)"}')

    # ------------------------------------------------------------------ #
    def _cb(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)

        if self._prev is None:
            self._prev = (t, x, y, yaw)
            return

        t0, x0, y0, yaw0 = self._prev
        dt = t - t0
        if dt <= 1e-4 or dt > self._max_dt:
            self._prev = (t, x, y, yaw)
            return
        self._prev = (t, x, y, yaw)

        # ── world-frame 변위 -> body-frame 속도 ──────────────────────
        #    중점 Yaw(mid-point rule)로 회전 변환하여 2차 정확도를 확보합니다.
        #    (전방 오일러 근사보다 회전 중 오차가 작음)
        dx, dy = x - x0, y - y0
        dyaw = _norm_angle(yaw - yaw0)
        mid = yaw0 + 0.5 * dyaw
        c, s = math.cos(mid), math.sin(mid)

        vx = ( dx * c + dy * s) / dt      # body 전진
        vy = (-dx * s + dy * c) / dt      # body 횡이동
        wz = dyaw / dt

        # ── 아웃라이어 차단 (스캔매칭 발산 프레임 폐기) ─────────────
        if (abs(vx) > self._max_speed or abs(vy) > self._max_speed
                or abs(wz) > self._max_omega):
            self.get_logger().warn(
                f'[rf2o] 아웃라이어 폐기: v=({vx:.2f},{vy:.2f}) w={wz:.2f}')
            return

        # ── [Stage 2 훅] 퇴화 게인 ──────────────────────────────────
        gain = self._degeneracy_gain(vx, wz)

        out = Odometry()
        out.header.stamp = msg.header.stamp          # 원본 타임스탬프 보존 (EKF 시간 정합 필수)
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = self._child_frame

        # pose는 EKF에서 사용하지 않지만(odom0_config pose 전부 false),
        # RViz/Foxglove 확인용으로 그대로 전달하고 공분산만 크게 표기합니다.
        out.pose = msg.pose
        pc = [0.0] * 36
        pc[0] = pc[7] = 1e3
        pc[14] = pc[21] = pc[28] = 1e6
        pc[35] = 1e3
        out.pose.covariance = pc

        out.twist.twist.linear.x  = vx
        out.twist.twist.linear.y  = vy
        out.twist.twist.angular.z = wz

        # ── ★ EKF가 읽는 R 행렬 ─────────────────────────────────────
        #    인덱스: 0=vx, 7=vy, 14=vz, 21=vroll, 28=vpitch, 35=vyaw
        tc = [0.0] * 36
        tc[0]  = self._var_vx * gain      # X: 크게 = 복도에서 전진 데이터를 거의 무시
        tc[7]  = self._var_vy             # Y: 작게 = 횡방향은 라이다를 신뢰
        tc[35] = self._var_wz             # Yaw: 작게 (단, IMU가 더 정확해 EKF가 알아서 배분)
        tc[14] = tc[21] = tc[28] = 1e6
        out.twist.covariance = tc

        self._pub.publish(out)

        d = Float32MultiArray()
        # 0:vx 1:vy 2:wz 3:sigma_vx(적용값) 4:degeneracy_gain 5:stall_run
        d.data = [float(vx), float(vy), float(wz),
                  float(math.sqrt(tc[0])), float(gain), float(self._stall_run)]
        self._pub_diag.publish(d)

    # ------------------------------------------------------------------ #
    def _degeneracy_gain(self, vx: float, wz: float) -> float:
        """
        [Stage 2 자리] LiDAR 퇴화 감지 -> X축 공분산 순간 팽창.

        현재 구현은 가장 단순한 '스톨(stall) 감지'입니다:
          특징점 없는 복도에서 rf2o는 전진 변위를 찾지 못해 vx ≈ 0 을 연속 출력합니다.
          (실측에서 LiDAR가 2.5m 지점에서 '멈춘' 그래프가 바로 이 현상입니다.)
          회전 중에는 벽면 특징이 잡혀 정상 동작하므로 |wz|가 크면 판정에서 제외합니다.

        Stage 2 정식 구현 시에는 아래 중 하나로 교체하는 것을 권장합니다:
          (a) 스캔 법선 벡터의 분산 분석: 법선이 한 방향으로 몰리면 그 직교축이 퇴화
          (b) 스캔매칭 헤시안 H = J^T J 의 고유값 분해 후 조건수(condition number) 감시
              -> lambda_min / lambda_max 가 임계값 이하이면 해당 고유벡터 방향이 퇴화
          (b)가 이론적으로 정확하며, rf2o 소스의 헤시안을 그대로 퍼블리시하면 됩니다.
        """
        if not self._deg_on:
            self._stall_run = 0
            return 1.0

        if abs(vx) < self._deg_eps and abs(wz) < 0.15:
            self._stall_run += 1
        else:
            self._stall_run = 0

        if self._stall_run >= self._deg_count:
            return self._deg_gain
        return 1.0


def main(args=None):
    rclpy.init(args=args)
    node = Rf2oCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()