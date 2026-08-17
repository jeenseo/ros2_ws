#!/usr/bin/env python3
"""
calibrate_scales.py
===================
1회 직진 주행 실측값 -> encoder_scale_left / encoder_scale_right 산출기

[입력 4개]
  X_true    : 도착점 X 실측 [m]  (줄자)
  Y_true    : 도착점 Y 실측 [m]  (+ = 좌측)
  x_odom    : /odom_motor 의 최종 pose.position.x [m]
  yaw_odom  : /odom_motor 의 최종 yaw [rad]  (쿼터니언 z,w 에서 변환)

[원리]
로봇이 등곡률 원호를 그렸다고 보면, 원점에서 헤딩 +X 로 출발한 원호의 끝점은
    x = R·sinθ ,  y = R·(1 − cosθ)     ->   tan(θ/2) = y/x
따라서 총 회전각 θ 와 반지름 R, 실제 경로 길이(호) s = R·θ 가 모두 닫힌 형태로 나온다.
줄자로 잰 것은 '현(chord)'이고 엔코더가 잰 것은 '호(arc)'이므로 이 변환이 필수다.

메카넘 전륜 두 바퀴의 이상적(무슬립) 롤링 거리는 IK 로부터
    d_FL = s − Vy·t − l·θ
    d_FR = s + Vy·t + l·θ        ,  l = (track + wheelbase)/2 = 0.505 m
직진 주행이므로 Vy≈0 으로 두면
    d_FL_ideal = s − l·θ ,  d_FR_ideal = s + l·θ

한편 실제 엔코더 적산값은 motor_node 의 출력에서 역산할 수 있다.
    x_odom  ≈ (D_L + D_R)/2          (보고된 yaw 가 작아 cos≈1 일 때)
    yaw_odom = (D_R − D_L)/(2l)
    ->  D_L = x_odom − l·yaw_odom ,  D_R = x_odom + l·yaw_odom

최종:
    scale_left  = d_FL_ideal / D_L
    scale_right = d_FR_ideal / D_R

[한계 — 반드시 읽을 것]
이 계수는 '엔코더 눈금 보정'이 아니라 '현재 결함 상태의 주행 모델 피팅'이다.
RL 이 접지하거나, 하중/속도/바닥재가 바뀌면 다시 틀어진다.
따라서 실주행 요(yaw)는 반드시 자이로에서 가져오고(use_gyro_for_yaw),
이 계수는 (a) 거리 스케일 보정과 (b) IMU 고장 시 폴백 용도로만 쓴다.
"""

import argparse
import math

TRACK_WIDTH_M = 0.51
WHEELBASE_M   = 0.50
L_MEC         = (TRACK_WIDTH_M + WHEELBASE_M) / 2.0   # 0.505 m


def solve(x_true, y_true, x_odom, yaw_odom, v_cmd=None, verbose=True):
    # ── 1. 원호 기하 ────────────────────────────────────────────
    if abs(y_true) < 1e-9:
        theta = 0.0
        arc = abs(x_true)
        radius = float('inf')
    else:
        theta = 2.0 * math.atan2(y_true, x_true)       # 총 회전각 [rad], + = 좌회전
        radius = x_true / math.sin(theta)
        arc = radius * theta
    chord = math.hypot(x_true, y_true)

    # ── 2. 이상적(무슬립) 전륜 롤링 거리 ────────────────────────
    d_fl_ideal = arc - L_MEC * theta
    d_fr_ideal = arc + L_MEC * theta

    # ── 3. 실제 엔코더 적산값 역산 ──────────────────────────────
    d_l = x_odom - L_MEC * yaw_odom
    d_r = x_odom + L_MEC * yaw_odom

    scale_left  = d_fl_ideal / d_l
    scale_right = d_fr_ideal / d_r
    scale_mean  = 0.5 * (scale_left + scale_right)
    balance     = scale_right / scale_left

    if verbose:
        print('─' * 66)
        print(f'  입력 : 실측 ({x_true:.3f}, {y_true:.3f}) m | '
              f'odom x={x_odom:.3f} m, yaw={yaw_odom:+.4f} rad '
              f'({math.degrees(yaw_odom):+.2f}°)')
        print('─' * 66)
        print(f'  총 회전각 θ      = {theta:+.5f} rad ({math.degrees(theta):+.2f}°)')
        print(f'  원호 반지름 R    = {radius:.3f} m')
        print(f'  현(chord)        = {chord:.4f} m   ← 줄자가 재는 값')
        print(f'  호(arc)          = {arc:.4f} m   ← 엔코더가 재는 값  '
              f'(차이 {(arc/chord-1)*100:+.2f} %)')
        print()
        print(f'  이상적 d_FL      = {d_fl_ideal:.4f} m')
        print(f'  이상적 d_FR      = {d_fr_ideal:.4f} m')
        print(f'  실제   D_L       = {d_l:.4f} m')
        print(f'  실제   D_R       = {d_r:.4f} m')
        print()
        print(f'  >>> encoder_scale_left  = {scale_left:.4f}')
        print(f'  >>> encoder_scale_right = {scale_right:.4f}')
        print(f'      (평균 {scale_mean:.4f} / 밸런스 R/L {balance:.4f} '
              f'= 우측이 {(balance-1)*100:+.2f}% 더 셈)')
        if v_cmd:
            ratio = (d_l + d_r) / 2.0 / arc
            print(f'      명령 속도 {v_cmd:.3f} m/s 에서 엔코더/실제 = {ratio:.4f} '
                  f'(슬립 {(ratio-1)*100:.1f}%)')
    return dict(theta=theta, arc=arc, chord=chord, radius=radius,
                d_fl_ideal=d_fl_ideal, d_fr_ideal=d_fr_ideal, d_l=d_l, d_r=d_r,
                scale_left=scale_left, scale_right=scale_right,
                scale_mean=scale_mean, balance=balance)


def fit_speed_model(points):
    """
    (v_cmd, ratio) 점들로 slip(v) = a + b·v 선형 회귀.
    ratio = 엔코더거리 / 실제거리.  k(v) = 1 / (1 + a + b·v)
    """
    n = len(points)
    if n < 2:
        return None
    sv = sum(v for v, _ in points)
    ss = sum(r - 1.0 for _, r in points)
    svv = sum(v * v for v, _ in points)
    svs = sum(v * (r - 1.0) for v, r in points)
    den = n * svv - sv * sv
    if abs(den) < 1e-12:
        return None
    b = (n * svs - sv * ss) / den
    a = (ss - b * sv) / n
    return a, b


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='엔코더 좌/우 스케일 캘리브레이션')
    p.add_argument('--x-true', type=float, required=True, help='도착점 X 실측 [m]')
    p.add_argument('--y-true', type=float, required=True, help='도착점 Y 실측 [m], + = 좌측')
    p.add_argument('--x-odom', type=float, required=True, help='/odom_motor 최종 x [m]')
    p.add_argument('--yaw-odom', type=float, default=0.0, help='/odom_motor 최종 yaw [rad]')
    p.add_argument('--v-cmd', type=float, default=None, help='명령 바퀴 표면속도 [m/s]')
    a = p.parse_args()
    solve(a.x_true, a.y_true, a.x_odom, a.yaw_odom, a.v_cmd)