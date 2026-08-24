#!/usr/bin/env python3
"""
encoder_probe.py — 엔코더 '틱/바퀴회전' 직접 측정기
================================================================================
왜 이 도구가 필요한가
--------------------------------------------------------------------------------
10초 직진 3회로 얻은 유일한 하드 데이터는 **7049.4 ticks/m** 입니다.
그런데 이 값과 물리 상수의 관계는

        ticks/m  =  CPR / (pi * D)

**미지수 2개(CPR, D)에 식 1개** 입니다. 주행 시험을 100번 더 해도
이 둘은 분리되지 않습니다 — 관측 불가(unobservable)입니다.

  후보 (라벨 1:27, 13CPR 기준)          ticks/m    오차
    CPR=1404(x4체배), D=63.4mm           7038     -0.16%
    CPR=2808,         D=126.8mm          7038     -0.16%
  두 조합의 오도메트리는 **완전히 동일**합니다. 구분할 방법은 하나뿐입니다:
  **바퀴를 손으로 정확히 N바퀴 돌려서 틱을 직접 세는 것.**
  그러면 D 를 몰라도 CPR 이 단독으로 나옵니다.

이 스크립트는 motor_node 의 어떤 상수도 쓰지 않고 CAN 원시 프레임만 읽습니다.
따라서 지금 틀려 있는 값들에 오염되지 않습니다.

--------------------------------------------------------------------------------
측정 절차 (2분)
--------------------------------------------------------------------------------
 1. 로봇을 들어 올려 바퀴가 공중에 뜨게 하십시오. (모터 전원은 꺼도 됩니다.
    STM32 와 CAN 만 살아 있으면 됩니다 — 엔코더는 손으로 돌려도 셉니다.)
 2. ★ motor_node 를 반드시 끄십시오. 같은 CAN ID 를 두 프로세스가 읽으면
    프레임을 나눠 가져 틱을 놓칩니다.
 3. 앞바퀴 두 개에 테이프로 기준점을 표시하십시오.
 4. 이 스크립트를 실행하십시오.
       python3 encoder_probe.py --revs 10
 5. 화면에 'ZERO 완료' 가 뜨면, 표시한 바퀴를 **전진 방향으로 정확히 10바퀴**
    천천히 돌리십시오. 좌/우 따로 해도 되고 같이 해도 됩니다.
    (천천히 돌려야 합니다. 급하게 돌리면 20ms 프레임 사이에 int16 이 넘칩니다)
 6. Ctrl-C. 결과가 출력됩니다.

  ※ 10바퀴를 권하는 이유: 1바퀴만 돌리면 정지 위치 오차(±5도 = ±1.4%)가
    그대로 결과 오차가 됩니다. 10바퀴면 0.14% 로 줄어듭니다.

--------------------------------------------------------------------------------
읽는 법
--------------------------------------------------------------------------------
  출력된 'ticks/rev' 가 곧 encoder_cpr 입니다. 여기에는 지름이 개입하지 않습니다.
  그 다음 자로 바퀴 지름을 재서 wheel_diameter 에 넣으면 끝입니다.
  검산:  ticks/rev / (pi * D) 가 7049 ±2% 면 정합입니다.
"""

import argparse
import math
import struct
import sys
import time

import can

_CAN_RX_FB_ID = 0x124          # STM32 -> Pi 엔코더 피드백
_TICKS_M_MEASURED = 7049.4     # 주행 실측 (이 값과의 정합성 검산용)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--revs', type=float, default=10.0,
                    help='손으로 돌릴 바퀴 회전수 (기본 10)')
    ap.add_argument('--diameter', type=float, default=None,
                    help='바퀴 지름 [mm]. 주면 최종 파라미터까지 계산해 줍니다.')
    args = ap.parse_args()

    try:
        bus = can.interface.Bus(channel=args.channel, bustype='socketcan')
    except Exception as exc:
        print(f'CAN 열기 실패: {exc}', file=sys.stderr)
        return 1

    left = right = 0
    frames = 0
    max_abs = 0
    print(f'CAN {args.channel} 열림. 0x{_CAN_RX_FB_ID:03X} 대기 중...')
    print('ZERO 완료 — 지금부터 바퀴를 돌리십시오. 끝나면 Ctrl-C.\n')
    t0 = time.monotonic()

    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                if frames == 0 and time.monotonic() - t0 > 3.0:
                    print('\r프레임이 안 옵니다. STM32 전원 / motor_node 중복 실행을 '
                          '확인하십시오.', end='')
                continue
            # 에러 프레임 / 원격 프레임 / 다른 ID / 길이 불일치는 전부 버립니다.
            if msg.is_error_frame or msg.is_remote_frame:
                continue
            if msg.arbitration_id != _CAN_RX_FB_ID or msg.dlc != 4:
                continue

            dl, dr = struct.unpack('>hh', bytes(msg.data[:4]))
            left += dl
            right += dr
            frames += 1
            max_abs = max(max_abs, abs(dl), abs(dr))
            print(f'\r  좌 {left:+8d}   우 {right:+8d}   '
                  f'(프레임 {frames}, 프레임당 최대 {max_abs})   ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    print('\n')
    if frames == 0:
        print('수신 프레임 0. 측정 불가.')
        return 1

    print('=' * 70)
    print(f'누적 틱     좌 {left:+d}   우 {right:+d}      (프레임 {frames})')
    if max_abs > 30000:
        print('⚠ 프레임당 틱이 int16 한계에 근접했습니다. 더 천천히 돌려 다시 재십시오.')

    n = args.revs
    res = []
    for lbl, val in (('좌(FL)', left), ('우(FR)', right)):
        if val == 0:
            continue
        cpr = abs(val) / n
        res.append(cpr)
        print(f'  {lbl}: {abs(val)} ticks / {n} rev = ★ {cpr:.1f} ticks/rev')

    if not res:
        print('양쪽 다 0 입니다. 바퀴를 돌리셨습니까?')
        return 1

    cpr = sum(res) / len(res)
    print('=' * 70)
    print(f'★ 측정 CPR (바퀴 1회전당 틱) = {cpr:.1f}')
    if len(res) == 2:
        diff = 100.0 * abs(res[0] - res[1]) / cpr
        print(f'  좌우 편차 {diff:.2f}%'
              + ('  -> 1% 이내면 좌우 동일 취급 가능'
                 if diff < 1.0 else '  -> ★ 유의미. 좌우 스케일 분리 필요'))

    # 후보 대조 — 라벨(13 CPR, 1:27) 기준
    print()
    print('  라벨 대조 (GP36E13CPR, 1:27):')
    for mult in (1, 2, 4):
        cand = 13 * mult * 27
        print(f'    x{mult} 체배 -> {cand:5d}   오차 {100*(cand-cpr)/cpr:+7.2f}%')

    # 지름 역산 / 검산
    print()
    d_implied = cpr / _TICKS_M_MEASURED / math.pi
    print(f'  주행 실측({_TICKS_M_MEASURED:.1f} ticks/m)과 정합하려면')
    print(f'    바퀴 지름 = {d_implied*1000:.1f} mm  <- 자로 재서 확인하십시오')

    if args.diameter:
        D = args.diameter / 1000.0
        tpm = cpr / (math.pi * D)
        err = 100.0 * (tpm - _TICKS_M_MEASURED) / _TICKS_M_MEASURED
        print()
        print(f'  입력하신 지름 {args.diameter:.1f} mm 로 계산:')
        print(f'    ticks/m = {tpm:.1f}   (주행 실측 대비 {err:+.2f}%)')
        print()
        print('  ── robot_params.yaml 에 넣을 값 ──────────────────────')
        print(f'    encoder_cpr:    {round(cpr)}')
        print(f'    wheel_diameter: {D:.4f}')
        print(f'    encoder_scale_left:  {1.0/(1.0+err/100.0):.4f}')
        print(f'    encoder_scale_right: {1.0/(1.0+err/100.0):.4f}')
        if abs(err) > 5.0:
            print('    ⚠ 오차 5% 초과. 회전수나 지름 측정을 다시 확인하십시오.')
    else:
        print()
        print('  지름을 재신 뒤 --diameter <mm> 를 붙여 다시 실행하면')
        print('  YAML 에 넣을 최종 값까지 계산해 드립니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())