#!/usr/bin/env python3
"""
check_config.py — 기동 전 설정 정합성 검사 (Pre-flight check)
================================================================================
왜 필요한가
--------------------------------------------------------------------------------
ROS 2 는 설정이 어긋나도 **에러를 내지 않습니다.**

  - YAML 최상위 키가 노드 이름과 다르면  -> 그 파일 전체를 조용히 무시
  - YAML 안의 파라미터가 declare 되지 않았으면 -> 그 항목만 조용히 무시

둘 다 "노드는 멀쩡히 떠 있는데 아무 설정도 안 먹은" 상태를 만듭니다.
이 프로젝트에서 실제로 세 번 겪었습니다:

  1. `ekf_filter_node` vs `ekf_node`  -> ekf.yaml 전체 무시 -> odom 프레임 증발
  2. `i2c_addr` vs `i2c_address`      -> MPU6050 주소 미적용
  3. `degeneracy_enable` vs `enable_degeneracy_boost` -> 라이다 퇴화 훅 미작동

셋 다 로그에 한 줄도 남지 않았습니다. 그래서 기계적으로 대조하는 도구를 둡니다.

사용법
--------------------------------------------------------------------------------
    python3 check_config.py                 # 기본 경로 자동 탐색
    python3 check_config.py --root ~/ros2_ws/src/robot_controller

종료코드 0 = 통과, 1 = 문제 발견. CI 나 launch 전 훅에 걸어도 됩니다.
"""

import argparse
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    print('PyYAML 이 필요합니다:  pip install pyyaml --break-system-packages')
    sys.exit(1)


RED, GRN, YLW, RST = '\033[31m', '\033[32m', '\033[33m', '\033[0m'
problems = []
warnings = []


def fail(msg):
    problems.append(msg)
    print(f'  {RED}[FAIL]{RST} {msg}')


def warn(msg):
    warnings.append(msg)
    print(f'  {YLW}[WARN]{RST} {msg}')


def ok(msg):
    print(f'  {GRN}[ OK ]{RST} {msg}')


def head(t):
    print(f'\n{"="*72}\n{t}\n{"="*72}')


# ══════════════════════════════════════════════════════════════════════════
def parse_launch_nodes(path):
    """launch 파일에서 Node(package=, executable=, name=, parameters=) 를 뽑습니다.

    정규식으로 훑습니다. launch 파일은 실행해 봐야 최종 값이 정해지는 구조라
    (LaunchConfiguration 등) 정적 파싱에는 한계가 있지만, 우리가 잡으려는
    '이름 오타'는 대부분 리터럴이므로 이 정도로 충분히 걸립니다.
    """
    src = open(path).read()
    nodes = []
    for m in re.finditer(r'Node\s*\(', src):
        # 괄호 균형을 맞춰 Node(...) 블록 끝을 찾습니다.
        i = m.end() - 1
        depth = 0
        for j in range(i, len(src)):
            if src[j] == '(':
                depth += 1
            elif src[j] == ')':
                depth -= 1
                if depth == 0:
                    break
        blk = src[m.start():j + 1]
        def grab(key):
            mm = re.search(rf"{key}\s*=\s*'([^']+)'", blk)
            return mm.group(1) if mm else None
        nodes.append({'name': grab('name'),
                      'executable': grab('executable'),
                      'package': grab('package'),
                      'block': blk})
    return nodes


def yaml_top_keys(path):
    d = yaml.safe_load(open(path))
    return list(d.keys()) if isinstance(d, dict) else []


def declared_params(src_path):
    return set(re.findall(r"declare_parameter\(\s*'([^']+)'", open(src_path).read()))


# ══════════════════════════════════════════════════════════════════════════
def check_node_names(root, launch_nodes):
    head('[1] YAML 최상위 키 <-> launch 의 node name')
    ekf = os.path.join(root, 'config', 'ekf.yaml')
    if os.path.exists(ekf):
        keys = yaml_top_keys(ekf)
        ekf_nodes = [n for n in launch_nodes if n['executable'] == 'ekf_node']
        if not ekf_nodes:
            warn('launch 에서 ekf_node 를 찾지 못했습니다 (수동 확인 필요)')
        for n in ekf_nodes:
            if n['name'] in keys:
                ok(f"ekf.yaml 최상위 '{n['name']}' == launch name")
            else:
                fail(f"ekf.yaml 최상위 {keys} 인데 launch 는 name='{n['name']}'\n"
                     f"         -> ekf.yaml 이 통째로 무시되고 EKF 가 파라미터 0개로 뜹니다.\n"
                     f"         -> odom 프레임이 생기지 않습니다.")

    rp = os.path.join(root, 'config', 'robot_params.yaml')
    if os.path.exists(rp):
        keys = [k.lstrip('/') for k in yaml_top_keys(rp)]
        launch_names = {n['name'] for n in launch_nodes if n['name']}
        for k in keys:
            if k in launch_names:
                ok(f"robot_params.yaml '/{k}' 대응 노드 존재")
            elif k in ('keyboard_node', 'cruise_node'):
                ok(f"robot_params.yaml '/{k}' (launch 미포함, ros2 run 전용 — 정상)")
            else:
                warn(f"robot_params.yaml '/{k}' 에 대응하는 launch 노드가 없습니다")


def check_param_names(root):
    head('[2] YAML 파라미터 이름 <-> 소스의 declare_parameter')
    rp = os.path.join(root, 'config', 'robot_params.yaml')
    if not os.path.exists(rp):
        warn('robot_params.yaml 없음'); return
    cfg = yaml.safe_load(open(rp))
    srcdirs = [os.path.join(root, 'scripts')] + \
              [d for d in glob.glob(os.path.join(root, '*')) if os.path.isdir(d)]
    for node, body in cfg.items():
        base = node.lstrip('/')
        cand = None
        for d in srcdirs:
            p = os.path.join(d, base + '.py')
            if os.path.exists(p):
                cand = p; break
        if cand is None:
            warn(f'{node}: 소스 파일을 못 찾아 건너뜁니다')
            continue
        used = set(body.get('ros__parameters', {}).keys())
        miss = sorted(used - declared_params(cand))
        if miss:
            for m in miss:
                fail(f'{node}: 미선언 파라미터 \'{m}\' — 조용히 무시됩니다')
        else:
            ok(f'{node}: {len(used)}개 전부 선언됨')


def check_ekf(root):
    head('[3] ekf.yaml 구조')
    p = os.path.join(root, 'config', 'ekf.yaml')
    if not os.path.exists(p):
        warn('ekf.yaml 없음'); return None
    d = yaml.safe_load(open(p))
    prm = d[list(d.keys())[0]]['ros__parameters']
    for k, v in sorted(prm.items()):
        if k.endswith('_config') and isinstance(v, list):
            if len(v) == 15 and all(isinstance(x, bool) for x in v):
                ok(f'{k} 15개 bool')
            else:
                fail(f'{k} 길이 {len(v)} (15개 bool 이어야 함)')
    for k in ('process_noise_covariance', 'initial_estimate_covariance'):
        if k in prm:
            if len(prm[k]) == 225:
                ok(f'{k} 225개')
            else:
                fail(f'{k} 길이 {len(prm[k])} (15x15=225 여야 함)')
    # 1e-9 문자열 오파싱
    bad = []
    def scan(x, path=''):
        if isinstance(x, dict):
            for k, v in x.items(): scan(v, f'{path}.{k}')
        elif isinstance(x, list):
            for i, v in enumerate(x): scan(v, f'{path}[{i}]')
        elif isinstance(x, str) and any(c.isdigit() for c in x) and ('e-' in x or 'e+' in x):
            bad.append((path, x))
    scan(prm)
    if bad:
        for b in bad:
            fail(f'지수표기가 문자열로 파싱됨 {b} — `1.0e-9` 형태로 쓰십시오')
    else:
        ok('지수표기 오파싱 없음')
    return prm


def check_frames(root, ekf_prm):
    head('[4] 프레임 정합 (URDF <-> ekf.yaml <-> nav2_params.yaml <-> launch)')
    urdf = os.path.join(root, 'urdf', 'robot.urdf')
    links = set()
    if os.path.exists(urdf):
        r = ET.parse(urdf).getroot()
        links = {l.get('name') for l in r.findall('link')}
        children = {j.find('child').get('link') for j in r.findall('joint')}
        roots = links - children
        if len(roots) == 1:
            ok(f'URDF 루트 링크 = {roots.pop()}')
        else:
            fail(f'URDF 루트 링크가 {roots} — 정확히 1개여야 합니다')
        # 라이다 방향
        for j in r.findall('joint'):
            if j.get('name') == 'lidar_joint':
                rpy = j.find('origin').get('rpy', '0 0 0').split()
                yaw = float(rpy[2])
                ok(f'lidar_joint yaw = {yaw:.5f} rad ({yaw*57.2958:.1f}°)')
                if abs(yaw) > 0.1:
                    warn('lidar_joint 에 회전이 있습니다 — 그 자체는 문제가 아닙니다.\n'
                         '         물리 장착과 일치하는지만 확인하십시오:\n'
                         '           ros2 run robot_controller lidar_check\n'
                         '         (정면 1m 물체 -> base_link 방위 0도면 정상, 180도면 오류)')

    if ekf_prm:
        base = ekf_prm.get('base_link_frame')
        if base in links or not links:
            ok(f'ekf.yaml base_link_frame = {base} (URDF 에 존재)')
        else:
            fail(f'ekf.yaml base_link_frame = {base} 가 URDF 링크에 없습니다 {sorted(links)}')

    n2 = os.path.join(root, 'config', 'nav2_params.yaml')
    if os.path.exists(n2) and ekf_prm:
        txt = open(n2).read()
        base = ekf_prm.get('base_link_frame')
        wrong = re.findall(r'(?:robot_)?base_frame(?:_id)?:\s*"?([\w/]+)"?', txt)
        bad = [w for w in set(wrong) if w != base]
        if bad:
            fail(f'nav2_params.yaml 의 base frame {bad} 가 ekf.yaml 의 {base} 와 다릅니다\n'
                 f"         -> 'Robot is out of bounds' 로 코스트맵이 죽습니다")
        else:
            ok(f'nav2_params.yaml base frame 전부 {base}')

    # sllidar frame_id vs URDF
    for lp in glob.glob(os.path.join(root, 'launch', '*.launch.py')):
        m = re.search(r"'frame_id':\s*'([^']+)'", open(lp).read())
        if m:
            fid = m.group(1)
            if not links or fid in links:
                ok(f'sllidar frame_id = {fid} (URDF 에 존재)')
            else:
                fail(f'sllidar frame_id = {fid} 가 URDF 링크에 없습니다 -> TF 끊김')


def check_speed(root):
    head('[5] 속도/이득 물리 정합')
    rp = os.path.join(root, 'config', 'robot_params.yaml')
    if not os.path.exists(rp):
        return
    p = yaml.safe_load(open(rp)).get('/motor_node', {}).get('ros__parameters', {})
    if not p:
        return
    import math
    D = p.get('wheel_diameter', 0.12)
    M = p.get('max_wheel_speed', 0.95)
    v_mech = 146.0 / 60.0 * math.pi * D
    if M <= v_mech:
        ok(f'max_wheel_speed {M} <= 기계적 상한 {v_mech:.3f} m/s')
    else:
        fail(f'max_wheel_speed {M} 가 기계적 상한 {v_mech:.3f} m/s 초과')
    L = 0.505
    wz_max = M / L
    lim = p.get('yaw_comp_limit', 0.35)
    if lim <= wz_max:
        ok(f'yaw_comp_limit {lim} <= 최대 요레이트 {wz_max:.3f} rad/s')
    else:
        fail(f'yaw_comp_limit {lim} 가 최대 요레이트 {wz_max:.3f} 초과')
    n2 = os.path.join(root, 'config', 'nav2_params.yaml')
    if os.path.exists(n2):
        t = open(n2).read()
        mx = re.search(r'max_vel_x:\s*([\d.]+)', t)
        if mx and float(mx.group(1)) > M:
            fail(f"nav2 max_vel_x {mx.group(1)} 가 max_wheel_speed {M} 초과 — 도달 불가")
        elif mx:
            ok(f'nav2 max_vel_x {mx.group(1)} <= max_wheel_speed {M}')


# ══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='.', help='패키지 소스 루트')
    a = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(a.root))
    print(f'검사 대상: {root}')

    lp = glob.glob(os.path.join(root, 'launch', '*.launch.py'))
    nodes = []
    for f in lp:
        nodes += parse_launch_nodes(f)

    check_node_names(root, nodes)
    check_param_names(root)
    prm = check_ekf(root)
    check_frames(root, prm)
    check_speed(root)

    head('결과')
    if problems:
        print(f'  {RED}문제 {len(problems)}건{RST} — 고치기 전에는 기동하지 마십시오.')
    else:
        print(f'  {GRN}치명적 문제 없음{RST}')
    if warnings:
        print(f'  {YLW}경고 {len(warnings)}건{RST}')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())