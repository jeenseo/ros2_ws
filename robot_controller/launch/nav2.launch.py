#!/usr/bin/env python3
"""
nav2.launch.py — 단일 통합 브링업
================================================================================
    ros2 launch <PKG> nav2.launch.py map:=/absolute/path/to/map.yaml

포함:  robot_state_publisher, sllidar, mpu6050, imu_gyro_bias, motor_node,
       rf2o, rf2o_covariance_relay, ekf_node, map_server, amcl, nav2_bringup
제외:  avoidance_node  (Nav2 로컬 코스트맵과 cmd_vel 을 두고 싸웁니다)
       static_transform_publisher (URDF 가 전부 제공 — robot_state_publisher 담당)
       keyboard_node  (launch 로 띄우면 stdin 이 TTY 가 아니라 키 입력 불가.
                       수동 조종은 별도 터미널에서 `ros2 run <PKG> keyboard_node`)

--------------------------------------------------------------------------------
TF 트리
--------------------------------------------------------------------------------
    map ──(amcl)──> odom ──(ekf_node)──> base_footprint
                                              │ (URDF / robot_state_publisher)
                                              └─> base_link ──┬─> wheel_*_link ×4
                                                              ├─> lidar_link
                                                              └─> imu_link

  ★ odom -> base_footprint 를 만드는 주체는 **ekf_node 하나뿐**이어야 합니다.
    그래서 rf2o 는 publish_tf:=false 입니다. 두 노드가 같은 변환을 내면
    TF 가 두 값 사이에서 진동하고 코스트맵이 찢어집니다.

--------------------------------------------------------------------------------
★ 라이다 방향 — URDF 의 rpy 를 3.14159 -> 0 으로 수정했습니다
--------------------------------------------------------------------------------
    <origin xyz="0.16 0.0 0.67" rpy="0 0 0"/>       (이전: rpy="0 0 3.14159")

  실측에서 '앞 벽이 뒤에 찍히는' 현상이 나왔습니다. 즉 라이다의 0도 방향은
  물리적으로 로봇 정면을 향하며, URDF 가 주장하던 180도 회전이 틀린 것이었습니다.
  전제:
    1) sllidar 의 frame_id 가 정확히 'lidar_link' 여야 합니다.
       (기본값 'laser' 로 두면 URDF 에 없는 프레임이라 TF 가 끊깁니다)
    2) 각도 보정은 **오직 URDF 에서만** 하십시오. 코드나 드라이버 파라미터로
       또 빼면 이중 보정이 되어 원인 추적이 불가능해집니다.
    3) 앞뒤는 맞는데 **좌우만** 뒤집힌 경우는 회전이 아니라 '거울' 이며,
       그때만 sllidar 의 inverted 파라미터를 씁니다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ── 패키지 이름 자동 판별 ──────────────────────────────────────────────
#   설치 경로가  install/<pkg>/share/<pkg>/launch/nav2.launch.py  이므로
#   두 단계 위 디렉터리 이름이 곧 패키지 이름입니다.
#   (하드코딩해 두면 패키지명이 다를 때 'package not found' 로 죽습니다)
#   자동 판별이 실패하는 환경이면 아래 _FALLBACK 을 직접 적으십시오.
_FALLBACK = 'robot_controller'
try:
    PKG = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    get_package_share_directory(PKG)          # 실제로 찾히는지 확인
except Exception:
    PKG = _FALLBACK


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    nav2_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    default_nav2   = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_ekf    = os.path.join(pkg_share, 'config', 'ekf.yaml')
    default_urdf   = os.path.join(pkg_share, 'urdf',   'robot.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    nav2_params  = LaunchConfiguration('nav2_params_file')
    ekf_file     = LaunchConfiguration('ekf_file')
    urdf_file    = LaunchConfiguration('urdf_file')
    map_yaml     = LaunchConfiguration('map')
    autostart    = LaunchConfiguration('autostart')
    use_lidar    = LaunchConfiguration('use_lidar')
    use_rf2o     = LaunchConfiguration('use_rf2o')
    use_nav2     = LaunchConfiguration('use_nav2')
    lidar_port   = LaunchConfiguration('lidar_port')
    lidar_baud   = LaunchConfiguration('lidar_baudrate')

    declare = [
        DeclareLaunchArgument('use_sim_time',     default_value='false'),
        DeclareLaunchArgument('params_file',      default_value=default_params),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        DeclareLaunchArgument('ekf_file',         default_value=default_ekf),
        DeclareLaunchArgument('urdf_file',        default_value=default_urdf),
        DeclareLaunchArgument('map',              default_value='',
                              description='맵 yaml 절대경로'),
        DeclareLaunchArgument('autostart',        default_value='true'),
        DeclareLaunchArgument('use_lidar',        default_value='true'),
        DeclareLaunchArgument('use_rf2o',         default_value='true'),
        DeclareLaunchArgument('use_nav2',         default_value='true',
                              description='false 로 두면 센서+오도메트리+EKF 만 (단계별 점검용)'),
        DeclareLaunchArgument('lidar_port',       default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate',   default_value='115200',
                              description='A1/A2=115200, A3/S1=256000'),
    ]

    common = [params_file, {'use_sim_time': use_sim_time}]

    # ══════════════════════════════════════════════════════════════════
    # 1. URDF -> TF  (모든 정적 변환의 유일한 출처)
    # ══════════════════════════════════════════════════════════════════
    #   ParameterValue(..., value_type=str) 가 없으면 URDF 문자열이 YAML 로
    #   재파싱되면서 깨집니다. Jazzy 에서 특히 자주 걸리는 함정입니다.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    state_publishers = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': use_sim_time}]),
        # 모든 조인트가 fixed 이므로 joint_state_publisher 는 필요 없습니다.
        # (넣으면 오히려 존재하지 않는 조인트 상태를 광고하게 됩니다)
    ]

    # ══════════════════════════════════════════════════════════════════
    # 2. 센서
    # ══════════════════════════════════════════════════════════════════
    sensors = [
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen', condition=IfCondition(use_lidar),
             parameters=[{'serial_port': lidar_port,
                          'serial_baudrate': lidar_baud,
                          'frame_id': 'lidar_link',      # ★ URDF 와 반드시 일치
                          'inverted': False,
                          'angle_compensate': True,
                          'scan_mode': 'Standard',
                          'use_sim_time': use_sim_time}]),

        Node(package=PKG, executable='mpu6050_node', name='mpu6050_node',
             output='screen', parameters=common),

        # ★ motor_node 의 요레이트 보상기가 이 노드의 출력을 먹습니다.
        #   없으면 보상기는 IMU 타임아웃으로 영구 비활성이 되고 error 로 알립니다.
        Node(package=PKG, executable='imu_gyro_bias_node', name='imu_gyro_bias_node',
             output='screen', parameters=common),
    ]

    # ══════════════════════════════════════════════════════════════════
    # 3. 오도메트리 소스
    # ══════════════════════════════════════════════════════════════════
    odometry = [
        Node(package=PKG, executable='motor_node', name='motor_node',
             output='screen', parameters=common),

        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             condition=IfCondition(use_rf2o),
             parameters=[{'laser_scan_topic': '/scan',
                          'odom_topic': '/odom_rf2o',
                          'publish_tf': False,          # ★ TF 는 EKF 만
                          'base_frame_id': 'base_footprint',
                          'odom_frame_id': 'odom',
                          'laser_frame_id': 'lidar_link',
                          'init_pose_from_topic': '',
                          'freq': 10.0,
                          'use_sim_time': use_sim_time}]),

        # rf2o 원본은 공분산이 비어 있어 EKF 가 절대위치를 맹신합니다.
        # 이 릴레이가 body-frame twist + 공분산으로 변환해 넘깁니다.
        Node(package=PKG, executable='rf2o_covariance_relay',
             name='rf2o_covariance_relay', output='screen',
             condition=IfCondition(use_rf2o), parameters=common),
    ]

    # ══════════════════════════════════════════════════════════════════
    # 4. EKF   (odom -> base_footprint)
    # ══════════════════════════════════════════════════════════════════
    ekf = [
        # ★★ name 은 반드시 ekf.yaml 의 최상위 키와 **글자 그대로** 같아야 합니다.
        #   ekf.yaml 은 `ekf_node:` 로 시작하므로 여기도 'ekf_node' 입니다.
        #   전에 'ekf_filter_node'(robot_localization 예제 관례)로 적어 두었는데,
        #   그러면 ROS2 가 **에러 없이** yaml 전체를 무시하고 EKF 가 파라미터 0개로
        #   기동합니다. 센서 입력이 하나도 없어 odom->base_footprint TF 를 만들지
        #   못하고, 결국 `odom` 프레임 자체가 생기지 않습니다.
        #   -> `Invalid frame ID "odom" ... frame does not exist` 의 정체입니다.
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_node', output='screen',
             parameters=[ekf_file, {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odom')]),
    ]

    # ══════════════════════════════════════════════════════════════════
    # 5. AMCL + map_server   (map -> odom)
    # ══════════════════════════════════════════════════════════════════
    #   nav2_bringup 의 localization_launch.py 를 씁니다.
    #   map_server / amcl 을 Node 로 직접 띄우면 **lifecycle 전이(configure ->
    #   activate)를 직접 해줘야** 하고, 빠뜨리면 노드는 살아 있는데 아무 일도
    #   안 하는 상태가 됩니다. 가장 흔하고 가장 찾기 어려운 함정입니다.
    #   이 launch 가 lifecycle_manager 까지 함께 띄워 줍니다.
    localization = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'localization_launch.py')),
            condition=IfCondition(use_nav2),
            launch_arguments={'map': map_yaml,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': nav2_params}.items()),
    ]

    # ══════════════════════════════════════════════════════════════════
    # 6. Nav2 본체
    # ══════════════════════════════════════════════════════════════════
    navigation = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            condition=IfCondition(use_nav2),
            launch_arguments={'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': nav2_params}.items()),
    ]

    return LaunchDescription(
        declare
        + state_publishers
        + sensors
        + odometry
        # ★ 순차 기동. EKF 가 센서보다 먼저 뜨면 "no odom received" 를 쏟고,
        #   Nav2 가 TF 트리 완성 전에 뜨면 코스트맵이 빈 채로 초기화됩니다.
        + [TimerAction(period=4.0, actions=ekf)]
        + [TimerAction(period=8.0, actions=localization + navigation)]
    )