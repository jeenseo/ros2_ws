"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일

수정 사항:
  - TF 트리 완성: base_footprint → base_link → lidar_link
    - base_link  → lidar_link: z=0.1m, yaw=180° (LiDAR 방향 반전 보정)
  - map_server + amcl + lifecycle_manager_localization 추가
  - rf2o base_frame_id: base_footprint
  - collision_monitor 제거

시작 노드:
  1. lidar_node          — RPLIDAR A1 (/scan)
  2. motor_node          — /cmd_vel → CAN TX
  3. keyboard_node       — MANUAL 제어 + /mode 토글
  4. nav2_goal_publisher — AUTO 모드 전방 목표 게시
  5. rf2o_laser_odometry — LiDAR 스캔 매칭 오도메트리 (/odom)
  6. static_tf (×2)     — base_footprint→base_link, base_link→lidar_link
  7. map_server          — 사전 빌드된 맵 로딩
  8. amcl                — 맵 기반 위치 추정
  9. lifecycle_manager_localization
  10. Nav2 navigation stack (navigation_launch.py)

전제 조건:
  sudo ip link set can0 up type can bitrate 500000
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-rf2o-laser-odometry
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')

    # ── Launch 인수 ───────────────────────────────────────────────
    goal_dist_arg = DeclareLaunchArgument(
        'goal_distance_m', default_value='3.0',
        description='AUTO 모드 전방 목표 거리 (m)'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Nav2 스택 활성화 여부'
    )
    fov_arg = DeclareLaunchArgument(
        'fov_degrees', default_value='360.0',
        description='LiDAR 전방위 스캔 각도 (Nav2 기본: 360°)'
    )

    # 파일 경로
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml_file    = os.path.join(pkg_dir, 'maps',   'map.yaml')

    # ─────────────────────────────────────────────────────────────
    # ── TF 트리 ──────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────

    # TF1: base_footprint → base_link (항등 변환, 지면 기준점)
    tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_base_tf',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0', '1',  # qx qy qz qw (회전 없음)
            'base_footprint',
            'base_link',
        ],
        output='screen',
    )

    # TF2: base_link → lidar_link
    #   - z=0.1m: LiDAR 높이
    #   - yaw=180° (π): LiDAR 물리적 방향 반전 보정
    #     quaternion(yaw=π): qx=0, qy=0, qz=sin(π/2)=1, qw=cos(π/2)=0
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=[
            '0', '0', '0.1',   # x y z (LiDAR 높이 10cm)
            '0', '0', '1', '0',  # qx qy qz qw (yaw=180°)
            'base_link',
            'lidar_link',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────
    # ── 하드웨어 노드 ─────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────

    # 1. LiDAR 노드
    lidar_node = Node(
        package='robot_controller',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'port':         '/dev/ttyUSB0',
            'baud':         115200,
            'fov_degrees':  LaunchConfiguration('fov_degrees'),
            'lidar_offset': 0.0,   # 180° 보정은 TF에서 처리
        }],
    )

    # 2. 모터 노드
    motor_node = Node(
        package='robot_controller',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'can_channel': 'can0',
            'can_id':      0x123,
            'max_speed':   9999,
        }],
    )

    # 3. 키보드 노드 (별도 터미널)
    keyboard_node = Node(
        package='robot_controller',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'normal_speed': 0.2002,
            'boost_speed':  0.4001,
        }],
    )

    # 4. Nav2 목표 게시 노드
    nav2_goal_publisher = Node(
        package='robot_controller',
        executable='nav2_goal_publisher',
        name='nav2_goal_publisher',
        output='screen',
        parameters=[{
            'goal_distance_m': LaunchConfiguration('goal_distance_m'),
            'update_rate_hz':  0.5,
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 오도메트리: rf2o (엔코더 없는 스캔 매칭) ──────────────────
    # ─────────────────────────────────────────────────────────────
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom',
            'publish_tf':           True,
            'base_frame_id':        'base_footprint',   # ← 수정
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 15.0,               # 약간 빠르게
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 맵 서버 + AMCL + 수명 주기 관리자 (위치 추정) ─────────────
    # ─────────────────────────────────────────────────────────────

    # map_server: 사전 빌드된 맵 로드
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'yaml_filename': map_yaml_file,
        }],
    )

    # amcl: 맵 기반 Monte Carlo 위치 추정
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

    # lifecycle_manager: map_server + amcl 수명 주기 관리
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── Nav2 네비게이션 스택 ───────────────────────────────────────
    # ─────────────────────────────────────────────────────────────
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  nav2_params_file,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Launch 인수
        goal_dist_arg,
        use_nav2_arg,
        fov_arg,

        # TF 트리
        tf_footprint_to_base,
        tf_base_to_lidar,

        # 하드웨어
        lidar_node,
        motor_node,
        keyboard_node,
        nav2_goal_publisher,

        # 오도메트리
        rf2o_node,

        # 위치 추정 (맵 + AMCL)
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        # Nav2 네비게이션
        nav2_launch,
    ])
