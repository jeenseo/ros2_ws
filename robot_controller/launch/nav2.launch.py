"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일 — Primary Entry Point

주요 수정:
  - sllidar_ros2 (src 기반) 드라이버 통합
  - ROS_DISCOVERY_SERVER / ROS_DOMAIN_ID 환경변수 설정
  - 55cm footprint 기반 Nav2 파라미터 연동
  - 모터 Buzzing 방지: inflation_radius > avoidance_node 임계값

시작 노드:
  1. sllidar_ros2_node  — src/sllidar_ros2 기반 LIDAR 드라이버 (/scan)
  2. motor_node         — /cmd_vel → CAN TX (E-Stop 내장)
  3. keyboard_node      — MANUAL 제어 + /mode 토글
  4. nav2_goal_publisher— AUTO 모드 전방 목표 게시
  5. rf2o_laser_odometry— LiDAR 스캔 매칭 오도메트리 (/odom)
  6. static_tf (×2)    — base_footprint→base_link, base_link→lidar_link
  7. map_server         — 사전 빌드된 맵 로딩
  8. amcl               — 맵 기반 위치 추정
  9. lifecycle_manager_localization
  10. Nav2 navigation stack

전제 조건:
  source ~/ros2_ws/install/setup.bash
  sudo ip link set can0 up type can bitrate 500000
  (또는 setup_env.sh 실행)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')

    # ── 환경변수 설정 (SSH 세션 안정성) ──────────────────────────
    # ROS_DOMAIN_ID: 동일 도메인 내 로봇끼리만 통신
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='30',
    )
    # ROS_DISCOVERY_SERVER: Fast-RTPS 디스커버리 서버 주소
    # Raspberry Pi 고정 IP에 맞게 수정하세요.
    set_discovery_server = SetEnvironmentVariable(
        name='ROS_DISCOVERY_SERVER',
        value='10.201.216.95:11811',   # ← Pi의 실제 IP로 변경
    )

    # ── Launch 인수 ───────────────────────────────────────────────
    goal_dist_arg = DeclareLaunchArgument(
        'goal_distance_m', default_value='3.0',
        description='AUTO 모드 전방 목표 거리 (m)'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Nav2 스택 활성화 여부'
    )

    # 파일 경로
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml_file    = os.path.join(pkg_dir, 'maps',   'map.yaml')

    # ─────────────────────────────────────────────────────────────
    # ── TF 트리 ──────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────

    # TF1: base_footprint → base_link (지면 기준점, 항등 변환)
    tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'base_footprint', 'base_link'],
        output='screen',
    )

    # TF2: base_link → lidar_link (실측 기반)
    #   x=+0.155m: LiDAR가 로봇 중심에서 15.5cm 전방
    #   z=+0.655m: LiDAR 광학 중심이 지면에서 65.5cm
    #   yaw=180° (qz=1, qw=0): LiDAR 물리적 방향 반전 보정
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.155', '0', '0.655', '0', '0', '1', '0',
                   'base_link', 'lidar_link'],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────
    # ── LIDAR: sllidar_ros2 (src 기반 드라이버) ───────────────────
    # ─────────────────────────────────────────────────────────────
    # 기존 rplidar_ros / 커스텀 lidar_node 대신 src/sllidar_ros2 사용
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_ros2_node',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/rplidar',  # udev 심볼릭 링크 권장
            'serial_baudrate':  115200,
            'frame_id':         'lidar_link',    # TF 트리와 일치
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Express',       # RPLiDAR A1M8 최적 모드
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 하드웨어 노드 ─────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────

    # 모터 노드 (E-Stop 내장 → Buzzing 방지)
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

    # 키보드 노드 (별도 터미널 — SSH headless 환경 주의)
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

    # Nav2 목표 게시 노드
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
            'base_frame_id':        'base_footprint',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 15.0,
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 맵 + AMCL + 수명주기 관리자 (위치 추정) ─────────────────
    # ─────────────────────────────────────────────────────────────
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

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

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
    # ── Nav2 네비게이션 스택 (55cm footprint 파라미터 연동) ────────
    # ─────────────────────────────────────────────────────────────
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  nav2_params_file,  # 55cm footprint 적용된 파라미터
            'autostart':    'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        # 환경변수 (가장 먼저)
        set_domain_id,
        set_discovery_server,

        # Launch 인수
        goal_dist_arg,
        use_nav2_arg,

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

        # 위치 추정
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        # Nav2 (55cm footprint params 연동)
        nav2_launch,
    ])
