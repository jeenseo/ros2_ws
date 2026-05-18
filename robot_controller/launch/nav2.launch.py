"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일 — Raspberry Pi 로컬 실행 전용 (Foxglove Studio)

주요 수정:
  - sllidar_ros2 (src 기반) 드라이버 통합
  - ROS_DOMAIN_ID / ROS_DISCOVERY_SERVER 환경변수 완전 제거 (순수 로컬 통신)
  - 55cm footprint 기반 Nav2 파라미터 연동
  - [수정] nav2_goal_publisher 완전 제거 → Foxglove에서 직접 NavigateToPose Goal 설정
  - [수정] motor_node remapping: /cmd_vel_nav2 ← /cmd_vel (Nav2 출력)
    → motor_node 내부 mode-aware mux에서 AUTO 모드에서만 CAN 전송
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')

    # ── Launch 인수 ───────────────────────────────────────────────
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

    # TF2: base_link → lidar_link (물리 실측 기반 수정)
    #
    # 계산 근거:
    #   x = +0.160m  : 로봇 중심에서 전방 16cm (실측)
    #   y =  0.000m  : 좌우 중앙 정렬
    #   z = +0.355m  : 로봇 3D 중심 높이(61cm/2=30.5cm) + LiDAR 오프셋(+5cm) = 35.5cm
    #
    # rotation (qx,qy,qz,qw) = (0,0,0,1): identity (회전 없음)
    #   ※ LiDAR가 물리적으로 뒤집혀 마운트된 경우 qz=1, qw=0 (yaw=180°)으로 변경
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.160', '0.0', '0.355', '3.14159', '0.0', '0.0', 'base_link', 'lidar_link'],
        output='screen'
    )

    # ─────────────────────────────────────────────────────────────
    # ── LIDAR: sllidar_ros2 ───────────────────────────────────────
    # ─────────────────────────────────────────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_ros2_node',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'lidar_link',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 하드웨어 노드 ─────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────

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
        # [핵심 remapping]
        # motor_node 내부: /cmd_vel_nav2 구독
        # 실제 ROS 그래프: Nav2 controller가 게시하는 /cmd_vel 수신
        # → motor_node의 mode-aware mux가 AUTO 모드에서만 CAN 전달
        remappings=[('/cmd_vel_nav2', '/cmd_vel')],
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
            'params_file':  nav2_params_file,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Launch 인수
        use_nav2_arg,

        # TF 트리
        tf_footprint_to_base,
        tf_base_to_lidar,

        # 하드웨어 (lidar, motor)
        lidar_node,
        motor_node,

        # 오도메트리
        rf2o_node,

        # 위치 추정 (map + AMCL)
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        # Nav2 플래너/컨트롤러 스택
        nav2_launch,
    ])
