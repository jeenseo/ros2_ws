"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일 — Raspberry Pi 로컬 실행 전용 (Foxglove Studio)

아키텍처:
  LiDAR → rf2o → /odom_rf2o (TF 없음)
                         ↓
  MPU6050 → /imu/data   ↓
                 EKF (robot_localization)
                         ↓
                 /odom + TF(odom→base_footprint) → Nav2

TF 트리 (URDF 기반, robot_state_publisher가 발행):
  base_footprint → base_link
                      ├─ wheel_front_left/right_link
                      ├─ wheel_rear_left/right_link
                      ├─ lidar_link  (x=0.160, z=0.355, yaw=180°)
                      └─ imu_link    (x=0.160, z=0.325)

수정 이력:
  - [수정] static_transform_publisher 노드 전체 제거
           → robot_state_publisher (URDF) 로 대체
  - [추가] mpu6050_node + EKF (robot_localization)
  - [수정] rf2o_node: publish_tf=False, /odom_rf2o
  - motor_node remapping: /cmd_vel_nav2 ← /cmd_vel
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
    ekf_yaml_file    = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    urdf_file        = os.path.join(pkg_dir, 'urdf',   'robot.urdf')

    # URDF 파일 내용 읽기 (robot_state_publisher 파라미터로 전달)
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ─────────────────────────────────────────────────────────────
    # ── robot_state_publisher (URDF 기반 TF 발행) ────────────────
    # ─────────────────────────────────────────────────────────────
    # [수정] static_transform_publisher 3개 완전 제거
    #        → robot_state_publisher가 URDF의 모든 fixed joint TF를 담당:
    #            base_footprint → base_link
    #            base_link → lidar_link  (x=0.160, z=0.355, yaw=180°)
    #            base_link → imu_link    (x=0.160, z=0.325)
    #            base_link → wheel_*_link (4개)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time':      False,
        }],
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
    # ── IMU: MPU6050 ─────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────
    mpu6050_node = Node(
        package='robot_controller',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'i2c_bus':     1,
            'i2c_address': 0x68,
            'publish_hz':  20.0,
            'frame_id':    'imu_link',
            'alpha':       0.98,
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 하드웨어: 모터 노드 ───────────────────────────────────────
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
        # Nav2의 /cmd_vel 출력을 /cmd_vel_nav2로 수신
        # → motor_node 내부 mux가 AUTO 모드에서만 CAN 전달
        remappings=[('/cmd_vel_nav2', '/cmd_vel')],
    )

    # ─────────────────────────────────────────────────────────────
    # ── 오도메트리: rf2o (LiDAR 스캔 매칭) ───────────────────────
    # ─────────────────────────────────────────────────────────────
    # publish_tf=False: EKF가 odom→base_footprint TF 담당
    # odom_topic=/odom_rf2o: EKF의 odom0 입력 토픽
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom_rf2o',
            'publish_tf':           False,
            'base_frame_id':        'base_footprint',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 5.0,
        }],
    )

    # ─────────────────────────────────────────────────────────────
    # ── EKF: IMU + LiDAR Odom 융합 ───────────────────────────────
    # ─────────────────────────────────────────────────────────────
    # /odometry/filtered → /odom 리맵
    # odom → base_footprint TF 발행 (publish_tf: true in ekf.yaml)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_yaml_file],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
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
    # ── Nav2 네비게이션 스택 ──────────────────────────────────────
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

        # URDF 기반 TF 트리 (static_transform_publisher 대체)
        robot_state_publisher_node,

        # 하드웨어 (LiDAR, IMU, Motor)
        lidar_node,
        mpu6050_node,
        motor_node,

        # 오도메트리 + EKF 융합
        rf2o_node,
        ekf_node,

        # 위치 추정 (Map + AMCL)
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        # Nav2 플래너/컨트롤러 스택
        nav2_launch,
    ])
