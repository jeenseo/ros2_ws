"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일 — Raspberry Pi 로컬 실행 전용 (Foxglove Studio)

아키텍처:
  LiDAR → rf2o → /odom_rf2o → rf2o_covariance_relay → /odom_rf2o_cov
                                                         ↓
  MPU6050 → /imu/data → imu_gyro_bias_node → /imu/data_unbiased
                                                         ↓
                                                 EKF (robot_localization)
                                                         ↓
                                                 /odom + TF(odom→base_footprint) → Nav2
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
    map_yaml_file    = os.path.join(pkg_dir, 'maps', 'capstone_map.yaml')
    ekf_yaml_file    = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    urdf_file        = os.path.join(pkg_dir, 'urdf',   'robot.urdf')

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── robot_state_publisher (URDF 기반 TF 발행) ────────────────
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

    # ── LIDAR: sllidar_ros2 ───────────────────────────────────────
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

    # ── IMU: MPU6050 ─────────────────────────────────────────────
    mpu6050_node = Node(
        package='robot_controller',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'i2c_bus':     1,
            'i2c_address': 0x68,
            'publish_hz':  30.0,
            'frame_id':    'imu_link',
            'alpha':       0.98,
        }],
    )

    # ★ 신규: IMU 유령 데이터(바이어스) 제거 노드
    imu_gyro_bias_node = Node(
        package='robot_controller',
        executable='imu_gyro_bias_node',
        name='imu_gyro_bias_node',
        output='screen',
    )

    # ── 하드웨어: 모터 노드 ───────────────────────────────────────
    motor_node = Node(
        package='robot_controller',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'can_channel': 'can0',
            'can_id':      0x123,
            'max_speed':   9999,
            'odom_frame':  'odom',
            'base_frame':  'base_footprint',
            'odom_topic':  '/odom_motor', 
            'imu_topic':   '/imu/data_unbiased',  # ★ 변경됨: 정수기를 거친 깨끗한 물을 마심
        }],
        remappings=[('/cmd_vel_nav2', '/cmd_vel')],
    )

    # ── 카메라: camera_ros ───────────────────────────────────────
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'RGB888',
            'AfMode': 0,
            'LensPosition': 2.0,
            'fps': 10.0,
        }],
    )

    # ── 오도메트리: rf2o (LiDAR 스캔 매칭) ───────────────────────
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
            'freq':                 10.0,
        }],
    )

    # ★ 신규: rf2o 위치 좌표를 EKF용 속도(Twist)로 변환하는 노드
    rf2o_covariance_relay_node = Node(
        package='robot_controller',
        executable='rf2o_covariance_relay',
        name='rf2o_covariance_relay',
        output='screen',
    )

    # ── EKF: IMU + LiDAR Odom 융합 ───────────────────────────────
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

    # ── 맵 + AMCL + 수명주기 관리자 (위치 추정) ─────────────────
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

    # ── Nav2 네비게이션 스택 ──────────────────────────────────────
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
        use_nav2_arg,

        robot_state_publisher_node,

        lidar_node,
        mpu6050_node,
        imu_gyro_bias_node,          # ★ 신규 편입
        motor_node,
        # camera_node,  

        rf2o_node,
        rf2o_covariance_relay_node,  # ★ 신규 편입
        ekf_node,

        map_server_node,
        amcl_node,
        lifecycle_manager_localization,

        nav2_launch,
    ])