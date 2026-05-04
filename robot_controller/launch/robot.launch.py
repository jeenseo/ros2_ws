from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ── Launch 인수 선언 (커맨드라인에서 오버라이드 가능) ───────
    fov_arg = DeclareLaunchArgument(
        'fov_degrees', default_value='60.0',
        description='전방 감시 시야각 (도): 60, 120, 180'
    )
    lidar_offset_arg = DeclareLaunchArgument(
        'lidar_offset', default_value='0.0',
        description='LiDAR 0° 보정 오프셋 (도)'
    )
    obstacle_dist_arg = DeclareLaunchArgument(
        'obstacle_dist_cm', default_value='50.0',
        description='장애물 감지 임계 거리 (cm)'
    )
    turn_sec_arg = DeclareLaunchArgument(
        'turn_10deg_sec', default_value='0.2',
        description='10도 회전 소요 시간 (초)'
    )
    back_sec_arg = DeclareLaunchArgument(
        'backward_10cm_sec', default_value='0.5',
        description='10cm 후진 소요 시간 (초)'
    )

    # ── 노드 정의 ─────────────────────────────────────────────
    
    # [수정됨] 기존 커스텀 lidar_node는 주석 처리하여 비활성화합니다.
    # lidar_node = Node(
    #     package='robot_controller',
    #     executable='lidar_node',
    #     name='lidar_node',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/ttyUSB0',
    #         'baud': 115200,
    #         'fov_degrees': LaunchConfiguration('fov_degrees'),
    #         'lidar_offset': LaunchConfiguration('lidar_offset'),
    #     }],
    # )

    # [추가됨] 공식 rplidar_ros 패키지의 안정적인 노드로 교체
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Express'  # 터미널에서 확인된 최적 모드
        }],
    )

    motor_node = Node(
        package='robot_controller',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'can_channel': 'can0',
            'can_id': 0x123,
            'max_speed': 9999,
        }],
    )

    avoidance_node = Node(
        package='robot_controller',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'obstacle_dist_cm': LaunchConfiguration('obstacle_dist_cm'),
            'turn_10deg_sec': LaunchConfiguration('turn_10deg_sec'),
            'backward_10cm_sec': LaunchConfiguration('backward_10cm_sec'),
            'forward_speed': 0.2002,
            'turn_speed': 0.2002,
        }],
    )

    return LaunchDescription([
        fov_arg,
        lidar_offset_arg,
        obstacle_dist_arg,
        turn_sec_arg,
        back_sec_arg,
        lidar_node,
        motor_node,
        avoidance_node
    ])
    
