#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray

class TagToPoseNode(Node):
    def __init__(self):
        super().__init__('tag_to_pose_node')
        
        # 1. 태그 인식 데이터 받기 (Subscribe)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)
            
        # 2. EKF로 로봇 절대 위치 쏘기 (Publish)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/vision_pose', 
            10)
            
        # =====================================================================
        # 🎯 [수정해야 할 파라미터] 지도의 태그 절대 좌표 (단위: 미터)
        # 나중에 실제 지도(map)를 그리고 나면, 각 태그가 붙은 실제 위치를 적어줍니다.
        # 사용하지 않는 태그는 0.0 으로 두면 로봇이 알아서 무시합니다!
        # =====================================================================
        self.tag_map_poses = {
            0: {'x': 0.0, 'y': 0.0},
            1: {'x': 0.0, 'y': 0.0},
            2: {'x': 1.5, 'y': 2.0},  # 예시: 2번 태그는 맵의 X:1.5m, Y:2.0m 위치에 있음
            3: {'x': 3.0, 'y': 2.0},  # 예시: 3번 태그는 맵의 X:3.0m, Y:2.0m 위치에 있음
            4: {'x': 0.0, 'y': 0.0},
            5: {'x': 0.0, 'y': 0.0},
            6: {'x': 0.0, 'y': 0.0},
            7: {'x': 0.0, 'y': 0.0},
            8: {'x': 0.0, 'y': 0.0},
            9: {'x': 0.0, 'y': 0.0}
        }
        self.get_logger().info("📷 비전 기반 EKF 좌표 변환 노드 시작됨!")

    def tag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            
            # 인식된 태그 번호가 우리 족보(0~9)에 있는지 확인
            if tag_id in self.tag_map_poses:
                map_x = self.tag_map_poses[tag_id]['x']
                map_y = self.tag_map_poses[tag_id]['y']
                
                # [안전장치] X, Y가 0.0이면 아직 설치 안 한 태그이므로 무시!
                if map_x == 0.0 and map_y == 0.0:
                    continue
                
                # 카메라 렌즈 기준 태그까지의 거리 (Z가 앞뒤 깊이, X가 좌우 거리)
                dist_forward = detection.centre.pose.pose.position.z 
                dist_lateral = detection.centre.pose.pose.position.x
                
                # 로봇의 현재 절대 위치 역산 (맵 위치 - 카메라 측정 거리)
                # (주의: 로봇에 카메라를 달 때의 각도에 따라 나중에 부호(+,-) 수정이 필요할 수 있습니다)
                robot_x = map_x - dist_forward
                robot_y = map_y - dist_lateral
                
                # EKF에 보낼 메시지 포장하기
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map" # 맵 기준 절대 좌표임을 명시
                
                pose_msg.pose.pose.position.x = robot_x
                pose_msg.pose.pose.position.y = robot_y
                
                # 데이터 신뢰도 (0.01: "이 데이터는 거의 100% 확실하니 무조건 믿어라!" 라는 뜻)
                pose_msg.pose.covariance[0] = 0.001  # X 위치 공분산
                pose_msg.pose.covariance[7] = 0.001  # Y 위치 공분산
                pose_msg.pose.covariance[35] = 0.001 # Yaw(회전) 공분산 (★추가)

                # EKF 에러 방지를 위한 기본 방향 쿼터니언 설정 (★추가)
                # (회전 데이터가 비어있으면 EKF가 수학적 오류를 일으킴)
                pose_msg.pose.pose.orientation.w = 1.0
                # EKF로 빵야!
                self.publisher.publish(pose_msg)
                self.get_logger().info(f"✅ {tag_id}번 태그 인식! 로봇 위치 전송 ➡️ (X: {robot_x:.2f}, Y: {robot_y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = TagToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()