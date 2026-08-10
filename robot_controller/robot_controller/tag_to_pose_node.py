#!/usr/bin/env python3
import math  # 🚀 [필수 추가] 삼각함수 계산을 위한 math 라이브러리
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray

class TagToPoseNode(Node):
    def __init__(self):
        super().__init__('tag_to_pose_node')
        
        # 1. 태그 인식 데이터 받기
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)
            
        # 2. EKF로 로봇 절대 위치 쏘기
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/vision_pose', 
            10)
            
        # =====================================================================
        # 🎯 [수정된 파라미터] 지도의 태그 절대 좌표 + 바라보는 방향(yaw) 추가
        # (동=0.0, 북=1.57, 서=3.14, 남=-1.57)
        # =====================================================================
        self.tag_map_poses = {
            0: {'x': -0.33, 'y': -4.36, 'yaw': -1.57},  # 교수님 방 (예시: 남쪽을 봄)
            1: {'x': -8.17, 'y': -4.37, 'yaw': 0.0},    # 사물함 지나 (예시: 동쪽을 봄)
            2: {'x': -17.30, 'y': -4.38, 'yaw': 1.57},  # 자판기 지나 (예시: 북쪽을 봄)
            3: {'x': -29.20, 'y': -4.47, 'yaw': 3.14},  # 화장실 중간 (예시: 서쪽을 봄)
            4: {'x': -39.00, 'y': -4.27, 'yaw': -1.57}, # 3140 문 옆
            5: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            6: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            7: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            8: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            9: {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        }
        self.get_logger().info("📷 비전 기반 EKF 좌표 변환 노드 시작됨!")

    def tag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            
            if tag_id in self.tag_map_poses:
                map_x = self.tag_map_poses[tag_id]['x']
                map_y = self.tag_map_poses[tag_id]['y']
                map_yaw = self.tag_map_poses[tag_id]['yaw']
                
                if map_x == 0.0 and map_y == 0.0:
                    continue
                
                # 카메라 렌즈 기준 태그까지의 거리 (광학 좌표계 기준: Z가 깊이, X가 좌우)
                dist_forward = detection.centre.pose.pose.position.z 
                dist_lateral = detection.centre.pose.pose.position.x
                
                # 🚀 2D 좌표 회전 변환 행렬 적용
                robot_x = map_x - (dist_forward * math.cos(map_yaw) - dist_lateral * math.sin(map_yaw))
                robot_y = map_y - (dist_forward * math.sin(map_yaw) + dist_lateral * math.cos(map_yaw))
                
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                
                pose_msg.pose.pose.position.x = robot_x
                pose_msg.pose.pose.position.y = robot_y
                pose_msg.pose.pose.position.z = 0.0  # 🚀 [추가] Z 고도값 무시
                
                pose_msg.pose.covariance[0] = 0.001  
                pose_msg.pose.covariance[7] = 0.001  
                pose_msg.pose.covariance[35] = 0.001 

                pose_msg.pose.pose.orientation.w = 1.0
                
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