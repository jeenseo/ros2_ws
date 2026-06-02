"""
lidar_node.py
=============
ROS 2 Node: RPLIDAR A1 스캔 게시자

수정 사항:
  - max_buf_meas 500 → 20: 버퍼 오버플로 방지
  - 타임스탬프를 publish 직전에 캡처: TF 캐시 미스매치 방지
  - frame_id = 'lidar_link': TF에서 180° 회전 처리 (base_link→lidar_link)
  - Auto-Recovery 유지
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import serial
from rplidar import RPLidar


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('fov_degrees',  60.0)
        self.declare_parameter('lidar_offset', 0.0)

        self._port        = self.get_parameter('port').value
        self._baud        = self.get_parameter('baud').value
        self._fov_degrees = self.get_parameter('fov_degrees').value
        self._offset      = self.get_parameter('lidar_offset').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._dist_pub = self.create_publisher(Float32, '/forward_distance', 10)

        # ── LiDAR 초기화 + 스캔 스레드 시작 ─────────────────────
        self._lidar = self._init_lidar()
        scan_t = threading.Thread(target=self._scan_loop, daemon=True)
        scan_t.start()

        self.get_logger().info(
            f'LidarNode ready  port={self._port} '
            f'FOV=±{self._fov_degrees/2:.0f}°  offset={self._offset:.0f}°'
        )

    # ── LiDAR 초기화 ─────────────────────────────────────────────
    def _init_lidar(self) -> RPLidar | None:
        try:
            # Step 1: DTR 신호 리셋 (junk 데이터 제거)
            self.get_logger().info('[LIDAR] DTR/RTS 리셋 중...')
            s = serial.Serial(self._port, self._baud, timeout=1)
            s.setDTR(False)
            time.sleep(0.1)
            s.reset_input_buffer()
            s.setDTR(True)
            s.close()
            time.sleep(0.5)

            # Step 2: 소프트웨어 리셋
            lidar = RPLidar(self._port, baudrate=self._baud)
            lidar.reset()
            time.sleep(2.0)

            # Step 3: 버퍼 플러시
            lidar._serial.reset_input_buffer()
            try:
                lidar.clean_input()
            except Exception:
                pass

            # Step 4: 모터 시작
            lidar.start_motor()
            self.get_logger().info('[LIDAR] 모터 시작 완료')
            return lidar

        except Exception as exc:
            self.get_logger().error(f'[LIDAR] 초기화 실패: {exc}')
            return None

    # ── 스캔 루프 (백그라운드 스레드, Auto-Recovery 포함) ────────
    def _scan_loop(self) -> None:
        half_fov = self._fov_degrees / 2.0
        NUM_BINS = 360

        while rclpy.ok():
            # 센서 미연결 시 재연결 대기
            if self._lidar is None:
                self.get_logger().warn('[LIDAR] 미연결 — 3초 후 재연결 시도...')
                time.sleep(3.0)
                self._lidar = self._init_lidar()
                continue

            try:
                self.get_logger().info('[LIDAR] 스캐닝 시작')

                # ─── [핵심] max_buf_meas=20 — 버퍼 오버플로 방지 ──
                for scan in self._lidar.iter_scans(max_buf_meas=20):

                    # LaserScan 메시지 구조 설정
                    ls = LaserScan()
                    ls.header.frame_id  = 'lidar_link'   # TF에서 180° 회전 처리
                    ls.angle_min        = 0.0
                    ls.angle_max        = 2.0 * math.pi
                    ls.angle_increment  = (2.0 * math.pi) / NUM_BINS
                    ls.time_increment   = 0.0
                    ls.scan_time        = 0.1
                    ls.range_min        = 0.15
                    ls.range_max        = 12.0
                    ls.ranges           = [0.0] * NUM_BINS

                    min_cm = float('inf')

                    # 스캔 데이터 처리
                    for (quality, angle_raw, dist_mm) in scan:
                        if quality == 0 or dist_mm == 0:
                            continue

                        dist_m = dist_mm / 1000.0
                        idx = int(angle_raw) % NUM_BINS
                        if ls.ranges[idx] == 0.0 or dist_m < ls.ranges[idx]:
                            ls.ranges[idx] = dist_m

                        # 전방 FOV 필터
                        adj = (angle_raw - self._offset) % 360.0
                        if adj > 180.0:
                            adj -= 360.0
                        if -half_fov <= adj <= half_fov:
                            cm = dist_mm / 10.0
                            if cm < min_cm:
                                min_cm = cm

                    # ─── [핵심] 타임스탬프를 publish 직전에 캡처 ──
                    # iter_scans 완료 후 캡처 → TF 캐시 미스매치 방지
                    ls.header.stamp = self.get_clock().now().to_msg()

                    # 게시
                    self._scan_pub.publish(ls)

                    dist_msg      = Float32()
                    dist_msg.data = float(min_cm) if min_cm != float('inf') else -1.0
                    self._dist_pub.publish(dist_msg)

            except Exception as exc:
                self.get_logger().error(f'[LIDAR] 스캔 오류: {exc}')
                self.get_logger().info('[LIDAR] Auto-Recovery: 2초 후 재부팅...')
                try:
                    self._lidar.stop()
                    self._lidar.stop_motor()
                    self._lidar.disconnect()
                except Exception:
                    pass
                self._lidar = None
                time.sleep(2.0)

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
