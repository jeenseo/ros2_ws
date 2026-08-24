#!/usr/bin/env python3
"""
lidar_check.py — 라이다 장착 방향을 '원시 데이터'로 확정
================================================================================
왜 이 도구가 필요한가
--------------------------------------------------------------------------------
지금 두 개의 관측이 서로 모순됩니다.

  (A) 직전 보고: "로봇 앞에 벽이 있는데 점군이 뒤에 찍힌다"
      -> URDF 의 rpy=pi 가 **틀렸다**는 뜻입니다.
  (B) 이번 보고: "라이다가 물리적으로 뒤통수를 보게 장착돼 있다"
      -> URDF 의 rpy=pi 가 **맞다**는 뜻입니다.

둘 다 참일 수 없습니다. 그리고 (A)를 관측하실 때 Foxglove 는
`Frame map not found` 상태였고 `/scan` 에 빨간 오류 표시가 있었습니다.
TF 체인이 끊긴 상태의 시각화는 신뢰할 수 없습니다.

이게 틀리면 **코스트맵이 앞 장애물을 뒤에 표시**합니다. 로봇이 실제 벽으로
돌진하면서 유령 장애물을 피하게 됩니다. 확인 비용은 3분, 틀렸을 때 비용은 충돌입니다.

--------------------------------------------------------------------------------
절차
--------------------------------------------------------------------------------
 1. 로봇 **정면 1 m 앞에만** 상자나 벽 같은 것을 두십시오.
    좌/우/뒤는 최소 3 m 이상 비워야 판정이 깨끗합니다. (복도면 복도 끝에서)
 2. 센서만 띄웁니다:
       ros2 launch robot_controller nav2.launch.py use_nav2:=false
 3. 다른 터미널에서:
       ros2 run robot_controller lidar_check
 4. 출력의 마지막 [판정] 줄을 보시면 끝입니다.

TF 를 쓰지만 필요한 것은 base_link <- lidar_link 하나뿐이며, 이건
robot_state_publisher 만 떠 있으면 EKF/AMCL 없이도 나옵니다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
import tf2_ros


_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                  reliability=ReliabilityPolicy.BEST_EFFORT)


def norm(a):
    """(-pi, pi] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class LidarCheck(Node):

    def __init__(self):
        super().__init__('lidar_check')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('samples', 5)       # 이만큼 모아 중앙값으로 판정

        self._base = self.get_parameter('base_frame').value
        self._need = int(self.get_parameter('samples').value)
        self._buf = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self._buf, self)
        self._yaw = None
        self._hits = []
        self._n = 0

        self.create_subscription(LaserScan,
                                 self.get_parameter('scan_topic').value,
                                 self._cb, _QOS)
        print('\n라이다 방향 검사 — 정면 1 m 앞에만 물체를 두십시오.')
        print('스캔 대기 중...\n')

    # ------------------------------------------------------------------ #
    def _lookup_yaw(self, laser_frame):
        """base_frame <- laser_frame 의 yaw [rad]."""
        try:
            tf = self._buf.lookup_transform(
                self._base, laser_frame, rclpy.time.Time())
        except Exception as e:
            return None, str(e)
        q = tf.transform.rotation
        # 쿼터니언 -> yaw (2D 가정)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw, None

    # ------------------------------------------------------------------ #
    def _cb(self, msg: LaserScan):
        if self._yaw is None:
            self._yaw, err = self._lookup_yaw(msg.header.frame_id)
            if self._yaw is None:
                self.get_logger().warn(
                    f'TF {self._base} <- {msg.header.frame_id} 대기 중... ({err})',
                    throttle_duration_sec=2.0)
                return
            print(f'스캔 프레임 : {msg.header.frame_id}')
            print(f'TF yaw      : {self._yaw:+.5f} rad = {math.degrees(self._yaw):+.1f}°'
                  f'   ({self._base} <- {msg.header.frame_id})')
            print(f'각도 범위   : {math.degrees(msg.angle_min):+.1f}° ~ '
                  f'{math.degrees(msg.angle_max):+.1f}°, '
                  f'증분 {math.degrees(msg.angle_increment):.3f}°, '
                  f'빔 {len(msg.ranges)}개\n')

        # ── 가장 가까운 유효 반사 ────────────────────────────────────
        best_i, best_r = -1, float('inf')
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min < r < msg.range_max and r < best_r:
                best_r, best_i = r, i
        if best_i < 0:
            return

        a_laser = msg.angle_min + best_i * msg.angle_increment
        a_base = norm(a_laser + self._yaw)      # 2D 이므로 yaw 만 더하면 됩니다
        self._hits.append((best_r, a_laser, a_base))
        self._n += 1
        print(f'  [{self._n}/{self._need}] 최근접 {best_r:.3f} m  '
              f'라이다프레임 {math.degrees(a_laser):+7.1f}°  '
              f'-> {self._base} 프레임 {math.degrees(a_base):+7.1f}°')

        if self._n >= self._need:
            self._verdict()
            rclpy.shutdown()

    # ------------------------------------------------------------------ #
    def _verdict(self):
        # 각도 중앙값 (원형 평균으로)
        sx = sum(math.cos(h[2]) for h in self._hits)
        sy = sum(math.sin(h[2]) for h in self._hits)
        a_base = math.atan2(sy, sx)
        sx = sum(math.cos(h[1]) for h in self._hits)
        sy = sum(math.sin(h[1]) for h in self._hits)
        a_laser = math.atan2(sy, sx)
        d = math.degrees(a_base)

        print('\n' + '=' * 70)
        print(f'  물체의 방위각')
        print(f'    라이다 프레임 : {math.degrees(a_laser):+7.1f}°')
        print(f'    {self._base:12} : {d:+7.1f}°   '
              f'(0=앞, +90=왼쪽, 180=뒤, -90=오른쪽)')
        print('=' * 70)

        if abs(d) <= 30.0:
            print(f'  [판정] ★ 정상. 정면 물체가 {self._base} 기준 앞(+X)에 있습니다.')
            print(f'         URDF 의 lidar_joint rpy 를 그대로 두십시오.')
        elif abs(abs(d) - 180.0) <= 30.0:
            print(f'  [판정] ★★ 180° 오류. 정면 물체가 **뒤**에 찍힙니다.')
            print(f'         URDF lidar_joint 의 rpy yaw 를 현재값에서 pi 만큼')
            print(f'         더하거나 빼십시오 (3.14159 <-> 0).')
            print(f'         이대로 두면 코스트맵이 앞 장애물을 뒤에 표시하여')
            print(f'         로봇이 실제 벽으로 돌진합니다.')
        elif 60.0 <= abs(d) <= 120.0:
            print(f'  [판정] ★ 약 90° 오류. rpy yaw 를 {-d:+.0f}° '
                  f'({math.radians(-d):+.5f} rad) 만큼 보정하십시오.')
        else:
            print(f'  [판정] {d:+.1f}° 어긋나 있습니다. '
                  f'rpy yaw 를 {-d:+.1f}° 보정하십시오.')
            print(f'         (물체가 정말 정면에만 있었는지 먼저 확인하십시오)')
        print()
        print('  ※ 앞뒤가 맞는데 좌/우만 반대라면 이건 회전이 아니라 "거울" 입니다.')
        print('    그때만 sllidar 의 inverted 파라미터를 쓰십시오. URDF 가 아닙니다.')
        print('    확인법: 물체를 **왼쪽에만** 두고 다시 실행 -> +90° 가 나와야 정상.')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = LidarCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()