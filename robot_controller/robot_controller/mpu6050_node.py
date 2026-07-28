"""
mpu6050_node.py
===============
ROS 2 Node: MPU6050 IMU 센서 데이터 읽기 및 /imu/data 게시 (동적 공분산 적용)

[센서 사양]
  - 칩: InvenSense MPU6050
  - 통신: I2C (기본 주소 0x68, AD0=LOW / 0x69, AD0=HIGH)
  - 가속도계 기본 범위: ±2g  → 감도 16384 LSB/g
  - 자이로스코프 기본 범위: ±250°/s → 감도 131 LSB/(°/s)
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry  # [수정됨] cmd_vel 대신 Odometry 구독을 위해 임포트

# I2C 라이브러리 (smbus2 우선, 없으면 smbus)
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


# ── MPU6050 레지스터 주소 ─────────────────────────────────────────
MPU6050_ADDR     = 0x68   # I2C 슬레이브 주소 (AD0=LOW)
REG_PWR_MGMT_1   = 0x6B   # 전원 관리 레지스터
REG_GYRO_CONFIG  = 0x1B   # 자이로 설정 레지스터
REG_ACCEL_CONFIG = 0x1C   # 가속도 설정 레지스터
REG_ACCEL_XOUT_H = 0x3B   # 가속도 X 데이터 상위바이트 (6바이트 블록)
REG_GYRO_XOUT_H  = 0x43   # 자이로 X 데이터 상위바이트 (6바이트 블록)

# ── 스케일링 상수 ─────────────────────────────────────────────────
ACCEL_SCALE_MODIFIER = 16384.0
GRAVITY_MS2          = 9.80665
GYRO_SCALE_MODIFIER  = 131.0
DEG_TO_RAD           = math.pi / 180.0
ALPHA                = 0.98

# ── 상태별 동적 노이즈 공분산 (대각 원소) ──────────────────────
# [정지 상태] 진동이 없으므로 IMU 데이터를 강하게 신뢰 (작은 값)
ORIENTATION_COV_STOP  = 1e-4
ANGULAR_VEL_COV_STOP  = 1e-5
LINEAR_ACCEL_COV_STOP = 1e-3

# [주행 상태] 메카넘 휠 진동으로 인한 노이즈 발생 -> 신뢰도 하향 (큰 값)
ORIENTATION_COV_MOVE  = 1e-2  # 주행 중 Yaw 드리프트 허용치 증가
ANGULAR_VEL_COV_MOVE  = 5e-2  # 자이로 진동 노이즈 반영
LINEAR_ACCEL_COV_MOVE = 0.5   # 가속도 진동 노이즈 대폭 반영


class MPU6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('i2c_bus',     1)
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('publish_hz',  20.0)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('alpha',       ALPHA)

        i2c_bus     = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        publish_hz  = self.get_parameter('publish_hz').value
        self._frame_id = self.get_parameter('frame_id').value
        self._alpha    = self.get_parameter('alpha').value

        # ── [수정됨] 주행 상태 추적용 변수 및 오도메트리 구독자 생성 ───
        self._is_moving = False
        self._sub_odom = self.create_subscription(
            Odometry, 
            '/odom_motor',       # 모터 노드가 계산한 실제 바퀴 속도 토픽
            self._odom_cb, 
            10
        )

        # ── I2C 버스 초기화 ───────────────────────────────────────
        try:
            self._bus     = SMBus(i2c_bus)
            self._address = i2c_address
            self._init_mpu6050()
            self.get_logger().info(
                f'MPU6050 초기화 완료 (I2C bus={i2c_bus}, addr=0x{i2c_address:02X})'
            )
        except Exception as exc:
            self.get_logger().fatal(f'MPU6050 I2C 초기화 실패: {exc}')
            raise

        # ── 상보 필터 상태 (라디안) ───────────────────────────────
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0

        # ── 이전 타임스탬프 (dt 계산용) ──────────────────────────
        self._last_time = self.get_clock().now()

        # ── IMU 게시자 ────────────────────────────────────────────
        self._pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── 20Hz 타이머 ───────────────────────────────────────────
        self._timer = self.create_timer(1.0 / publish_hz, self._timer_cb)

        self.get_logger().info(
            f'MPU6050 노드 준비 완료 '
            f'(frame_id={self._frame_id}, {publish_hz:.0f}Hz, α={self._alpha})'
        )

    # ── [수정됨] 로봇 이동 상태 판별 콜백 (Odometry 기반) ───────────
    def _odom_cb(self, msg: Odometry) -> None:
        """실제 바퀴의 오도메트리 속도를 수신하여 로봇이 이동 중인지 판단합니다."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        
        # 선속도나 각속도가 0.01 이상이면 주행 중으로 간주
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self._is_moving = True
        else:
            self._is_moving = False

    # ──────────────────────────────────────────────────────────────
    # ── MPU6050 초기화 ────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _init_mpu6050(self) -> None:
        self._bus.write_byte_data(self._address, REG_PWR_MGMT_1, 0x00)
        self._bus.write_byte_data(self._address, REG_GYRO_CONFIG, 0x00)
        self._bus.write_byte_data(self._address, REG_ACCEL_CONFIG, 0x00)

    # ──────────────────────────────────────────────────────────────
    # ── I2C 원시 데이터 읽기 ──────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _read_raw_data(self, reg: int) -> int:
        high = self._bus.read_byte_data(self._address, reg)
        low  = self._bus.read_byte_data(self._address, reg + 1)
        raw = (high << 8) | low
        if raw > 32767:
            raw -= 65536
        return raw

    def _read_accel(self):
        ax_raw = self._read_raw_data(REG_ACCEL_XOUT_H)
        ay_raw = self._read_raw_data(REG_ACCEL_XOUT_H + 2)
        az_raw = self._read_raw_data(REG_ACCEL_XOUT_H + 4)

        ax = ax_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        ay = ay_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        az = az_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        return ax, ay, az

    def _read_gyro(self):
        gx_raw = self._read_raw_data(REG_GYRO_XOUT_H)
        gy_raw = self._read_raw_data(REG_GYRO_XOUT_H + 2)
        gz_raw = self._read_raw_data(REG_GYRO_XOUT_H + 4)

        gx = gx_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gy = gy_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gz = gz_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        return gx, gy, gz

    # ──────────────────────────────────────────────────────────────
    # ── 상보 필터 ─────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _complementary_filter(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float, dt: float) -> tuple:
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        roll  = self._alpha * (self._roll  + gx * dt) + (1.0 - self._alpha) * roll_acc
        pitch = self._alpha * (self._pitch + gy * dt) + (1.0 - self._alpha) * pitch_acc
        yaw   = self._yaw + gz * dt

        self._roll  = roll
        self._pitch = pitch
        self._yaw   = yaw
        return roll, pitch, yaw

    # ──────────────────────────────────────────────────────────────
    # ── 오일러 → 쿼터니언 변환 ───────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    # ──────────────────────────────────────────────────────────────
    # ── 20Hz 타이머 콜백 ─────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _timer_cb(self) -> None:
        now       = self.get_clock().now()
        dt        = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        try:
            ax, ay, az = self._read_accel()
            gx, gy, gz = self._read_gyro()
        except Exception as exc:
            self.get_logger().error(f'I2C 읽기 오류: {exc}')
            return

        roll, pitch, yaw = self._complementary_filter(ax, ay, az, gx, gy, gz, dt)
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # ── 상태에 따른 공분산 할당 로직 ───────────────────────
        if self._is_moving:
            ori_cov = ORIENTATION_COV_MOVE
            ang_cov = ANGULAR_VEL_COV_MOVE
            acc_cov = LINEAR_ACCEL_COV_MOVE
        else:
            ori_cov = ORIENTATION_COV_STOP
            ang_cov = ANGULAR_VEL_COV_STOP
            acc_cov = LINEAR_ACCEL_COV_STOP

        _cov_orientation = [
            ori_cov, 0.0, 0.0,
            0.0, ori_cov, 0.0,
            0.0, 0.0, ori_cov,
        ]
        _cov_angular_vel = [
            ang_cov, 0.0, 0.0,
            0.0, ang_cov, 0.0,
            0.0, 0.0, ang_cov,
        ]
        _cov_linear_accel = [
            acc_cov, 0.0, 0.0,
            0.0, acc_cov, 0.0,
            0.0, 0.0, acc_cov,
        ]

        msg.orientation_covariance     = _cov_orientation
        msg.angular_velocity_covariance = _cov_angular_vel
        msg.linear_acceleration_covariance = _cov_linear_accel

        self._pub.publish(msg)

    def destroy_node(self):
        try:
            self._bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()