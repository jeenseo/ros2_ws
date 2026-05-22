"""
mpu6050_node.py
===============
ROS 2 Node: MPU6050 IMU 센서 데이터 읽기 및 /imu/data 게시

[센서 사양]
  - 칩: InvenSense MPU6050
  - 통신: I2C (기본 주소 0x68, AD0=LOW / 0x69, AD0=HIGH)
  - 가속도계 기본 범위: ±2g  → 감도 16384 LSB/g
  - 자이로스코프 기본 범위: ±250°/s → 감도 131 LSB/(°/s)

[I2C 레지스터 맵]
  0x6B  PWR_MGMT_1       슬립 모드 제어 (0x00 기록 → 정상 동작)
  0x1B  GYRO_CONFIG      자이로 범위 설정 (0x00 = ±250°/s)
  0x1C  ACCEL_CONFIG     가속도 범위 설정 (0x00 = ±2g)
  0x3B  ACCEL_XOUT_H     가속도 X 상위바이트 시작 (6바이트 연속)
  0x43  GYRO_XOUT_H      자이로 X 상위바이트 시작 (6바이트 연속)

[알고리즘]
  상보 필터 (Complementary Filter):
    - 자이로: 단기 정밀도 높음, 장기 누적 드리프트 있음
    - 가속도계: 장기 안정적, 단기 노이즈 큼
    - α=0.98: 자이로 98% + 가속도 2% 융합

[토픽]
  게시: /imu/data (sensor_msgs/msg/Imu) @ 20Hz
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

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
# 가속도계: ±2g 범위 → 감도 16384 LSB/g
# 단위 변환: g → m/s²  (1g = 9.80665 m/s²)
ACCEL_SCALE_MODIFIER = 16384.0
GRAVITY_MS2          = 9.80665

# 자이로스코프: ±250°/s 범위 → 감도 131 LSB/(°/s)
# 단위 변환: °/s → rad/s  (1° = π/180 rad)
GYRO_SCALE_MODIFIER  = 131.0
DEG_TO_RAD           = math.pi / 180.0

# ── 상보 필터 계수 ────────────────────────────────────────────────
# α = 0.98: 자이로 98% + 가속도계 2%
# α가 클수록 자이로 의존도 ↑ (단기 정밀), 작을수록 가속도 의존도 ↑ (장기 안정)
ALPHA = 0.98

# ── IMU 노이즈 공분산 (대각 원소) ────────────────────────────────
# 실제 센서 노이즈에 맞게 조정 필요. MPU6050 데이터시트 기준 초기값.
ORIENTATION_COV     = 1e-5
ANGULAR_VEL_COV     = 1e-5
LINEAR_ACCEL_COV    = 1e-3


class MPU6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('i2c_bus',     1)           # 라즈베리 파이 I2C 버스 번호
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('publish_hz',  20.0)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('alpha',       ALPHA)       # 상보 필터 계수

        i2c_bus     = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        publish_hz  = self.get_parameter('publish_hz').value
        self._frame_id = self.get_parameter('frame_id').value
        self._alpha    = self.get_parameter('alpha').value

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

    # ──────────────────────────────────────────────────────────────
    # ── MPU6050 초기화 ────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _init_mpu6050(self) -> None:
        """
        MPU6050 초기 설정:
          1. 슬립 해제 (PWR_MGMT_1 = 0x00)
          2. 자이로 범위 ±250°/s (GYRO_CONFIG = 0x00)
          3. 가속도 범위 ±2g    (ACCEL_CONFIG = 0x00)
        """
        # 슬립 모드 해제: PWR_MGMT_1 레지스터에 0x00 기록
        self._bus.write_byte_data(self._address, REG_PWR_MGMT_1, 0x00)

        # 자이로 범위 설정: GYRO_CONFIG[4:3] = 00 → ±250°/s (감도 131 LSB/(°/s))
        self._bus.write_byte_data(self._address, REG_GYRO_CONFIG, 0x00)

        # 가속도 범위 설정: ACCEL_CONFIG[4:3] = 00 → ±2g (감도 16384 LSB/g)
        self._bus.write_byte_data(self._address, REG_ACCEL_CONFIG, 0x00)

    # ──────────────────────────────────────────────────────────────
    # ── I2C 원시 데이터 읽기 ──────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _read_raw_data(self, reg: int) -> int:
        """
        지정 레지스터부터 2바이트 읽어 부호 있는 16비트 정수 반환.
        MPU6050은 빅엔디안: 상위바이트(reg) + 하위바이트(reg+1)
        """
        high = self._bus.read_byte_data(self._address, reg)
        low  = self._bus.read_byte_data(self._address, reg + 1)

        # 상위바이트 << 8 | 하위바이트 → unsigned 16bit
        raw = (high << 8) | low

        # 부호 있는 16비트로 변환 (two's complement)
        # 32768 이상이면 음수 영역 (65536 - raw)
        if raw > 32767:
            raw -= 65536
        return raw

    def _read_accel(self):
        """
        가속도계 원시값 읽기 → m/s² 변환.
        Returns: (ax, ay, az) in m/s²
        """
        ax_raw = self._read_raw_data(REG_ACCEL_XOUT_H)
        ay_raw = self._read_raw_data(REG_ACCEL_XOUT_H + 2)
        az_raw = self._read_raw_data(REG_ACCEL_XOUT_H + 4)

        # 변환: raw / 감도(LSB/g) × 중력가속도(m/s²)
        ax = ax_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        ay = ay_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        az = az_raw / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        return ax, ay, az

    def _read_gyro(self):
        """
        자이로스코프 원시값 읽기 → rad/s 변환.
        Returns: (gx, gy, gz) in rad/s
        """
        gx_raw = self._read_raw_data(REG_GYRO_XOUT_H)
        gy_raw = self._read_raw_data(REG_GYRO_XOUT_H + 2)
        gz_raw = self._read_raw_data(REG_GYRO_XOUT_H + 4)

        # 변환: raw / 감도(LSB/(°/s)) × (π/180) → rad/s
        gx = gx_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gy = gy_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gz = gz_raw / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        return gx, gy, gz

    # ──────────────────────────────────────────────────────────────
    # ── 상보 필터 ─────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def _complementary_filter(
        self,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
        dt: float,
    ) -> tuple:
        """
        상보 필터로 Roll/Pitch/Yaw(라디안) 갱신.

        [가속도계로 Roll/Pitch 계산]
          roll_acc  = atan2(ay, az)
            → Y축 가속도와 Z축 가속도의 비율로 X축 기울기(roll) 계산
          pitch_acc = atan2(-ax, sqrt(ay² + az²))
            → X축 가속도를 Y,Z 합력과 비교하여 Y축 기울기(pitch) 계산

        [상보 필터 융합]
          roll  = α × (roll_prev  + gx×dt) + (1-α) × roll_acc
          pitch = α × (pitch_prev + gy×dt) + (1-α) × pitch_acc
          yaw   = yaw_prev + gz×dt   (자기계 없으므로 자이로만 사용)

        Returns: (roll, pitch, yaw) in radians
        """
        # ── 가속도계 기반 각도 (중력 벡터 분해) ──────────────────
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # ── 상보 필터: 자이로 통합 + 가속도 보정 ─────────────────
        roll  = self._alpha * (self._roll  + gx * dt) + (1.0 - self._alpha) * roll_acc
        pitch = self._alpha * (self._pitch + gy * dt) + (1.0 - self._alpha) * pitch_acc
        yaw   = self._yaw + gz * dt   # 자이로 적분만 사용 (누적 드리프트 주의)

        self._roll  = roll
        self._pitch = pitch
        self._yaw   = yaw
        return roll, pitch, yaw

    # ──────────────────────────────────────────────────────────────
    # ── 오일러 → 쿼터니언 변환 ───────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
        """
        ZYX 오일러 각 (Yaw-Pitch-Roll 순서) → 단위 쿼터니언 (x, y, z, w).

        [반각 삼각함수]
          cr = cos(roll/2),  sr = sin(roll/2)
          cp = cos(pitch/2), sp = sin(pitch/2)
          cy = cos(yaw/2),   sy = sin(yaw/2)

        [쿼터니언 곱셈 전개]
          q = q_yaw ⊗ q_pitch ⊗ q_roll
          qw = cr×cp×cy + sr×sp×sy
          qx = sr×cp×cy - cr×sp×sy
          qy = cr×sp×cy + sr×cp×sy
          qz = cr×cp×sy - sr×sp×cy

        Returns: (qx, qy, qz, qw)
        """
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
        """MPU6050 데이터 읽기 → 필터 → /imu/data 게시."""
        # ── dt 계산 ───────────────────────────────────────────────
        now       = self.get_clock().now()
        dt        = (now - self._last_time).nanoseconds * 1e-9   # 초 단위
        self._last_time = now

        # dt가 비정상적으로 크거나 0이면 스킵 (초기화 직후 보호)
        if dt <= 0.0 or dt > 1.0:
            return

        # ── I2C 데이터 읽기 ───────────────────────────────────────
        try:
            ax, ay, az = self._read_accel()
            gx, gy, gz = self._read_gyro()
        except Exception as exc:
            self.get_logger().error(f'I2C 읽기 오류: {exc}')
            return

        # ── 상보 필터 → Roll/Pitch/Yaw (rad) ─────────────────────
        roll, pitch, yaw = self._complementary_filter(ax, ay, az, gx, gy, gz, dt)

        # ── 오일러 → 쿼터니언 ────────────────────────────────────
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

        # ── IMU 메시지 구성 ───────────────────────────────────────
        msg = Imu()

        # Header
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self._frame_id

        # Orientation (쿼터니언)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Angular Velocity (rad/s) — 자이로 측정값
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Linear Acceleration (m/s²) — 가속도계 측정값
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # ── 공분산 행렬 설정 (3×3 → 9원소 리스트) ────────────────
        # 형식: [σxx, σxy, σxz,
        #        σyx, σyy, σyz,
        #        σzx, σzy, σzz]
        # 대각 원소(인덱스 0,4,8)만 노이즈 상수 설정, 나머지 0.0
        _cov_orientation = [
            ORIENTATION_COV, 0.0, 0.0,
            0.0, ORIENTATION_COV, 0.0,
            0.0, 0.0, ORIENTATION_COV,
        ]
        _cov_angular_vel = [
            ANGULAR_VEL_COV, 0.0, 0.0,
            0.0, ANGULAR_VEL_COV, 0.0,
            0.0, 0.0, ANGULAR_VEL_COV,
        ]
        _cov_linear_accel = [
            LINEAR_ACCEL_COV, 0.0, 0.0,
            0.0, LINEAR_ACCEL_COV, 0.0,
            0.0, 0.0, LINEAR_ACCEL_COV,
        ]

        msg.orientation_covariance     = _cov_orientation
        msg.angular_velocity_covariance = _cov_angular_vel
        msg.linear_acceleration_covariance = _cov_linear_accel

        self._pub.publish(msg)

        self.get_logger().debug(
            f'IMU | roll={math.degrees(roll):.1f}° '
            f'pitch={math.degrees(pitch):.1f}° '
            f'yaw={math.degrees(yaw):.1f}°'
        )

    # ── 노드 소멸 ─────────────────────────────────────────────────
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
