#!/usr/bin/env python3
"""
mpu6050_node.py  —  v2 (노이즈 내성 강화판)
================================================================================
ROS 2 Node: MPU6050 IMU 드라이버

[v2 변경 요약]
  1. ★ I2C 버스트 읽기 — 트랜잭션 12회 → 1회
     기존: 16비트 값 하나를 read_byte_data 2회로 나눠 읽음 (6축 = 12 트랜잭션)
       - 노이즈에 노출되는 횟수가 12배
       - 두 읽기 사이에 센서가 갱신되면 상/하위 바이트가 서로 다른 샘플에서 와서
         '찢어진(torn) 값'이 됨 → 간헐적 대형 스파이크의 정체
     변경: read_i2c_block_data(0x3B, 14) 한 번으로 ACCEL+TEMP+GYRO 전부 읽음
       - MPU6050 은 버스트 읽기 중 출력 레지스터를 갱신하지 않으므로 원자성 보장

  2. ★ DLPF(디지털 저역통과) 설정 — 앨리어싱 차단
     기존: REG 0x1A 를 아예 안 건드림 → DLPF_CFG=0 → 대역폭 260 Hz, 내부 8 kHz
       20~50 Hz 로 폴링하면 나이퀴스트(폴링/2) 를 한참 넘는 성분이 전부 접혀 들어옴
       메카넘 롤러 진동(수백 Hz)이 저주파 신호로 둔갑 → EKF 로는 절대 제거 불가
     변경: DLPF_CFG=3 (42 Hz), SMPLRT_DIV=4 (200 Hz 샘플)
       - 42 Hz < 200/2 이므로 내부적으로도 앨리어싱 없음
       - 로봇 회전 대역(~2 Hz)보다 훨씬 위라 신호 손실 없음

  3. ★ I2C 버스 복구 루틴 — 먹통을 '복구 가능한 결함'으로 강등
     슬레이브가 전송 도중 글리치를 맞으면 SDA 를 LOW 로 잡은 채 멈춥니다.
     SCL 을 9번 쳐주면 바이트를 마치고 버스를 놓습니다.

  4. ★ 글리치 카운터 — 이게 없으면 3번은 문제를 '숨기는' 코드가 됩니다
     복구 횟수를 반드시 남겨 배선 개선 전후를 정량 비교하십시오.

  5. PWR_MGMT_1 = 0x01 (자이로 X PLL). 내부 RC(0x00)보다 온도 안정성이 좋습니다.

[주의] 자이로 바이어스 보정은 여기서 하지 않습니다.
  imu_gyro_bias_node.py 가 /imu/data → /imu/data_unbiased 로 담당합니다.
  두 곳에서 빼면 이중 보정이 됩니다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

# ── MPU6050 레지스터 ──────────────────────────────────────────────
MPU6050_ADDR     = 0x68
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A   # ★ DLPF_CFG (기존 코드가 건드리지 않던 레지스터)
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B   # 여기부터 14바이트 = ACCEL(6) TEMP(2) GYRO(6)
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

BURST_START = REG_ACCEL_XOUT_H
BURST_LEN   = 14

# ── 스케일링 상수 ─────────────────────────────────────────────────
ACCEL_SCALE_MODIFIER = 16384.0     # ±2g
GRAVITY_MS2          = 9.80665
GYRO_SCALE_MODIFIER  = 131.0       # ±250 dps
DEG_TO_RAD           = math.pi / 180.0
ALPHA                = 0.98

# ── 상태별 동적 노이즈 공분산 (기본값, 파라미터로 덮어쓸 수 있음) ──
ORIENTATION_COV_STOP  = 1e-4
ANGULAR_VEL_COV_STOP  = 1e-5
LINEAR_ACCEL_COV_STOP = 1e-3
ORIENTATION_COV_MOVE  = 1e-2
ANGULAR_VEL_COV_MOVE  = 5e-2
LINEAR_ACCEL_COV_MOVE = 0.5


# ═══════════════════════════════════════════════════════════════════
#  I2C 버스 복구 (GPIO 비트뱅잉)
# ═══════════════════════════════════════════════════════════════════

class I2CRecovery:
    """
    걸린 I2C 슬레이브를 SCL 9펄스 + STOP 으로 풀어준다.

    ▣ 왜 9펄스인가
      I2C 슬레이브가 데이터 바이트를 송신하던 중 마스터가 사라지면, 슬레이브는
      남은 비트를 내보내려고 SDA 를 잡은 채 SCL 을 기다립니다. 최대 8비트 +
      ACK 1비트 = 9클럭이면 어떤 상태에서든 바이트를 끝내고 버스를 놓습니다.

    ▣ 왜 GPIO 를 직접 두드려야 하는가
      I2C 컨트롤러는 SDA 가 LOW 로 잡혀 있으면 START 를 낼 수 없습니다.
      즉 정상 I2C API 로는 탈출이 불가능하고, 핀을 GPIO 로 되돌려 직접
      클럭을 만들어야 합니다.

    ▣ 백엔드
      Pi 5 는 RPi.GPIO 가 동작하지 않으므로 lgpio 를 우선 시도합니다.
      아무 백엔드도 없으면 복구를 비활성화하고 경고만 남깁니다(노드는 계속 동작).
    """

    def __init__(self, logger, sda_pin: int, scl_pin: int, gpio_chip: int = 4):
        self._log = logger
        self._sda, self._scl = sda_pin, scl_pin
        self._chip_num = gpio_chip
        self._backend = None
        self._h = None

        try:
            import lgpio                                   # noqa: F401
            self._lgpio = lgpio
            self._backend = 'lgpio'
        except Exception:
            try:
                import RPi.GPIO as GPIO                    # noqa: F401
                self._GPIO = GPIO
                self._backend = 'rpigpio'
            except Exception:
                self._backend = None

        if self._backend is None:
            self._log.warn(
                '[I2C복구] lgpio / RPi.GPIO 를 모두 찾지 못했습니다. '
                '버스 복구가 비활성화됩니다. (sudo apt install python3-lgpio)')
        else:
            self._log.info(f'[I2C복구] 백엔드 = {self._backend}, '
                           f'SDA=GPIO{self._sda}, SCL=GPIO{self._scl}')

    @property
    def available(self) -> bool:
        return self._backend is not None

    def recover(self) -> bool:
        """SMBus 핸들을 '닫은 뒤' 호출해야 합니다 (핀 소유권 충돌 방지)."""
        if not self.available:
            return False
        try:
            if self._backend == 'lgpio':
                return self._recover_lgpio()
            return self._recover_rpigpio()
        except Exception as exc:
            self._log.error(f'[I2C복구] 실패: {exc}')
            return False

    # ------------------------------------------------------------------ #
    def _recover_lgpio(self) -> bool:
        lg = self._lgpio
        h = lg.gpiochip_open(self._chip_num)
        try:
            lg.gpio_claim_output(h, self._scl, 1)
            lg.gpio_claim_input(h, self._sda)
            released = False
            for _ in range(9):
                lg.gpio_write(h, self._scl, 0); time.sleep(5e-6)
                lg.gpio_write(h, self._scl, 1); time.sleep(5e-6)
                if lg.gpio_read(h, self._sda) == 1:
                    released = True
                    break
            # STOP 조건: SCL HIGH 인 동안 SDA 를 LOW → HIGH
            lg.gpio_claim_output(h, self._sda, 0); time.sleep(5e-6)
            lg.gpio_write(h, self._scl, 1);        time.sleep(5e-6)
            lg.gpio_write(h, self._sda, 1);        time.sleep(5e-6)
            lg.gpio_free(h, self._sda)
            lg.gpio_free(h, self._scl)
            return released
        finally:
            lg.gpiochip_close(h)

    def _recover_rpigpio(self) -> bool:
        G = self._GPIO
        G.setmode(G.BCM)
        G.setwarnings(False)
        G.setup(self._scl, G.OUT, initial=G.HIGH)
        G.setup(self._sda, G.IN)
        released = False
        for _ in range(9):
            G.output(self._scl, 0); time.sleep(5e-6)
            G.output(self._scl, 1); time.sleep(5e-6)
            if G.input(self._sda):
                released = True
                break
        G.setup(self._sda, G.OUT); G.output(self._sda, 0); time.sleep(5e-6)
        G.output(self._scl, 1);                            time.sleep(5e-6)
        G.output(self._sda, 1);                            time.sleep(5e-6)
        G.cleanup([self._sda, self._scl])
        return released


# ═══════════════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════════════

class MPU6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus',     0)          # 사용자 환경은 i2c-0
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('publish_hz',  50.0)       # 20 → 50 (버스트 읽기로 가능)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('alpha',       ALPHA)

        # ── DLPF / 샘플레이트 ─────────────────────────────────────
        #  DLPF_CFG: 0=260Hz 1=184 2=94 3=42 4=20 5=10 6=5
        #  Sample Rate = 1kHz / (1 + SMPLRT_DIV)   (DLPF 활성 시)
        #
        #  ★ 반드시 지켜야 할 부등식 두 개
        #     (a) DLPF 대역폭 < 내부 샘플레이트 / 2   ← 센서 내부 앨리어싱 방지
        #     (b) DLPF 대역폭 < publish_hz / 2       ← ROS 발행 단계 앨리어싱 방지
        #
        #  (b)를 놓치기 쉽습니다. 센서가 42 Hz 까지 통과시키는데 50 Hz 로 폴링하면
        #  나이퀴스트가 25 Hz 라, 25~42 Hz 성분이 저주파로 접혀 들어옵니다.
        #  DLPF 를 넣고도 앨리어싱이 남는 전형적인 실수입니다.
        #
        #  기본값: DLPF_CFG=4 (20 Hz) + publish 50 Hz -> 20 < 25  OK
        #  로봇 요 회전 대역은 ~2 Hz 수준이라 20 Hz 로도 신호 손실이 없습니다.
        #  발행을 100 Hz 로 올린다면 DLPF_CFG=3 (42 Hz) 로 넓혀도 됩니다.
        self.declare_parameter('dlpf_cfg',    4)          # 20 Hz
        self.declare_parameter('smplrt_div',  4)          # 1000/(1+4) = 200 Hz

        # ── I2C 복구 ─────────────────────────────────────────────
        self.declare_parameter('enable_i2c_recovery', True)
        self.declare_parameter('recovery_sda_gpio',   0)  # ★ 환경에 맞게 반드시 확인
        self.declare_parameter('recovery_scl_gpio',   1)
        self.declare_parameter('recovery_gpio_chip',  4)  # Pi 5 = 4, Pi 4 이하 = 0
        self.declare_parameter('fail_streak_to_recover', 3)

        # ── 축 부호 (IMU 회전 장착 대응) ──────────────────────────
        self.declare_parameter('gyro_sign_z',  1.0)
        self.declare_parameter('accel_sign_x', 1.0)

        # ── 주행 판정 워치독 ──────────────────────────────────────
        self.declare_parameter('odom_timeout', 0.5)

        gp = self.get_parameter
        i2c_bus_num  = gp('i2c_bus').value
        self._address = gp('i2c_address').value
        publish_hz   = float(gp('publish_hz').value)
        self._frame_id = gp('frame_id').value
        self._alpha    = float(gp('alpha').value)
        self._dlpf     = int(gp('dlpf_cfg').value)
        self._smplrt   = int(gp('smplrt_div').value)
        self._gz_sign  = float(gp('gyro_sign_z').value)
        self._ax_sign  = float(gp('accel_sign_x').value)
        self._odom_timeout = float(gp('odom_timeout').value)
        self._fail_streak_limit = int(gp('fail_streak_to_recover').value)
        self._i2c_bus_num = i2c_bus_num

        # ── 상태 ──────────────────────────────────────────────────
        self._is_moving = False
        self._last_odom_time = 0.0
        self._roll = self._pitch = self._yaw = 0.0
        self._last_time = self.get_clock().now()

        # ★ 글리치 통계 — 이것을 남기지 않으면 복구 루틴은 문제를 '숨기는' 코드가 됩니다
        self._glitch_count   = 0     # I2C 읽기 실패 누적
        self._recover_count  = 0     # 복구 시도 누적
        self._recover_ok     = 0     # 복구 성공 누적
        self._fail_streak    = 0
        self._torn_suspect   = 0     # 물리적으로 불가능한 값 폐기 누적

        # ── I2C ───────────────────────────────────────────────────
        try:
            self._bus = SMBus(i2c_bus_num)
            self._init_mpu6050()
        except Exception as exc:
            self.get_logger().fatal(f'MPU6050 I2C 초기화 실패: {exc}')
            raise

        self._recovery = None
        if bool(gp('enable_i2c_recovery').value):
            self._recovery = I2CRecovery(
                self.get_logger(),
                sda_pin=int(gp('recovery_sda_gpio').value),
                scl_pin=int(gp('recovery_scl_gpio').value),
                gpio_chip=int(gp('recovery_gpio_chip').value))

        # ── 토픽 ──────────────────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, '/odom_motor', self._odom_cb, 10)
        self._pub      = self.create_publisher(Imu, '/imu/data', 10)
        self._pub_diag = self.create_publisher(Float32MultiArray, '~/i2c_health', 10)

        self._timer = self.create_timer(1.0 / publish_hz, self._timer_cb)
        self._diag_timer = self.create_timer(1.0, self._publish_health)

        # ★ 설정 자기검증 — 앨리어싱 부등식을 실제로 만족하는지 부팅 시 확인
        _DLPF_BW = {0: 260.0, 1: 184.0, 2: 94.0, 3: 42.0, 4: 20.0, 5: 10.0, 6: 5.0}
        bw = _DLPF_BW.get(self._dlpf, 260.0)
        internal_sr = 1000.0 / (1 + self._smplrt)
        if bw >= internal_sr / 2.0:
            self.get_logger().warn(
                f'[설정] DLPF 대역폭 {bw:.0f}Hz >= 내부 샘플레이트/2 '
                f'({internal_sr/2:.0f}Hz). 센서 내부에서 앨리어싱이 발생합니다. '
                f'smplrt_div 를 줄이거나 dlpf_cfg 를 키우십시오.')
        if bw >= publish_hz / 2.0:
            self.get_logger().warn(
                f'[설정] DLPF 대역폭 {bw:.0f}Hz >= publish_hz/2 ({publish_hz/2:.0f}Hz). '
                f'ROS 발행 단계에서 앨리어싱이 남습니다. '
                f'publish_hz 를 {bw*2:.0f} 이상으로 올리거나 dlpf_cfg 를 더 낮추십시오.')

        self.get_logger().info(
            f'MPU6050 v2 준비 완료 — i2c-{i2c_bus_num} 0x{self._address:02X}, '
            f'{publish_hz:.0f} Hz\n'
            f'  DLPF_CFG={self._dlpf} ({bw:.0f}Hz), '
            f'SMPLRT_DIV={self._smplrt} ({internal_sr:.0f}Hz 내부 샘플)\n'
            f'  버스트 읽기 14B x1 (기존 2B x12 대비 트랜잭션 1/12)\n'
            f'  I2C 복구 = {"활성" if (self._recovery and self._recovery.available) else "비활성"}')

    # ══════════════════════════════════════════════════════════════
    # 초기화
    # ══════════════════════════════════════════════════════════════

    def _init_mpu6050(self) -> None:
        # PWR_MGMT_1: 슬립 해제 + 클럭 소스 = 자이로 X PLL(0x01)
        #   0x00(내부 8MHz RC)보다 온도/시간 안정성이 좋습니다.
        self._bus.write_byte_data(self._address, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # ★ DLPF — 기존 코드가 설정하지 않아 260 Hz 대역폭으로 열려 있던 부분
        self._bus.write_byte_data(self._address, REG_CONFIG, self._dlpf & 0x07)
        # 샘플레이트 = 1 kHz / (1 + SMPLRT_DIV)
        self._bus.write_byte_data(self._address, REG_SMPLRT_DIV, self._smplrt & 0xFF)

        self._bus.write_byte_data(self._address, REG_GYRO_CONFIG,  0x00)  # ±250 dps
        self._bus.write_byte_data(self._address, REG_ACCEL_CONFIG, 0x00)  # ±2 g

        who = self._bus.read_byte_data(self._address, REG_WHO_AM_I)
        if who != 0x68:
            self.get_logger().warn(f'WHO_AM_I = 0x{who:02X} (기대 0x68). 배선/주소 확인 요망')

    # ══════════════════════════════════════════════════════════════
    # ★ 버스트 읽기
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _s16(hi: int, lo: int) -> int:
        v = (hi << 8) | lo
        return v - 65536 if v > 32767 else v

    def _read_burst(self):
        """
        0x3B 부터 14바이트를 한 번에 읽는다.
          [0:6]  ACCEL X/Y/Z
          [6:8]  TEMP
          [8:14] GYRO X/Y/Z
        MPU6050 은 버스트 읽기가 진행되는 동안 출력 레지스터를 갱신하지 않으므로
        6축이 '같은 샘플'임이 보장됩니다 (기존 12회 분할 읽기에는 없던 성질).
        """
        d = self._bus.read_i2c_block_data(self._address, BURST_START, BURST_LEN)
        ax = self._s16(d[0],  d[1])  / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        ay = self._s16(d[2],  d[3])  / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        az = self._s16(d[4],  d[5])  / ACCEL_SCALE_MODIFIER * GRAVITY_MS2
        gx = self._s16(d[8],  d[9])  / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gy = self._s16(d[10], d[11]) / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        gz = self._s16(d[12], d[13]) / GYRO_SCALE_MODIFIER * DEG_TO_RAD
        return ax * self._ax_sign, ay, az, gx, gy, gz * self._gz_sign

    # ══════════════════════════════════════════════════════════════
    # 복구
    # ══════════════════════════════════════════════════════════════

    def _try_recover(self) -> None:
        """SMBus 핸들을 닫고 → GPIO 비트뱅잉 → 핸들 재생성 → 센서 재초기화."""
        self._recover_count += 1
        self.get_logger().warn(
            f'[I2C] 연속 실패 {self._fail_streak}회 → 버스 복구 시도 '
            f'(누적 글리치 {self._glitch_count}, 복구 {self._recover_count})')
        try:
            self._bus.close()
        except Exception:
            pass

        ok = False
        if self._recovery is not None:
            ok = self._recovery.recover()

        try:
            self._bus = SMBus(self._i2c_bus_num)
            self._init_mpu6050()
            self._fail_streak = 0
            if ok:
                self._recover_ok += 1
            self.get_logger().info(
                f'[I2C] 복구 {"성공" if ok else "완료(SDA 상태 미확인)"} — 재초기화 OK')
        except Exception as exc:
            self.get_logger().error(f'[I2C] 재초기화 실패: {exc}')

    # ══════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        # ※ vy 는 판정에서 제외했습니다. motor_node 의 vy 는 실측이 아니라
        #    'cmd_vy 적분값'이라, 명령만 있고 실제로 안 움직여도 주행 중으로 오판합니다.
        self._is_moving = (abs(vx) > 0.01 or abs(wz) > 0.01)
        self._last_odom_time = time.monotonic()

    def _timer_cb(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        # ── 센서 읽기 (실패 시 복구) ─────────────────────────────
        try:
            ax, ay, az, gx, gy, gz = self._read_burst()
            self._fail_streak = 0
        except Exception as exc:
            self._glitch_count += 1
            self._fail_streak  += 1
            self.get_logger().warn(
                f'[I2C] 읽기 실패({self._fail_streak}): {exc}',
                throttle_duration_sec=1.0)
            if self._fail_streak >= self._fail_streak_limit:
                self._try_recover()
            return

        # ── 물리적 타당성 검사 ───────────────────────────────────
        #   ±250 dps = ±4.36 rad/s 가 풀스케일. 그 근처면 포화이거나 손상된 값입니다.
        #   가속도는 정지/저속 주행에서 |a| 가 g 근방이어야 합니다.
        if abs(gz) > 4.3 or math.hypot(math.hypot(ax, ay), az) > 4.0 * GRAVITY_MS2:
            self._torn_suspect += 1
            self.get_logger().warn(
                f'[IMU] 비정상 값 폐기 gz={gz:.2f} |a|={math.hypot(math.hypot(ax,ay),az):.1f} '
                f'(누적 {self._torn_suspect})', throttle_duration_sec=2.0)
            return

        # ── 상보 필터 ────────────────────────────────────────────
        roll, pitch, yaw = self._complementary_filter(ax, ay, az, gx, gy, gz, dt)
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # ── 동적 공분산 ──────────────────────────────────────────
        #   ★ 워치독: /odom_motor 가 끊기면 '정지'로 단정하지 않고 보수적으로 주행 취급.
        #     정지(1e-5)는 매우 강한 확신이라, 잘못 걸리면 EKF 가 흔들리는 자이로를
        #     그대로 믿어버립니다.
        odom_stale = (time.monotonic() - self._last_odom_time) > self._odom_timeout
        moving = True if odom_stale else self._is_moving

        if moving:
            ori_cov, ang_cov, acc_cov = (ORIENTATION_COV_MOVE,
                                         ANGULAR_VEL_COV_MOVE,
                                         LINEAR_ACCEL_COV_MOVE)
        else:
            ori_cov, ang_cov, acc_cov = (ORIENTATION_COV_STOP,
                                         ANGULAR_VEL_COV_STOP,
                                         LINEAR_ACCEL_COV_STOP)

        msg.orientation_covariance         = [ori_cov, 0.0, 0.0,
                                              0.0, ori_cov, 0.0,
                                              0.0, 0.0, ori_cov]
        msg.angular_velocity_covariance    = [ang_cov, 0.0, 0.0,
                                              0.0, ang_cov, 0.0,
                                              0.0, 0.0, ang_cov]
        msg.linear_acceleration_covariance = [acc_cov, 0.0, 0.0,
                                              0.0, acc_cov, 0.0,
                                              0.0, 0.0, acc_cov]
        self._pub.publish(msg)

    # ══════════════════════════════════════════════════════════════
    # 필터 / 변환
    # ══════════════════════════════════════════════════════════════

    def _complementary_filter(self, ax, ay, az, gx, gy, gz, dt):
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        self._roll  = self._alpha * (self._roll  + gx * dt) + (1 - self._alpha) * roll_acc
        self._pitch = self._alpha * (self._pitch + gy * dt) + (1 - self._alpha) * pitch_acc
        self._yaw   = self._yaw + gz * dt
        return self._roll, self._pitch, self._yaw

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll * .5),  math.sin(roll * .5)
        cp, sp = math.cos(pitch * .5), math.sin(pitch * .5)
        cy, sy = math.cos(yaw * .5),   math.sin(yaw * .5)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    # ══════════════════════════════════════════════════════════════
    # 진단
    # ══════════════════════════════════════════════════════════════

    def _publish_health(self) -> None:
        """
        0:글리치누적 1:복구시도 2:복구성공 3:비정상값폐기 4:현재연속실패 5:주행판정

        ★ 배선을 고치기 전/후로 [0] 값의 증가 속도를 비교하십시오.
          이것이 하드웨어 개선 효과를 측정하는 유일한 정량 지표입니다.
        """
        m = Float32MultiArray()
        m.data = [float(self._glitch_count), float(self._recover_count),
                  float(self._recover_ok),   float(self._torn_suspect),
                  float(self._fail_streak),  1.0 if self._is_moving else 0.0]
        self._pub_diag.publish(m)
        if self._glitch_count > 0:
            self.get_logger().info(
                f'[I2C 건강] 글리치 {self._glitch_count} / 복구 {self._recover_count} '
                f'(성공 {self._recover_ok}) / 비정상값 {self._torn_suspect}',
                throttle_duration_sec=10.0)

    def destroy_node(self):
        try:
            self._timer.cancel()
            self._diag_timer.cancel()
        except Exception:
            pass
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