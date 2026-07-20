warning: in the working copy of 'robot_controller/robot_controller/motor_node.py', LF will be replaced by CRLF the next time Git touches it
warning: in the working copy of 'robot_controller/robot_controller/mpu6050_node.py', LF will be replaced by CRLF the next time Git touches it
[1mdiff --git a/.vscode/settings.json b/.vscode/settings.json[m
[1mindex 23c04d2..b9b3752 100644[m
[1m--- a/.vscode/settings.json[m
[1m+++ b/.vscode/settings.json[m
[36m@@ -9,5 +9,6 @@[m
         "/home/jeen/ros2_ws/build/robot_controller",[m
         "/home/jeen/ros2_ws/install/robot_controller/lib/python3.12/site-packages",[m
         "/opt/ros/jazzy/lib/python3.12/site-packages"[m
[31m-    ][m
[32m+[m[32m    ],[m
[32m+[m[32m    "cmake.sourceDirectory": "C:/Users/tjwld/ros2_ws/src/rf2o_laser_odometry"[m
 }[m
\ No newline at end of file[m
[1mdiff --git a/README.md b/README.md[m
[1mdeleted file mode 100644[m
[1mindex 534d2c9..0000000[m
[1m--- a/README.md[m
[1m+++ /dev/null[m
[36m@@ -1,106 +0,0 @@[m
[31m-[m
[31m-[m
[31m-[m
[31m-[m
[31m-# 🤖 ROS2 기반 GPS-less 실내 자율주행 배달 로봇 (CAN 통신 & EKF 센서 퓨전)[m
[31m-[m
[31m-> **프로젝트 기간:** 2026.04 ~ 2026.06 (신한대학교 캡스톤 디자인)  [m
[31m-> **핵심 키워드:** `ROS2 Jazzy`, `Raspberry Pi 4`, `STM32`, `CAN Bus`, `EKF Sensor Fusion`, `MPPI Planner`, `Feedforward Control`[m
[31m-[m
[31m-https://github.com/user-attachments/assets/7d0e72e7-6a7c-47e9-95b3-fcc1107b7214[m
[31m-[m
[31m-https://github.com/user-attachments/assets/ab331f74-d661-44e5-b351-fbb98d675f2c[m
[31m-[m
[31m-[m
[31m-[m
[31m-## 📌 Project Overview[m
[31m-병원, 학교 복도 등 GPS(GNSS) 사용이 불가능한 실내 환경에서, 주변의 동적 장애물을 실시간으로 회피하며 목적지까지 주행하는 10kg급 실내 자율주행 배달 로봇입니다[cite: 4]. [m
[31m-[m
[31m-단순히 오픈소스를 조립한 수준을 넘어, **저가형 센서와 모터의 물리적 한계(노이즈, 슬립, 데드밴드)를 하드웨어 아키텍처 분리와 자체적인 소프트웨어 알고리즘(EKF, 2단계 피드포워드)으로 완벽하게 극복(Masking)**하는 것에 집중했습니다[cite: 4].[m
[31m-[m
[31m----[m
[31m-[m
[31m-## ⚙️ System Architecture[m
[31m-[m
[31m-<!-- 💡 팁: PPT 7페이지의 '시스템 하드웨어 블록도' 이미지를 여기에 삽입하세요 -->[m
[31m-![시스템 아키텍처](아키텍처_이미지_링크.png)[m
[31m-[m
[31m-고연산(SLAM, Navigation)을 담당하는 **상위 제어기(Raspberry Pi 4)**와 실시간 하드웨어 제어를 담당하는 **하위 제어기(STM32F103RBT6)**로 완벽히 분업화된 분산형 아키텍처를 구축했습니다[cite: 4].[m
[31m-[m
[31m-*   **High-Level (Raspberry Pi 4, ROS2 Jazzy):** RPLIDAR A1, MPU6050 IMU, 모터 엔코더 데이터를 통합하여 EKF 기반 위치 추정 및 Nav2(MPPI) 경로 생성 수행[cite: 4].[m
[31m-*   **Low-Level (STM32, Firmware):** 상위 제어기로부터 수신한 목표 속도를 2단계 피드포워드+PID로 제어하고, 실시간 모터 엔코더 펄스(QEI)를 측정하여 피드백[cite: 4].[m
[31m-*   **Communication:** CAN Bus 기반 차동 신호 양방향 통신 네트워크 구축[cite: 4, 8].[m
[31m-[m
[31m----[m
[31m-[m
[31m-## 🛠️ Tech Stack & Hardware[m
[31m-[m
[31m-| Category | Technology |[m
[31m-| :--- | :--- |[m
[31m-| **SBC / MCU** | Raspberry Pi 4B (Ubuntu 24.04), STM32F103RBT6[cite: 4] |[m
[31m-| **Framework** | ROS 2 Jazzy, STM32CubeIDE (HAL Driver)[cite: 4] |[m
[31m-| **Sensors** | RPLIDAR A1, MPU6050(IMU), Motor Encoders[cite: 4, 7, 9] |[m
[31m-| **Languages** | Python (ROS 2 Nodes), C/C++ (STM32 Firmware)[cite: 7, 8, 9] |[m
[31m-| **Communication**| CAN Bus (SocketCAN), I2C, Serial[cite: 7, 8, 9] |[m
[31m-[m
[31m----[m
[31m-[m
[31m-## 🚀 Key Troubleshooting & Core Algorithms[m
[31m-[m
[31m-### 1. EMI 노이즈 극복: USART 단일 통신에서 CAN Bus 아키텍처로 전환[m
[31m-*   **Problem:** 기존 USART(Serial) 통신 방식은 10kg 고중량 로봇 모터 기동 시 발생하는 강력한 전자기 간섭(EMI) 노이즈에 취약하여 데이터 패킷이 파괴되고 제어가 단절되는 치명적 한계가 존재[cite: 4].[m
[31m-*   **Solution:** 차량용 표준인 **CAN 통신(Differential Signal)을 도입**하여 공통 모드 노이즈를 하드웨어적으로 완벽 상쇄[cite: 4].[m
[31m-*   **Result:** 하드웨어 자가 치유(CRC 감지 및 자동 재전송) 기능을 활용하여 데이터 유실률 0% 및 Zero-Latency 제어 달성. Float(4byte) 데이터를 int16_t(2byte)로 압축 패킹하여 전송 효율을 극대화함[cite: 4].[m
[31m-[m
[31m-### 2. 고중량 차체 제어: 2단계 피드포워드(Feedforward) + PID 모터 제어[m
[31m-*   **Problem:** 10kg의 기구적 마찰력으로 인해 단순 PID 제어 시 모터의 데드밴드(Deadband) 구간에서 I-Gain이 무한 누적되어 로봇이 급발진하는 Integral Windup(적분 발작) 발생[cite: 4].[m
[31m-*   **Solution:** 모터 출력 특성을 분석하여 **'2단계 피드포워드 제어'** 구현[cite: 4].[m
[31m-    *   `Zone 1 (비선형)`: 정지 마찰을 깨는 2차 곡선형 가속 구간 설계로 부드러운 출발 보장[cite: 4].[m
[31m-    *   `Zone 2 (선형)`: 마찰이 무시되는 관성 구간부터 PID 정밀 오차 보정 개입[cite: 4].[m
[31m-*   **Result:** Stop-and-Go 진자 운동 및 급발진을 완전히 차단하고, Nav2 플래너가 요구하는 cmd_vel을 부드럽고 정밀하게 추종[cite: 4].[m
[31m-[m
[31m-### 3. 단일 센서 한계 돌파: EKF 다중 센서 퓨전 (LiDAR + IMU + Encoder)[m
[31m-*   **Problem:** 실내 복도 등 특징점이 부족한 대칭 환경에서 단일 LiDAR 의존 시 맵 매칭 붕괴(Teleport) 발생. 바퀴 엔코더는 슬립(Slip)에 취약하고, IMU는 가속도 이중 적분 시 심각한 Drift 발생[cite: 4].[m
[31m-*   **Solution:** `robot_localization` 패키지를 활용해 각 센서의 신뢰할 수 있는 데이터만 취사선택하는 EKF(Extended Kalman Filter) 설계[cite: 2, 4].[m
[31m-    *   `Encoder`: 직진 선속도(Vx)의 1차 소스로 활용[cite: 2, 4].[m
[31m-    *   `IMU`: 회전 각속도(Vyaw)의 1차 소스로 활용하며 EKF 가중치 최적화[cite: 2, 4, 9].[m
[31m-    *   `LiDAR (rf2o)`: 절대 좌표(X,Y,Yaw)를 장기적인 드리프트 보정 용도로만 제한적 개입[cite: 2, 4].[m
[31m-*   **Result:** 제자리 급회전 시나 바퀴 슬립 발생 상황에서도 안정적으로 절대 궤적을 유지하며 시스템 붕괴 완벽 차단[cite: 4].[m
[31m-[m
[31m-### 4. 동적 장애물 회피: 차세대 MPPI 플래너 최적화[m
[31m-*   **Problem:** 기존 DWB 로컬 플래너는 궤적이 정적이며, 학교 복도처럼 보행자가 돌발적으로 나타나는 환경에서 회피 능력이 현저히 저하됨[cite: 3, 4].[m
[31m-*   **Solution:** 몬테카를로 병렬 샘플링 기반의 **MPPI 플래너 전격 도입**[cite: 3, 4].[m
[31m-*   **Result:** Raspberry Pi 멀티코어를 활용해 0.1초마다 2,000개의 가상 궤적(Batch size 2000)을 흩뿌려 최적의 틈새를 찾아내는 극한의 동적 회피 기동 성공. `ObstaclesCritic`과 `TwirlingCritic` 비용 가중치를 최적화하여 회피 동작 안정화[cite: 3, 4].[m
[31m-[m
[31m----[m
[31m-[m
[31m-## 📂 Repository Structure[m
[31m-[m
[31m-```text[m
[31m-ros2_ws/src/robot_controller/[m
[31m-├── config/[m
[31m-│   ├── ekf.yaml               # EKF 센서 퓨전 파라미터 (공분산 최적화)[cite: 2][m
[31m-│   └── nav2_params.yaml       # Nav2 & MPPI 플래너, Costmap 파라미터[cite: 3][m
[31m-├── launch/[m
[31m-│   └── nav2.launch.py         # 시스템 전체 노드 런치 (TF, EKF, Nav2 등)[cite: 5][m
[31m-├── maps/[m
[31m-│   └── map.yaml               # 실내 주행용 2D 점유 격자 지도[cite: 3][m
[31m-├── urdf/[m
[31m-│   └── robot.urdf             # 로봇 3D 모델 및 센서 위치(TF) 정의[cite: 6][m
[31m-├── robot_controller/[m
[31m-│   ├── motor_node.py          # CAN 통신 송수신 및 차동 구동 오도메트리 연산[cite: 8][m
[31m-│   └── mpu6050_node.py        # I2C 통신, 상보 필터 기반 IMU 데이터 연산[cite: 9][m
[31m-├── rf2o_laser_odometry/       # LiDAR 스캔 매칭 오도메트리 모듈[cite: 1][m
[31m-└── sllidar_ros2/              # RPLIDAR 제어 모듈[cite: 1][m
[31m-[m
[31m-🛠️ How to Build & Run[m
[31m-Bash[m
[31m-# 1. 작업 공간 이동 및 빌드[m
[31m-cd ~/ros2_ws[m
[31m-colcon build --packages-select robot_controller rf2o_laser_odometry sllidar_ros2[m
[31m-[m
[31m-# 2. 환경 변수 적용[m
[31m-source install/setup.bash[m
[31m-[m
[31m-# 3. 로봇 자율주행 통합 시스템 실행[m
[31m-ros2 launch robot_controller nav2.launch.py[m
[1mdiff --git a/robot_controller/config/ekf.yaml b/robot_controller/config/ekf.yaml[m
[1mindex 99c8440..999dc84 100644[m
[1m--- a/robot_controller/config/ekf.yaml[m
[1m+++ b/robot_controller/config/ekf.yaml[m
[36m@@ -52,7 +52,7 @@[m [mekf_node:[m
     # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az][m
     # rf2o: X, Y 위치 + Yaw (보조: 장기 드리프트 보정 역할)[m
     # rf2o의 covariance가 IMU보다 크므로 EKF가 자동으로 낮은 가중치 부여 → gentle correction[m
[31m-    odom0_config: [true,  true,  false,[m
[32m+[m[32m    odom0_config: [true,  true,  false,  # X, Y 위치 활성화[m
                    false, false, true,   # Yaw 활성화 (보조 드리프트 보정)[m
                    false, false, false,[m
                    false, false, false,[m
[36m@@ -81,7 +81,7 @@[m [mekf_node:[m
     # ORIENTATION_COV=1e-4, ANGULAR_VEL_COV=1e-5 (mpu6050_node.py)[m
     # IMU: Yaw(index5) + Wz(index11) — 회전 추적 1차 소스[m
     imu0_config: [false, false, false,[m
[31m-                  false, false, false,   # Yaw 활성화 (1차 회전 소스)[m
[32m+[m[32m                  false, false, false,[m[41m  [m
                   false, false, false,[m
                   false, false, true,   # Wz 각속도 활성화[m
                   false, false, false][m
[36m@@ -103,8 +103,8 @@[m [mekf_node:[m
     # 오직 치고 나가는 전진 속도(Vx)와 회전 속도(Vyaw)만 true로 살려둡니다.[m
     odom1_config: [false, false, false,[m
                    false, false, false,[m
[31m-                   true,  false, false,[m
[31m-                   false, false, false,   # Vx, Vyaw 활성화 (회전 추적 2차 소스)[m
[32m+[m[32m                   true,  false, false,   # Vx 활성화 (전진 속도 2차 소스)[m
[32m+[m[32m                   false, false, false,[m[41m   [m
                    false, false, false][m
 [m
     odom1_queue_size:     5[m
[36m@@ -120,34 +120,39 @@[m [mekf_node:[m
     # [수정] x,y 프로세스 노이즈 0.05→0.20: 회전 중 rf2o 실패 허용[m
     # 큰 Q → EKF가 rf2o garbage 값에 덜 민감하게 반응[m
 # x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az[m
[32m+[m[32m# ─────────────────────────────────────────────────────────────[m
[32m+[m[32m    # 프로세스 노이즈 공분산 Q (15x15 대각 행렬)[m
[32m+[m[32m    # 시스템 내부 물리 모델의 불확실성을 정의합니다.[m[41m [m
[32m+[m[32m    # ─────────────────────────────────────────────────────────────[m
     process_noise_covariance: [[m
[31m-      0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0, # Yaw 노이즈 (0.06 -> 0.03 하향: EKF가 rf2o의 Yaw를 꽉 잡게 함)[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.04,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.05,  0.0,   0.0,   0.0, # Vyaw 노이즈 (0.02 -> 0.05 상향: IMU 노이즈를 덜 믿게 함)[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,[m
[31m-      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,[m
[32m+[m[32m      0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.04,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.05,  0.0,   0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,[m[41m [m
[32m+[m[32m      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,[m[41m [m
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.015[m
     ][m
 [m
     # ─────────────────────────────────────────────────────────────[m
[31m-    # 초기 추정 공분산 P0 (15×15 대각 행렬)[m
[32m+[m[32m    # 초기 추정 공분산 P0 (15x15 대각 행렬)[m
[32m+[m[32m    # 시스템 부팅 시 로봇의 초기 위치(0,0,0)에 대한 확신도를 정의합니다.[m
     # ─────────────────────────────────────────────────────────────[m
     initial_estimate_covariance: [[m
[31m-      1e-9,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0, 