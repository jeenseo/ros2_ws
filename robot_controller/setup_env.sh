#!/bin/bash
# =============================================================================
# setup_env.sh
# ============
# Raspberry Pi SSH 세션용 ROS 2 환경 설정 스크립트
#
# 사용법:
#   source ~/ros2_ws/src/robot_controller/setup_env.sh
#
# 또는 ~/.bashrc에 추가:
#   echo "source ~/ros2_ws/src/robot_controller/setup_env.sh" >> ~/.bashrc
# =============================================================================

# ── ROS 2 기본 설정 ──────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# ── 통신 안정성: Domain ID ───────────────────────────────────────
# 동일 네트워크 내 다른 ROS 2 노드와의 충돌 방지
export ROS_DOMAIN_ID=30

# ── Fast-RTPS 디스커버리 서버 (SSH 세션 안정성) ──────────────────
# Raspberry Pi의 고정 IP 주소로 변경하세요
export ROS_DISCOVERY_SERVER="192.168.1.100:11811"

# Fast-RTPS 프로파일 (슈퍼클라이언트 모드)
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/fastdds_super_client.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# ── CAN 버스 초기화 (sudo 필요) ───────────────────────────────────
# 이미 활성화된 경우 무시됩니다
if ! ip link show can0 2>/dev/null | grep -q "UP"; then
    echo "[setup_env] CAN0 초기화 중..."
    sudo ip link set can0 up type can bitrate 500000
    sudo ip link set can0 txqueuelen 1000
    echo "[setup_env] CAN0 활성화 완료"
else
    echo "[setup_env] CAN0 이미 활성화됨"
fi

# ── 상태 출력 ─────────────────────────────────────────────────────
echo "====================================================="
echo " ROS 2 환경 설정 완료"
echo " ROS_DOMAIN_ID       = $ROS_DOMAIN_ID"
echo " ROS_DISCOVERY_SERVER = $ROS_DISCOVERY_SERVER"
echo " Workspace           = ~/ros2_ws"
echo "====================================================="
echo ""
echo "실행 명령:"
echo "  ros2 launch robot_controller nav2.launch.py"
echo ""
