#!/bin/bash
################################################################################
# Jetson Nano 컴파일 전체 명령어 스크립트
# Happy Robot ROS Project
# 작성일: 2025-12-16
################################################################################

set -e  # 에러 발생 시 중단

echo "======================================================================"
echo "Jetson Nano - Happy Robot ROS 프로젝트 컴파일 스크립트"
echo "======================================================================"

# ROS 배포판 설정 (melodic 또는 noetic)
# Ubuntu 18.04 -> melodic
# Ubuntu 20.04 -> noetic
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" == "18.04" ]; then
    export ROS_DISTRO=melodic
elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    export ROS_DISTRO=noetic
else
    echo "⚠️  경고: Ubuntu 버전이 18.04 또는 20.04가 아닙니다."
    echo "수동으로 ROS_DISTRO를 설정하세요: export ROS_DISTRO=melodic"
    exit 1
fi

echo "✅ Ubuntu 버전: $UBUNTU_VERSION"
echo "✅ ROS 배포판: $ROS_DISTRO"

################################################################################
# Step 1: 시스템 업데이트
################################################################################
echo ""
echo "======================================================================"
echo "Step 1: 시스템 업데이트"
echo "======================================================================"

sudo apt update
sudo apt upgrade -y

################################################################################
# Step 2: ROS 설치 (이미 설치되어 있으면 스킵)
################################################################################
echo ""
echo "======================================================================"
echo "Step 2: ROS $ROS_DISTRO 설치 확인"
echo "======================================================================"

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "ROS $ROS_DISTRO이 설치되어 있지 않습니다. 설치를 시작합니다..."
    
    # ROS 저장소 추가
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # GPG 키 추가
    if [ "$ROS_DISTRO" == "melodic" ]; then
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    else
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    fi
    
    # ROS 설치
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-desktop-full
    
    # rosdep 초기화
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # 환경 설정
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    echo "✅ ROS $ROS_DISTRO 설치 완료"
else
    echo "✅ ROS $ROS_DISTRO이 이미 설치되어 있습니다."
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

################################################################################
# Step 3: 빌드 도구 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 3: 빌드 도구 설치"
echo "======================================================================"

sudo apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git

# CMake 버전 확인
CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
echo "✅ CMake 버전: $CMAKE_VERSION"

# CMake 3.16 미만이면 경고
CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 16 ]); then
    echo "⚠️  경고: CMake 버전이 3.16 미만입니다. WoorobotSDK 빌드에 문제가 있을 수 있습니다."
    echo "CMake 업그레이드를 권장합니다."
fi

################################################################################
# Step 4: 필수 ROS 패키지 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 4: 필수 ROS 패키지 설치"
echo "======================================================================"

sudo apt install -y \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-laser-geometry \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-dynamic-reconfigure \
    ros-$ROS_DISTRO-serial

echo "✅ ROS 패키지 설치 완료"

################################################################################
# Step 5: Cartographer (SLAM) 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 5: Cartographer SLAM 설치"
echo "======================================================================"

# Cartographer 패키지 설치 시도
if sudo apt install -y ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros ros-$ROS_DISTRO-cartographer-ros-msgs; then
    echo "✅ Cartographer 패키지 설치 완료"
else
    echo "⚠️  경고: Cartographer 패키지를 찾을 수 없습니다."
    echo "소스에서 빌드해야 할 수 있습니다. (메모리 집약적)"
    echo "계속 진행하려면 Enter를 누르세요..."
    read
fi

################################################################################
# Step 6: Point Cloud Library (PCL) 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 6: Point Cloud Library (PCL) 설치"
echo "======================================================================"

sudo apt install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions

# PCL 버전 확인
PCL_VERSION=$(pkg-config --modversion pcl_common 2>/dev/null || echo "Unknown")
echo "✅ PCL 버전: $PCL_VERSION"

if [ "$PCL_VERSION" != "Unknown" ]; then
    PCL_MAJOR=$(echo $PCL_VERSION | cut -d. -f1)
    PCL_MINOR=$(echo $PCL_VERSION | cut -d. -f2)
    if [ "$PCL_MAJOR" -eq 1 ] && [ "$PCL_MINOR" -lt 10 ]; then
        echo "⚠️  경고: PCL 버전이 1.10 미만입니다 ($PCL_VERSION)."
        echo "CMakeLists.txt에서 find_package(PCL 1.10 REQUIRED)를"
        echo "find_package(PCL $PCL_VERSION REQUIRED)로 수정해야 할 수 있습니다."
    fi
fi

################################################################################
# Step 7: Eigen3 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 7: Eigen3 설치"
echo "======================================================================"

sudo apt install -y libeigen3-dev

EIGEN_VERSION=$(pkg-config --modversion eigen3 2>/dev/null || echo "Unknown")
echo "✅ Eigen3 버전: $EIGEN_VERSION"

################################################################################
# Step 8: Python 의존성 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 8: Python 의존성 설치"
echo "======================================================================"

sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy

# FastAPI 및 WebSocket 서버용
pip3 install --user fastapi uvicorn websockets

echo "✅ Python 패키지 설치 완료"

################################################################################
# Step 9: SWAP 메모리 설정 (매우 중요!)
################################################################################
echo ""
echo "======================================================================"
echo "Step 9: SWAP 메모리 설정"
echo "======================================================================"

# 현재 SWAP 확인
CURRENT_SWAP=$(free -h | grep Swap | awk '{print $2}')
echo "현재 SWAP 크기: $CURRENT_SWAP"

# SWAP이 8GB 미만이면 추가
SWAP_SIZE_MB=$(free -m | grep Swap | awk '{print $2}')
if [ "$SWAP_SIZE_MB" -lt 8000 ]; then
    echo "SWAP 메모리가 부족합니다. 8GB SWAP을 생성합니다..."
    
    # 기존 swapfile이 있으면 제거
    if [ -f /swapfile ]; then
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm /swapfile
    fi
    
    # 8GB SWAP 생성
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    
    # /etc/fstab에 영구 추가 (중복 방지)
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    
    echo "✅ SWAP 8GB 설정 완료"
else
    echo "✅ SWAP 메모리가 충분합니다."
fi

free -h

################################################################################
# Step 10: USB 시리얼 포트 권한 설정
################################################################################
echo ""
echo "======================================================================"
echo "Step 10: USB 시리얼 포트 권한 설정"
echo "======================================================================"

# dialout 그룹에 사용자 추가
if ! groups $USER | grep -q dialout; then
    sudo usermod -aG dialout $USER
    echo "✅ $USER를 dialout 그룹에 추가했습니다."
    echo "⚠️  변경사항 적용을 위해 로그아웃 후 재로그인하거나 'newgrp dialout' 명령을 실행하세요."
else
    echo "✅ $USER가 이미 dialout 그룹에 속해 있습니다."
fi

################################################################################
# Step 11: 워크스페이스로 이동
################################################################################
echo ""
echo "======================================================================"
echo "Step 11: 워크스페이스 준비"
echo "======================================================================"

# 워크스페이스 경로 (필요시 수정)
WORKSPACE_DIR=~/happy_ws

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ 워크스페이스 디렉토리를 찾을 수 없습니다: $WORKSPACE_DIR"
    echo "현재 디렉토리에서 진행하시겠습니까? (y/n)"
    read answer
    if [ "$answer" != "y" ]; then
        echo "스크립트를 종료합니다."
        exit 1
    fi
    WORKSPACE_DIR=$(pwd)
fi

cd $WORKSPACE_DIR
echo "✅ 워크스페이스: $WORKSPACE_DIR"

################################################################################
# Step 12: rosdep으로 의존성 자동 설치
################################################################################
echo ""
echo "======================================================================"
echo "Step 12: rosdep으로 의존성 자동 설치"
echo "======================================================================"

if [ -d "$WORKSPACE_DIR/src" ]; then
    cd $WORKSPACE_DIR
    rosdep install --from-paths src --ignore-src -r -y || true
    echo "✅ rosdep 의존성 설치 완료 (일부 실패는 정상)"
else
    echo "⚠️  src 디렉토리를 찾을 수 없습니다. rosdep 건너뜀."
fi

################################################################################
# Step 13: PCL 버전 확인 및 CMakeLists.txt 수정 (옵션)
################################################################################
echo ""
echo "======================================================================"
echo "Step 13: PCL 버전 호환성 확인"
echo "======================================================================"

if [ "$PCL_MAJOR" -eq 1 ] && [ "$PCL_MINOR" -lt 10 ]; then
    echo "⚠️  PCL 버전이 1.10 미만입니다."
    echo "CMakeLists.txt를 자동으로 수정하시겠습니까? (y/n)"
    read answer
    if [ "$answer" == "y" ]; then
        # CMakeLists.txt에서 PCL 1.10을 현재 버전으로 변경
        find $WORKSPACE_DIR/src -name "CMakeLists.txt" -exec sed -i "s/find_package(PCL 1\.10 REQUIRED)/find_package(PCL $PCL_VERSION REQUIRED)/g" {} \;
        echo "✅ CMakeLists.txt PCL 버전 수정 완료"
    fi
fi

################################################################################
# Step 14: 전력 모드 최대 성능으로 설정 (Jetson Nano)
################################################################################
echo ""
echo "======================================================================"
echo "Step 14: Jetson Nano 전력 모드 설정"
echo "======================================================================"

if [ -f "/usr/sbin/nvpmodel" ]; then
    echo "Jetson Nano 전력 모드를 최대 성능(10W)으로 설정합니다..."
    sudo nvpmodel -m 0
    sudo jetson_clocks
    echo "✅ 전력 모드 설정 완료"
else
    echo "⚠️  nvpmodel을 찾을 수 없습니다. Jetson Nano가 아닐 수 있습니다."
fi

################################################################################
# Step 15: 빌드 시작
################################################################################
echo ""
echo "======================================================================"
echo "Step 15: 프로젝트 빌드 시작"
echo "======================================================================"

cd $WORKSPACE_DIR

# 이전 빌드 디렉토리 정리 (옵션)
echo "이전 빌드 결과를 삭제하시겠습니까? (y/n)"
read answer
if [ "$answer" == "y" ]; then
    rm -rf build devel .catkin_workspace
    echo "✅ 빌드 디렉토리 정리 완료"
fi

# 메모리에 따라 병렬 빌드 수 결정
TOTAL_MEM_MB=$(free -m | awk 'NR==2{print $2}')
if [ "$TOTAL_MEM_MB" -lt 4000 ]; then
    JOBS=1
    echo "⚠️  메모리가 4GB 미만입니다. 단일 코어로 빌드합니다 (-j1)"
else
    JOBS=2
    echo "메모리가 충분합니다. 2개 코어로 빌드합니다 (-j2)"
fi

echo ""
echo "======================================================================"
echo "🚀 컴파일 시작! (약 30-60분 소요 예상)"
echo "======================================================================"
echo ""

# 빌드 시작 시간 기록
START_TIME=$(date +%s)

# catkin_make로 빌드 (메모리 최적화)
catkin_make -j$JOBS -DCMAKE_BUILD_TYPE=Release

# 빌드 종료 시간
END_TIME=$(date +%s)
BUILD_TIME=$((END_TIME - START_TIME))
BUILD_MINUTES=$((BUILD_TIME / 60))
BUILD_SECONDS=$((BUILD_TIME % 60))

echo ""
echo "======================================================================"
echo "✅ 빌드 완료!"
echo "======================================================================"
echo "빌드 소요 시간: ${BUILD_MINUTES}분 ${BUILD_SECONDS}초"

################################################################################
# Step 16: 환경 설정
################################################################################
echo ""
echo "======================================================================"
echo "Step 16: 환경 설정"
echo "======================================================================"

# .bashrc에 워크스페이스 소싱 추가 (중복 방지)
if ! grep -q "source $WORKSPACE_DIR/devel/setup.bash" ~/.bashrc; then
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
    echo "✅ .bashrc에 워크스페이스 환경 설정 추가"
fi

# 현재 세션에 소싱
source $WORKSPACE_DIR/devel/setup.bash

echo "✅ 환경 설정 완료"

################################################################################
# Step 17: 빌드 결과 확인
################################################################################
echo ""
echo "======================================================================"
echo "Step 17: 빌드 결과 확인"
echo "======================================================================"

# 빌드된 실행 파일 확인
if [ -d "$WORKSPACE_DIR/devel/lib/happy_robo" ]; then
    echo "빌드된 실행 파일 목록:"
    ls -lh $WORKSPACE_DIR/devel/lib/happy_robo/
else
    echo "⚠️  happy_robo 실행 파일을 찾을 수 없습니다."
fi

# 빌드된 Python 스크립트 확인
if [ -d "$WORKSPACE_DIR/devel/lib/happy_robo" ]; then
    echo ""
    echo "빌드된 Python 스크립트:"
    ls -lh $WORKSPACE_DIR/devel/lib/happy_robo/*.py 2>/dev/null || echo "(Python 스크립트 없음)"
fi

################################################################################
# 완료 메시지
################################################################################
echo ""
echo "======================================================================"
echo "🎉 Happy Robot ROS 프로젝트 컴파일 완료!"
echo "======================================================================"
echo ""
echo "다음 명령으로 노드를 실행할 수 있습니다:"
echo ""
echo "  # 환경 소싱 (새 터미널에서)"
echo "  source $WORKSPACE_DIR/devel/setup.bash"
echo ""
echo "  # ROS 마스터 실행"
echo "  roscore &"
echo ""
echo "  # 속도 변환기 노드 실행"
echo "  rosrun happy_robo vel_converter_sdk"
echo ""
echo "  # Python 마스터 노드 실행"
echo "  rosrun happy_robo master_node.py"
echo ""
echo "  # Launch 파일 실행 (있는 경우)"
echo "  roslaunch happy_robo <launch_file>.launch"
echo ""
echo "======================================================================"
echo "⚠️  주의사항:"
echo "======================================================================"
echo "1. USB 시리얼 권한 적용을 위해 로그아웃 후 재로그인하세요."
echo "2. /dev/ttyUSB0 포트가 연결되어 있는지 확인하세요."
echo "3. 모터 컨트롤러 전원이 켜져 있는지 확인하세요."
echo ""
echo "문제 해결:"
echo "  - 빌드 에러: JETSON_NANO_BUILD_GUIDE.md의 문제 해결 섹션 참조"
echo "  - 코드 분석: CODE_ANALYSIS_SUMMARY.md 참조"
echo ""
echo "======================================================================"
