#!/bin/bash
################################################################################
# Jetson Nano 완전 자동 설정 스크립트
# Happy Robot ROS Project - One-Click Setup
################################################################################

set -e  # 에러 발생 시 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 로그 함수
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

################################################################################
# 0. 시작 배너
################################################################################
clear
cat << "EOF"
  _   _                           ____       _           _   
 | | | | __ _ _ __  _ __  _   _  |  _ \ ___ | |__   ___ | |_ 
 | |_| |/ _` | '_ \| '_ \| | | | | |_) / _ \| '_ \ / _ \| __|
 |  _  | (_| | |_) | |_) | |_| | |  _ < (_) | |_) | (_) | |_ 
 |_| |_|\__,_| .__/| .__/ \__, | |_| \_\___/|_.__/ \___/ \__|
             |_|   |_|    |___/                               
                                                              
     Jetson Nano Setup Script - Happy Robot ROS Project
     Version: 1.0 | Date: 2025-12-16
EOF

echo ""
log_info "Jetson Nano 완전 자동 설정을 시작합니다..."
echo ""

# 사용자 확인
read -p "계속하시겠습니까? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    log_warn "사용자가 취소했습니다."
    exit 0
fi

################################################################################
# 1. 시스템 정보 확인
################################################################################
log_section "1. 시스템 정보 확인"

# Jetson 모델 확인
if [ -f /etc/nv_tegra_release ]; then
    JETSON_MODEL=$(cat /etc/nv_tegra_release)
    log_info "Jetson 모델: $JETSON_MODEL"
else
    log_warn "Jetson 장치를 감지할 수 없습니다. 일반 Ubuntu로 진행합니다."
fi

# Ubuntu 버전 확인
UBUNTU_VERSION=$(lsb_release -rs)
log_info "Ubuntu 버전: $UBUNTU_VERSION"

# ROS 배포판 결정
if [ "$UBUNTU_VERSION" == "18.04" ]; then
    export ROS_DISTRO=melodic
    log_info "ROS 배포판: Melodic"
elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    export ROS_DISTRO=noetic
    log_info "ROS 배포판: Noetic"
else
    log_error "지원하지 않는 Ubuntu 버전입니다: $UBUNTU_VERSION"
    log_error "Ubuntu 18.04 또는 20.04가 필요합니다."
    exit 1
fi

# 메모리 확인
TOTAL_MEM=$(free -m | awk 'NR==2{print $2}')
log_info "총 메모리: ${TOTAL_MEM}MB"

if [ "$TOTAL_MEM" -lt 3500 ]; then
    log_warn "메모리가 4GB 미만입니다. SWAP 설정이 필수입니다."
    NEED_SWAP=1
else
    NEED_SWAP=0
fi

# 디스크 공간 확인
AVAILABLE_DISK=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
log_info "사용 가능한 디스크 공간: ${AVAILABLE_DISK}GB"

if [ "$AVAILABLE_DISK" -lt 10 ]; then
    log_error "디스크 공간이 부족합니다. 최소 10GB 이상 필요합니다."
    exit 1
fi

################################################################################
# 2. 시스템 업데이트
################################################################################
log_section "2. 시스템 업데이트"

log_info "패키지 목록 업데이트 중..."
sudo apt update

log_info "시스템 업그레이드 중... (시간이 걸릴 수 있습니다)"
sudo apt upgrade -y

################################################################################
# 3. ROS 설치
################################################################################
log_section "3. ROS $ROS_DISTRO 설치"

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    log_info "ROS $ROS_DISTRO 설치 중..."
    
    # ROS 저장소 추가
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # GPG 키 추가
    if [ "$ROS_DISTRO" == "melodic" ]; then
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    else
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    fi
    
    sudo apt update
    
    # ROS 설치
    log_info "ROS Desktop-Full 설치 중... (약 15-30분 소요)"
    sudo apt install -y ros-$ROS_DISTRO-desktop-full
    
    # rosdep 초기화
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    log_info "✅ ROS $ROS_DISTRO 설치 완료"
else
    log_info "✅ ROS $ROS_DISTRO이 이미 설치되어 있습니다."
fi

# ROS 환경 소싱
source /opt/ros/$ROS_DISTRO/setup.bash

################################################################################
# 4. 빌드 도구 설치
################################################################################
log_section "4. 빌드 도구 설치"

log_info "빌드 도구 설치 중..."
sudo apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git \
    wget \
    curl

# CMake 버전 확인
CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
log_info "CMake 버전: $CMAKE_VERSION"

CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)

if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 16 ]); then
    log_warn "CMake 버전이 3.16 미만입니다. 업그레이드를 시도합니다..."
    
    # CMake 최신 버전 설치
    CMAKE_SCRIPT="cmake-3.22.0-linux-aarch64.sh"
    if [ ! -f "$CMAKE_SCRIPT" ]; then
        wget https://github.com/Kitware/CMake/releases/download/v3.22.0/$CMAKE_SCRIPT
        chmod +x $CMAKE_SCRIPT
        sudo ./$CMAKE_SCRIPT --prefix=/usr/local --skip-license
        rm $CMAKE_SCRIPT
    fi
    
    CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
    log_info "✅ CMake 업그레이드 완료: $CMAKE_VERSION"
fi

################################################################################
# 5. 필수 ROS 패키지 설치
################################################################################
log_section "5. 필수 ROS 패키지 설치"

log_info "ROS 패키지 설치 중..."
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
    ros-$ROS_DISTRO-serial \
    ros-$ROS_DISTRO-joy

log_info "✅ ROS 패키지 설치 완료"

################################################################################
# 6. Cartographer (SLAM) 설치
################################################################################
log_section "6. Cartographer SLAM 설치"

if sudo apt install -y ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros ros-$ROS_DISTRO-cartographer-ros-msgs 2>/dev/null; then
    log_info "✅ Cartographer 패키지 설치 완료"
else
    log_warn "Cartographer 패키지를 apt로 설치할 수 없습니다."
    log_warn "소스에서 빌드가 필요할 수 있습니다."
fi

################################################################################
# 7. Point Cloud Library (PCL) 설치
################################################################################
log_section "7. PCL 설치"

log_info "PCL 설치 중..."
sudo apt install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions

PCL_VERSION=$(pkg-config --modversion pcl_common 2>/dev/null || echo "Unknown")
log_info "PCL 버전: $PCL_VERSION"

################################################################################
# 8. Eigen3 설치
################################################################################
log_section "8. Eigen3 설치"

sudo apt install -y libeigen3-dev

EIGEN_VERSION=$(pkg-config --modversion eigen3 2>/dev/null || echo "Unknown")
log_info "Eigen3 버전: $EIGEN_VERSION"

################################################################################
# 9. Python 패키지 설치
################################################################################
log_section "9. Python 패키지 설치"

log_info "Python 패키지 설치 중..."
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-pyqt5 \
    python3-serial

# pip 패키지
pip3 install --user --upgrade pip
pip3 install --user fastapi uvicorn websockets scipy PyQt5

log_info "✅ Python 패키지 설치 완료"

################################################################################
# 10. SWAP 메모리 설정
################################################################################
log_section "10. SWAP 메모리 설정"

CURRENT_SWAP=$(free -m | grep Swap | awk '{print $2}')
log_info "현재 SWAP 크기: ${CURRENT_SWAP}MB"

if [ "$CURRENT_SWAP" -lt 8000 ] || [ "$NEED_SWAP" -eq 1 ]; then
    log_info "SWAP 메모리 8GB 설정 중..."
    
    # 기존 swapfile 제거
    if [ -f /swapfile ]; then
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm /swapfile
    fi
    
    # 8GB SWAP 생성
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    
    # /etc/fstab에 추가 (중복 방지)
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    
    log_info "✅ SWAP 8GB 설정 완료"
    free -h
else
    log_info "✅ SWAP 메모리가 충분합니다."
fi

################################################################################
# 11. USB 시리얼 권한 설정
################################################################################
log_section "11. USB 시리얼 권한 설정"

if ! groups $USER | grep -q dialout; then
    sudo usermod -aG dialout $USER
    log_info "✅ $USER를 dialout 그룹에 추가했습니다."
    log_warn "⚠️  변경사항 적용을 위해 로그아웃 후 재로그인하거나"
    log_warn "    'newgrp dialout' 명령을 실행하세요."
else
    log_info "✅ $USER가 이미 dialout 그룹에 속해 있습니다."
fi

################################################################################
# 12. Jetson Nano 전력 모드 설정
################################################################################
log_section "12. Jetson Nano 전력 모드 설정"

if [ -f "/usr/sbin/nvpmodel" ]; then
    log_info "Jetson Nano 전력 모드를 최대 성능(10W)으로 설정합니다..."
    sudo nvpmodel -m 0
    sudo jetson_clocks
    log_info "✅ 전력 모드 설정 완료"
else
    log_warn "nvpmodel을 찾을 수 없습니다. Jetson Nano가 아닐 수 있습니다."
fi

################################################################################
# 13. 워크스페이스 설정
################################################################################
log_section "13. ROS 워크스페이스 설정"

# 워크스페이스 경로
WORKSPACE_DIR="$HOME/happy_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    log_info "워크스페이스 디렉토리를 찾을 수 없습니다."
    log_info "현재 디렉토리를 워크스페이스로 사용하시겠습니까?"
    read -p "사용하시려면 'y', 새로 생성하려면 'n': " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        WORKSPACE_DIR=$(pwd)
        log_info "현재 디렉토리를 워크스페이스로 사용합니다: $WORKSPACE_DIR"
    else
        log_info "워크스페이스 생성: $WORKSPACE_DIR"
        mkdir -p $WORKSPACE_DIR/src
        cd $WORKSPACE_DIR
        catkin_make
    fi
fi

cd $WORKSPACE_DIR
log_info "워크스페이스: $WORKSPACE_DIR"

################################################################################
# 14. rosdep 의존성 설치
################################################################################
log_section "14. rosdep 의존성 설치"

if [ -d "$WORKSPACE_DIR/src" ]; then
    cd $WORKSPACE_DIR
    log_info "rosdep으로 의존성 설치 중..."
    rosdep install --from-paths src --ignore-src -r -y || true
    log_info "✅ rosdep 의존성 설치 완료"
else
    log_warn "src 디렉토리를 찾을 수 없습니다. rosdep 건너뜀."
fi

################################################################################
# 15. PCL 버전 호환성 처리
################################################################################
log_section "15. PCL 버전 호환성 처리"

if [ "$PCL_VERSION" != "Unknown" ]; then
    PCL_MAJOR=$(echo $PCL_VERSION | cut -d. -f1)
    PCL_MINOR=$(echo $PCL_VERSION | cut -d. -f2)
    
    if [ "$PCL_MAJOR" -eq 1 ] && [ "$PCL_MINOR" -lt 10 ]; then
        log_warn "PCL 버전이 1.10 미만입니다 ($PCL_VERSION)."
        log_info "CMakeLists.txt를 자동으로 수정합니다..."
        
        # CMakeLists.txt에서 PCL 버전 수정
        find $WORKSPACE_DIR/src -name "CMakeLists.txt" -type f -exec sed -i "s/find_package(PCL 1\.10 REQUIRED)/find_package(PCL $PCL_VERSION REQUIRED)/g" {} \;
        
        log_info "✅ PCL 버전 호환성 처리 완료"
    fi
fi

################################################################################
# 16. .bashrc 환경 설정
################################################################################
log_section "16. 환경 변수 설정"

# ROS 환경 추가
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    log_info "✅ ROS 환경을 .bashrc에 추가했습니다."
fi

# 워크스페이스 환경 추가
if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    if ! grep -q "source $WORKSPACE_DIR/devel/setup.bash" ~/.bashrc; then
        echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
        log_info "✅ 워크스페이스 환경을 .bashrc에 추가했습니다."
    fi
fi

# Python 경로 추가
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    log_info "✅ Python 경로를 .bashrc에 추가했습니다."
fi

################################################################################
# 17. 프로젝트 빌드
################################################################################
log_section "17. 프로젝트 빌드"

if [ -d "$WORKSPACE_DIR/src" ] && [ "$(ls -A $WORKSPACE_DIR/src)" ]; then
    log_info "프로젝트 빌드를 시작합니다..."
    
    cd $WORKSPACE_DIR
    
    # 이전 빌드 정리 (선택)
    read -p "이전 빌드 결과를 삭제하시겠습니까? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf build devel .catkin_workspace
        log_info "이전 빌드 결과를 삭제했습니다."
    fi
    
    # 병렬 빌드 수 결정
    if [ "$TOTAL_MEM" -lt 4000 ]; then
        JOBS=1
        log_warn "메모리가 4GB 미만입니다. 단일 코어로 빌드합니다 (-j1)"
    else
        JOBS=2
        log_info "메모리가 충분합니다. 2개 코어로 빌드합니다 (-j2)"
    fi
    
    log_info "빌드 시작... (약 30-60분 소요 예상)"
    START_TIME=$(date +%s)
    
    # 빌드 실행
    if catkin_make -j$JOBS -DCMAKE_BUILD_TYPE=Release; then
        END_TIME=$(date +%s)
        BUILD_TIME=$((END_TIME - START_TIME))
        BUILD_MINUTES=$((BUILD_TIME / 60))
        BUILD_SECONDS=$((BUILD_TIME % 60))
        
        log_info "✅ 빌드 완료! (소요 시간: ${BUILD_MINUTES}분 ${BUILD_SECONDS}초)"
        
        # 환경 소싱
        source $WORKSPACE_DIR/devel/setup.bash
    else
        log_error "❌ 빌드 실패!"
        log_error "에러 로그를 확인하세요."
        exit 1
    fi
else
    log_warn "src 디렉토리가 비어 있습니다. 빌드를 건너뜁니다."
fi

################################################################################
# 18. 빌드 결과 확인
################################################################################
log_section "18. 빌드 결과 확인"

if [ -d "$WORKSPACE_DIR/devel/lib/happy_robo" ]; then
    log_info "✅ 빌드된 실행 파일:"
    ls -lh $WORKSPACE_DIR/devel/lib/happy_robo/ | grep -v "^d" | awk '{print "   " $9}'
else
    log_warn "happy_robo 실행 파일을 찾을 수 없습니다."
fi

################################################################################
# 19. 실행 스크립트 생성
################################################################################
log_section "19. 실행 스크립트 생성"

# run_robot.sh 생성
cat > $WORKSPACE_DIR/run_robot.sh << 'EOFSCRIPT'
#!/bin/bash

# 색상
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Happy Robot - 실행 스크립트${NC}"
echo -e "${BLUE}========================================${NC}\n"

# ROS 환경 소싱
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash
source ~/happy_ws/devel/setup.bash

echo -e "${GREEN}[1]${NC} roscore 실행"
echo -e "${GREEN}[2]${NC} vel_converter_sdk 실행 (메인 노드)"
echo -e "${GREEN}[3]${NC} FastAPI 서버 실행"
echo -e "${GREEN}[4]${NC} 제어 UI 실행"
echo -e "${GREEN}[5]${NC} Launch 파일 실행 (run.launch)"
echo -e "${GREEN}[6]${NC} 모든 노드 종료"
echo ""

read -p "선택 (1-6): " choice

case $choice in
    1)
        echo -e "${GREEN}roscore 실행 중...${NC}"
        roscore
        ;;
    2)
        echo -e "${GREEN}vel_converter_sdk 실행 중...${NC}"
        rosrun happy_robo vel_converter_sdk
        ;;
    3)
        echo -e "${GREEN}FastAPI 서버 실행 중...${NC}"
        rosrun happy_robo server_fastapi.py
        ;;
    4)
        echo -e "${GREEN}제어 UI 실행 중...${NC}"
        cd ~/happy_ws
        python3 robot_control_ui.py
        ;;
    5)
        echo -e "${GREEN}run.launch 실행 중...${NC}"
        roslaunch happy_robo run.launch
        ;;
    6)
        echo -e "${GREEN}모든 노드 종료 중...${NC}"
        killall -9 roscore rosmaster rosout 2>/dev/null || true
        echo -e "${GREEN}✅ 종료 완료${NC}"
        ;;
    *)
        echo "잘못된 선택입니다."
        ;;
esac
EOFSCRIPT

chmod +x $WORKSPACE_DIR/run_robot.sh
log_info "✅ 실행 스크립트 생성: $WORKSPACE_DIR/run_robot.sh"

################################################################################
# 20. 완료 및 요약
################################################################################
log_section "20. 설치 완료!"

cat << EOF

${GREEN}╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║          🎉 Jetson Nano 설정 완료! 🎉                    ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝${NC}

${BLUE}📊 설치 요약:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ ROS $ROS_DISTRO
✅ 빌드 도구 (catkin, cmake)
✅ 필수 ROS 패키지 (15+)
✅ Cartographer (SLAM)
✅ PCL (Point Cloud Library)
✅ Eigen3
✅ Python 패키지 (PyQt5, FastAPI 등)
✅ SWAP 메모리 8GB
✅ USB 시리얼 권한 설정
✅ Jetson Nano 전력 모드 최대
✅ 프로젝트 빌드
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

${BLUE}🚀 실행 방법:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

${GREEN}방법 1: 실행 스크립트 사용${NC}
  cd ~/happy_ws
  ./run_robot.sh

${GREEN}방법 2: 수동 실행${NC}
  # 터미널 1: ROS Master
  roscore
  
  # 터미널 2: 메인 노드
  rosrun happy_robo vel_converter_sdk
  
  # 터미널 3: 제어 UI
  cd ~/happy_ws
  python3 robot_control_ui.py

${GREEN}방법 3: Launch 파일${NC}
  roslaunch happy_robo run.launch

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

${BLUE}📚 문서:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  README_FINAL.md              - 시작 가이드
  COMPLETE_USAGE_GUIDE.md      - 완전 사용법
  QUICK_COMPILE_GUIDE.md       - 빠른 참조
  CODE_ANALYSIS_SUMMARY.md     - 코드 분석
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

${YELLOW}⚠️  중요 사항:${NC}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. USB 시리얼 권한 적용을 위해 로그아웃 후 재로그인하거나
   'newgrp dialout' 명령을 실행하세요.

2. 환경 변수를 적용하려면 새 터미널을 열거나
   'source ~/.bashrc' 명령을 실행하세요.

3. /dev/ttyUSB0 포트가 연결되어 있는지 확인하세요.
   ls -l /dev/ttyUSB*

4. 모터 컨트롤러 전원이 켜져 있는지 확인하세요.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

${GREEN}Happy Robot과 즐거운 시간 보내세요! 🤖✨${NC}

EOF

# 로그아웃 권장
log_warn ""
log_warn "설정을 완전히 적용하려면 로그아웃 후 재로그인하는 것을 권장합니다."
log_warn ""

exit 0
