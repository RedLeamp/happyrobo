# Jetson Nano 빠른 컴파일 가이드

## 🚀 전체 명령어 요약 (복사해서 실행)

### Ubuntu 18.04 (ROS Melodic) 사용 시

```bash
# ============================================================
# Step 1: ROS 배포판 설정
# ============================================================
export ROS_DISTRO=melodic

# ============================================================
# Step 2: 시스템 업데이트
# ============================================================
sudo apt update
sudo apt upgrade -y

# ============================================================
# Step 3: ROS Melodic 설치
# ============================================================
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full

# rosdep 초기화
sudo rosdep init
rosdep update

# 환경 설정
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash

# ============================================================
# Step 4: 빌드 도구 설치
# ============================================================
sudo apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git

# ============================================================
# Step 5: 필수 ROS 패키지 설치
# ============================================================
sudo apt install -y \
    ros-melodic-tf \
    ros-melodic-tf2 \
    ros-melodic-tf2-ros \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-laser-geometry \
    ros-melodic-controller-manager \
    ros-melodic-diff-drive-controller \
    ros-melodic-robot-state-publisher \
    ros-melodic-joint-state-controller \
    ros-melodic-robot-localization \
    ros-melodic-teleop-twist-joy \
    ros-melodic-twist-mux \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-dynamic-reconfigure \
    ros-melodic-serial

# ============================================================
# Step 6: Cartographer (SLAM) 설치
# ============================================================
sudo apt install -y \
    ros-melodic-cartographer \
    ros-melodic-cartographer-ros \
    ros-melodic-cartographer-ros-msgs

# ============================================================
# Step 7: PCL (Point Cloud Library) 설치
# ============================================================
sudo apt install -y \
    libpcl-dev \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions

# ============================================================
# Step 8: Eigen3 설치
# ============================================================
sudo apt install -y libeigen3-dev

# ============================================================
# Step 9: Python 의존성 설치
# ============================================================
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy

pip3 install --user fastapi uvicorn websockets

# ============================================================
# Step 10: SWAP 메모리 설정 (매우 중요!)
# ============================================================
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# SWAP 확인
free -h

# ============================================================
# Step 11: USB 시리얼 포트 권한 설정
# ============================================================
sudo usermod -aG dialout $USER
# ⚠️  로그아웃 후 재로그인 필요 (또는 'newgrp dialout' 실행)

# ============================================================
# Step 12: Jetson Nano 전력 모드 최대로 설정
# ============================================================
sudo nvpmodel -m 0
sudo jetson_clocks

# ============================================================
# Step 13: 워크스페이스로 이동
# ============================================================
cd ~/happy_ws

# ============================================================
# Step 14: rosdep 의존성 자동 설치
# ============================================================
rosdep install --from-paths src --ignore-src -r -y

# ============================================================
# Step 15: PCL 버전 확인 (1.10 미만이면 수정 필요)
# ============================================================
pkg-config --modversion pcl_common

# 만약 PCL이 1.8이면 CMakeLists.txt 수정
# find_package(PCL 1.10 REQUIRED) → find_package(PCL 1.8 REQUIRED)
# sed -i 's/find_package(PCL 1\.10 REQUIRED)/find_package(PCL 1.8 REQUIRED)/g' src/happy_robot/CMakeLists.txt

# ============================================================
# Step 16: 빌드 시작! (30-60분 소요)
# ============================================================
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Step 17: 환경 설정
# ============================================================
echo "source ~/happy_ws/devel/setup.bash" >> ~/.bashrc
source ~/happy_ws/devel/setup.bash

# ============================================================
# 완료! 🎉
# ============================================================
echo "컴파일 완료!"
```

---

### Ubuntu 20.04 (ROS Noetic) 사용 시

```bash
# ============================================================
# Step 1: ROS 배포판 설정
# ============================================================
export ROS_DISTRO=noetic

# ============================================================
# Step 2: 시스템 업데이트
# ============================================================
sudo apt update
sudo apt upgrade -y

# ============================================================
# Step 3: ROS Noetic 설치
# ============================================================
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# rosdep 초기化
sudo rosdep init
rosdep update

# 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

# ============================================================
# Step 4: 빌드 도구 설치
# ============================================================
sudo apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git

# ============================================================
# Step 5: 필수 ROS 패키지 설치
# ============================================================
sudo apt install -y \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-laser-geometry \
    ros-noetic-controller-manager \
    ros-noetic-diff-drive-controller \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-controller \
    ros-noetic-robot-localization \
    ros-noetic-teleop-twist-joy \
    ros-noetic-twist-mux \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-serial

# ============================================================
# Step 6: Cartographer (SLAM) 설치
# ============================================================
sudo apt install -y \
    ros-noetic-cartographer \
    ros-noetic-cartographer-ros \
    ros-noetic-cartographer-ros-msgs

# ============================================================
# Step 7: PCL (Point Cloud Library) 설치
# ============================================================
sudo apt install -y \
    libpcl-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions

# ============================================================
# Step 8: Eigen3 설치
# ============================================================
sudo apt install -y libeigen3-dev

# ============================================================
# Step 9: Python 의존성 설치
# ============================================================
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy

pip3 install --user fastapi uvicorn websockets

# ============================================================
# Step 10: SWAP 메모리 설정 (매우 중요!)
# ============================================================
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# SWAP 확인
free -h

# ============================================================
# Step 11: USB 시리얼 포트 권한 설정
# ============================================================
sudo usermod -aG dialout $USER
# ⚠️  로그아웃 후 재로그인 필요 (또는 'newgrp dialout' 실행)

# ============================================================
# Step 12: Jetson Nano 전력 모드 최대로 설정
# ============================================================
sudo nvpmodel -m 0
sudo jetson_clocks

# ============================================================
# Step 13: 워크스페이스로 이동
# ============================================================
cd ~/happy_ws

# ============================================================
# Step 14: rosdep 의존성 자동 설치
# ============================================================
rosdep install --from-paths src --ignore-src -r -y

# ============================================================
# Step 15: PCL 버전 확인
# ============================================================
pkg-config --modversion pcl_common

# ============================================================
# Step 16: 빌드 시작! (30-60분 소요)
# ============================================================
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Step 17: 환경 설정
# ============================================================
echo "source ~/happy_ws/devel/setup.bash" >> ~/.bashrc
source ~/happy_ws/devel/setup.bash

# ============================================================
# 완료! 🎉
# ============================================================
echo "컴파일 완료!"
```

---

## 🔥 초간단 버전 (의존성이 모두 설치되어 있는 경우)

```bash
# 1. SWAP 설정
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 2. 전력 모드 최대
sudo nvpmodel -m 0
sudo jetson_clocks

# 3. 워크스페이스로 이동
cd ~/happy_ws

# 4. 빌드
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release

# 5. 환경 소싱
source devel/setup.bash
```

---

## 📋 빌드 옵션 설명

### 병렬 빌드 옵션

| 옵션 | 메모리 사용량 | 빌드 시간 | 권장 사용 |
|------|-------------|----------|---------|
| `-j1` | 낮음 | 느림 (60분) | 2GB 이하 RAM |
| `-j2` | 중간 | 보통 (40분) | 4GB RAM + SWAP |
| `-j4` | 높음 | 빠름 (30분) | 8GB+ RAM (비추천 Jetson) |

### 빌드 타입 옵션

| 옵션 | 설명 | 최적화 | 디버그 | 권장 |
|------|------|-------|-------|------|
| `-DCMAKE_BUILD_TYPE=Release` | 릴리즈 빌드 | ✅ O3 | ❌ | 실제 사용 |
| `-DCMAKE_BUILD_TYPE=Debug` | 디버그 빌드 | ❌ O0 | ✅ | 개발 중 |
| (기본값) | 기본 빌드 | ⚠️ O2 | ⚠️ | - |

---

## 🛠️ 빌드 후 실행 명령어

### 1. ROS 마스터 실행

```bash
# 터미널 1
roscore
```

### 2. 속도 변환기 노드 실행

```bash
# 터미널 2
source ~/happy_ws/devel/setup.bash
rosrun happy_robo vel_converter_sdk
```

### 3. Python 마스터 노드 실행

```bash
# 터미널 3
source ~/happy_ws/devel/setup.bash
rosrun happy_robo master_node.py
```

### 4. Launch 파일 실행 (있는 경우)

```bash
source ~/happy_ws/devel/setup.bash
roslaunch happy_robo <launch_file_name>.launch
```

---

## ⚠️ 자주 발생하는 에러 해결

### 1. 메모리 부족 (OOM Killed)

```bash
# 증상: c++: fatal error: Killed signal terminated program cc1plus

# 해결:
# 1) SWAP 확인
free -h

# 2) SWAP 재설정
sudo swapoff /swapfile
sudo fallocate -l 10G /swapfile  # 10GB로 증가
sudo mkswap /swapfile
sudo swapon /swapfile

# 3) 단일 코어로 재빌드
catkin_make clean
catkin_make -j1
```

### 2. PCL 버전 에러

```bash
# 증상: Could not find a package configuration file provided by "PCL" with any of the following versions: 1.10

# 해결:
# PCL 버전 확인
pkg-config --modversion pcl_common

# CMakeLists.txt 수정
cd ~/happy_ws/src/happy_robot
sed -i 's/find_package(PCL 1\.10 REQUIRED)/find_package(PCL 1.8 REQUIRED)/g' CMakeLists.txt

# 재빌드
cd ~/happy_ws
catkin_make -j1
```

### 3. Cartographer 없음

```bash
# 증상: Could not find a package configuration file provided by "cartographer_ros"

# 해결 1: 패키지 설치
sudo apt install -y ros-$ROS_DISTRO-cartographer-ros

# 해결 2: 소스에서 빌드 (메모리 많이 필요)
cd ~/happy_ws/src
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git
cd ~/happy_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j1
```

### 4. USB 시리얼 권한 에러

```bash
# 증상: Permission denied: '/dev/ttyUSB0'

# 해결:
sudo usermod -aG dialout $USER
newgrp dialout  # 또는 로그아웃 후 재로그인

# USB 장치 확인
ls -l /dev/ttyUSB*
```

### 5. C++17 컴파일 에러

```bash
# 증상: error: 'filesystem' is not a member of 'std'

# 해결:
# GCC 버전 확인
gcc --version

# GCC 7.5 이상이어야 함
# Ubuntu 18.04는 기본적으로 7.5 제공
```

---

## 📊 빌드 진행 상황 모니터링

### 빌드 중 메모리 사용량 확인

```bash
# 다른 터미널에서 실행
watch -n 1 free -h
```

### 빌드 중 CPU 사용률 확인

```bash
htop
```

### 빌드 로그 저장

```bash
catkin_make -j1 2>&1 | tee build_log.txt
```

---

## 🎯 체크리스트

빌드 전 확인사항:

- [ ] Ubuntu 버전 확인 (18.04 또는 20.04)
- [ ] ROS 설치 완료 (Melodic 또는 Noetic)
- [ ] CMake 3.16+ 설치
- [ ] GCC 7.5+ 설치
- [ ] SWAP 8GB 이상 설정
- [ ] 필수 ROS 패키지 설치
- [ ] Cartographer 설치
- [ ] PCL 설치
- [ ] Eigen3 설치
- [ ] USB 시리얼 권한 설정
- [ ] Jetson Nano 전력 모드 최대

빌드 후 확인사항:

- [ ] `devel/lib/happy_robo/` 디렉토리 존재
- [ ] 실행 파일들 생성 확인
- [ ] `source devel/setup.bash` 실행
- [ ] `rospack find happy_robo` 동작
- [ ] `/dev/ttyUSB0` 장치 연결 확인

---

## 📞 추가 도움말

더 자세한 정보는 다음 문서를 참조하세요:

- **JETSON_NANO_BUILD_GUIDE.md**: 상세한 빌드 가이드 (Step-by-step)
- **CODE_ANALYSIS_SUMMARY.md**: 코드 구조 및 CMake 분석
- **JETSON_NANO_COMPILE_COMMANDS.sh**: 자동 빌드 스크립트

---

**마지막 업데이트**: 2025-12-16  
**대상 플랫폼**: Jetson Nano (Ubuntu 18.04/20.04)  
**ROS 버전**: Melodic / Noetic
