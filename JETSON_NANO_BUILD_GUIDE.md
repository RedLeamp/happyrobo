# Jetson Nano 컴파일 가이드 - Happy Robot ROS 프로젝트

## 📋 프로젝트 구조 분석

### 주요 구성 요소

```
happy_ws/
├── src/
│   ├── happy_robot/              # 메인 로봇 제어 패키지
│   │   ├── CMakeLists.txt        # 메인 빌드 설정
│   │   ├── package.xml           # ROS 패키지 의존성
│   │   ├── WoorobotSDK/          # 모터 제어 C++ SDK
│   │   │   ├── CMakeLists.txt    # SDK 빌드 설정 (C++17)
│   │   │   ├── include/          # 공개 헤더
│   │   │   ├── woorobot_sdk_cpp/ # SDK 구현
│   │   │   └── example/          # 예제 코드
│   │   ├── control/              # Jackal 컨트롤러
│   │   ├── src/                  # C++ 소스 코드
│   │   │   ├── vel_converter_sdk.cpp      # 속도 변환기
│   │   │   ├── conversion.cpp             # 데이터 변환
│   │   │   ├── dynamic_map_publisher.cpp  # 맵 퍼블리셔
│   │   │   ├── imu_receiver.cpp           # IMU 수신기
│   │   │   ├── async_scan_merger_real_2d.cpp
│   │   │   ├── odom_publisher.cpp         # Odometry 퍼블리셔
│   │   │   └── filter_pcd.cpp             # 포인트클라우드 필터
│   │   └── script/               # Python 스크립트
│   │       ├── master_node.py
│   │       ├── server_fastapi.py
│   │       └── ...
│   └── ros_imu_ws/              # IMU 워크스페이스
└── build/                        # 빌드 출력
```

## 🔍 CMake 및 ROS 코드 상세 분석

### 1. WoorobotSDK (C++ 모터 제어 라이브러리)

**파일**: `happy_ws/src/happy_robot/WoorobotSDK/CMakeLists.txt`

#### 주요 특징:
- **CMake 버전**: 3.16 이상 필요
- **C++ 표준**: C++17
- **Position Independent Code**: ON (공유 라이브러리용)
- **빌드 타입**: SHARED 라이브러리

```cmake
cmake_minimum_required(VERSION 3.16)
project(woorobot_sdk VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 공유 라이브러리 생성
add_library(woorobot_sdk SHARED ${SDK_SOURCES})

# 예제 실행 파일
add_executable(woorobot_example ${EXAMPLE_MAIN})
add_executable(woorobot_config ${EXAMPLE_CONFIG})
```

#### Jetson Nano 호환성:
- ✅ C++17 지원 (GCC 7.5+ 필요)
- ✅ 외부 의존성 없음 (시리얼 통신만 사용)
- ⚠️ Python 바인딩은 주석 처리됨 (pybind11)

### 2. 메인 Happy Robot 패키지

**파일**: `happy_ws/src/happy_robot/CMakeLists.txt`

#### CMake 설정:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(happy_robo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
```

#### ROS 의존성:
```cmake
find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  tf
  tf2
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  cartographer_ros          # SLAM 시스템
  cartographer_ros_msgs
  pcl_ros                   # 포인트클라우드
  message_generation
)
```

#### 외부 라이브러리:
```cmake
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)  # Point Cloud Library
```

#### 빌드 타겟:
1. **conversion** - 데이터 변환 노드
2. **motor_controller_lib** - 모터 컨트롤러 라이브러리
3. **vel_converter_sdk** - 속도 변환기 (WoorobotSDK 사용)
4. **dynamic_map_publisher** - 동적 맵 퍼블리셔
5. **imu_receiver** - IMU 데이터 수신기
6. **async_scan_merger_real_2d** - 2D 레이저 스캔 병합
7. **odom_publisher** - Odometry 퍼블리셔
8. **filter_pcd** - 포인트클라우드 필터

### 3. 커스텀 메시지 및 서비스

```cmake
add_message_files(
  FILES
  AlignAndGoalPoseStamped.msg
)

add_service_files(
  FILES
  TriggerWithCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

## 🎯 Jetson Nano에서 컴파일하는 방법

### 시스템 요구사항

#### 하드웨어:
- **Jetson Nano** (4GB 권장)
- **저장 공간**: 최소 10GB 이상 여유 공간
- **메모리**: 4GB RAM + SWAP 권장

#### 소프트웨어:
- **OS**: Ubuntu 18.04 (JetPack 4.x) 또는 Ubuntu 20.04 (JetPack 5.x)
- **ROS**: Melodic (18.04) 또는 Noetic (20.04)
- **CMake**: 3.16 이상
- **GCC**: 7.5 이상 (C++17 지원)

---

## 📦 Step 1: 기본 환경 설정

### 1.1 ROS 설치 (Ubuntu 18.04 - Melodic)

```bash
# ROS Melodic 설치
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full

# 환경 설정
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

### 1.2 ROS 설치 (Ubuntu 20.04 - Noetic)

```bash
# ROS Noetic 설치
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

### 1.3 빌드 도구 설치

```bash
# catkin 빌드 도구
sudo apt install -y python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# CMake 업그레이드 (3.16 이상 필요)
sudo apt install -y cmake

# CMake 버전 확인
cmake --version

# 만약 3.16 미만이면 수동 설치
# wget https://github.com/Kitware/CMake/releases/download/v3.22.0/cmake-3.22.0-linux-aarch64.sh
# chmod +x cmake-3.22.0-linux-aarch64.sh
# sudo ./cmake-3.22.0-linux-aarch64.sh --prefix=/usr/local --skip-license
```

---

## 📦 Step 2: 의존성 패키지 설치

### 2.1 필수 ROS 패키지

```bash
# 기본 ROS 패키지
sudo apt install -y \
  ros-${ROS_DISTRO}-tf \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-sensor-msgs \
  ros-${ROS_DISTRO}-laser-geometry \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-diff-drive-controller \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-controller \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-interactive-marker-twist-server \
  ros-${ROS_DISTRO}-dynamic-reconfigure

# 시리얼 통신
sudo apt install -y ros-${ROS_DISTRO}-serial

# ROS_DISTRO 변수 설정 (melodic 또는 noetic)
# export ROS_DISTRO=melodic  # 또는 noetic
```

### 2.2 Cartographer (SLAM) 설치

**중요**: Cartographer는 Jetson Nano에서 메모리 집약적이므로 주의가 필요합니다.

```bash
# Cartographer 설치 (Melodic)
sudo apt install -y \
  ros-melodic-cartographer \
  ros-melodic-cartographer-ros \
  ros-melodic-cartographer-ros-msgs

# 또는 Noetic
sudo apt install -y \
  ros-noetic-cartographer \
  ros-noetic-cartographer-ros \
  ros-noetic-cartographer-ros-msgs
```

**대안 (소스에서 빌드)**: 패키지 관리자에 없는 경우

```bash
cd ~/happy_ws/src
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git

# 의존성 설치
sudo apt install -y \
  libceres-dev \
  libcairo2-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  liblua5.2-dev \
  libprotobuf-dev \
  protobuf-compiler \
  python3-sphinx
```

### 2.3 Point Cloud Library (PCL) 설치

```bash
# PCL 1.10 이상 필요
sudo apt install -y \
  libpcl-dev \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-pcl-conversions

# PCL 버전 확인
pkg-config --modversion pcl_common
```

**PCL 1.10 미만인 경우 (소스 빌드)**:

```bash
# 의존성
sudo apt install -y \
  libeigen3-dev \
  libflann-dev \
  libvtk6-dev \
  libboost-all-dev

# PCL 소스 빌드
cd /tmp
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.11.0.tar.gz
tar -xvf pcl-1.11.0.tar.gz
cd pcl-pcl-1.11.0
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 2.4 Eigen3 설치

```bash
sudo apt install -y libeigen3-dev

# 버전 확인
pkg-config --modversion eigen3
```

### 2.5 Python 의존성

```bash
# Python 3 패키지
sudo apt install -y \
  python3-pip \
  python3-numpy \
  python3-scipy

# FastAPI 서버용
pip3 install fastapi uvicorn websockets
```

---

## 📦 Step 3: SWAP 메모리 증설 (필수)

Jetson Nano는 메모리가 제한적이므로 컴파일 시 SWAP을 늘려야 합니다.

```bash
# 현재 SWAP 확인
free -h

# SWAP 파일 생성 (8GB 권장)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 영구 적용
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# 확인
free -h
```

---

## 🔨 Step 4: 프로젝트 빌드

### 4.1 워크스페이스 준비

```bash
cd ~/happy_ws

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 없는 패키지는 수동으로 처리
```

### 4.2 컴파일 옵션 조정 (메모리 최적화)

Jetson Nano는 메모리가 제한적이므로 병렬 빌드를 제한해야 합니다.

```bash
# 단일 코어로 빌드 (안전)
catkin_make -j1

# 또는 2개 코어로 빌드 (약간 빠름)
catkin_make -j2

# catkin_tools 사용 시
catkin build -j1 --mem-limit 80%
```

### 4.3 빌드 순서 (의존성 고려)

```bash
# 1단계: WoorobotSDK 먼저 빌드
cd ~/happy_ws
catkin_make --only-pkg-with-deps woorobot_sdk -j1

# 2단계: 메인 패키지 빌드
catkin_make -j1

# 또는 선택적 빌드
catkin_make --pkg happy_robo -j1
```

### 4.4 환경 설정

```bash
# 빌드 후 환경 소싱
source ~/happy_ws/devel/setup.bash

# .bashrc에 추가 (영구 적용)
echo "source ~/happy_ws/devel/setup.bash" >> ~/.bashrc
```

---

## ⚠️ 알려진 문제 및 해결 방법

### 문제 1: C++17 컴파일 에러

**증상**:
```
error: 'filesystem' is not a member of 'std'
```

**해결**:
```bash
# GCC 버전 확인
gcc --version

# GCC 7.5 이상이면 OK
# 만약 낮으면 업그레이드 필요 (Ubuntu 18.04는 7.5 기본 제공)
```

### 문제 2: Cartographer 빌드 실패

**증상**:
```
Could not find a package configuration file provided by "cartographer_ros"
```

**해결**:
```bash
# 패키지 재설치
sudo apt remove ros-${ROS_DISTRO}-cartographer*
sudo apt install -y ros-${ROS_DISTRO}-cartographer-ros

# 또는 소스에서 빌드 (위 2.2 참조)
```

### 문제 3: PCL 버전 불일치

**증상**:
```
Could not find a package configuration file provided by "PCL" with any of the following versions: 1.10
```

**해결**:
```bash
# PCL 재설치
sudo apt install -y libpcl-dev

# 버전 확인 후 CMakeLists.txt 수정
find_package(PCL 1.8 REQUIRED)  # 1.10 → 1.8로 낮춤
```

### 문제 4: 메모리 부족 (OOM)

**증상**:
```
c++: fatal error: Killed signal terminated program cc1plus
```

**해결**:
```bash
# SWAP 증설 (위 Step 3 참조)
# 병렬 빌드 제한
catkin_make -j1

# 불필요한 프로세스 종료
sudo systemctl stop docker  # 만약 실행 중이면
```

### 문제 5: USB 시리얼 권한 문제

**증상**:
```
Permission denied: '/dev/ttyUSB0'
```

**해결**:
```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -aG dialout $USER

# 로그아웃 후 재로그인
# 또는 즉시 적용
newgrp dialout

# 권한 확인
ls -l /dev/ttyUSB0
```

---

## 🚀 빌드 후 실행 테스트

### 1. WoorobotSDK 테스트

```bash
# 예제 실행 파일
cd ~/happy_ws
rosrun happy_robo woorobot_example

# 또는 직접 실행
./devel/lib/woorobot_sdk/woorobot_example
```

### 2. 메인 노드 실행

```bash
# roscore 실행
roscore &

# 속도 변환기 실행
rosrun happy_robo vel_converter_sdk

# Python 마스터 노드
rosrun happy_robo master_node.py
```

### 3. Launch 파일 실행

```bash
# Launch 파일이 있다면
roslaunch happy_robo <launch_file_name>.launch
```

---

## 📊 성능 최적화 팁 (Jetson Nano)

### 1. 전력 모드 변경

```bash
# 최대 성능 모드 (10W)
sudo nvpmodel -m 0
sudo jetson_clocks

# 전력 모드 확인
sudo nvpmodel -q
```

### 2. CPU 주파수 고정

```bash
# CPU 거버너를 performance로 설정
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### 3. 불필요한 서비스 비활성화

```bash
# GUI 비활성화 (SSH 사용 시)
sudo systemctl set-default multi-user.target
sudo reboot

# GUI 재활성화 (필요 시)
sudo systemctl set-default graphical.target
```

### 4. 컴파일 최적화 플래그

**CMakeLists.txt 수정**:
```cmake
# 최적화 플래그 추가
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native")
```

---

## 📝 요약 체크리스트

- [ ] ROS Melodic/Noetic 설치
- [ ] CMake 3.16+ 설치
- [ ] GCC 7.5+ 확인
- [ ] SWAP 메모리 8GB 이상 설정
- [ ] 필수 ROS 패키지 설치 (tf, tf2, laser_geometry, serial 등)
- [ ] Cartographer 설치
- [ ] PCL 1.10+ 설치 (또는 1.8로 다운그레이드)
- [ ] Eigen3 설치
- [ ] USB 시리얼 권한 설정 (dialout 그룹)
- [ ] catkin_make -j1 로 빌드
- [ ] 환경 소싱 (source ~/happy_ws/devel/setup.bash)
- [ ] 전력 모드 최대 성능으로 설정

---

## 🔧 추가 리소스

### 공식 문서
- [ROS Melodic 설치 가이드](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS Noetic 설치 가이드](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Jetson Nano 개발자 가이드](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)

### 문제 해결 커뮤니티
- [ROS Answers](https://answers.ros.org/)
- [Jetson Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)

---

## 📞 기술 지원

빌드 중 문제가 발생하면 다음 정보를 수집하세요:

```bash
# 시스템 정보
uname -a
lsb_release -a
cat /etc/nv_tegra_release  # Jetson 정보

# ROS 버전
rosversion -d

# CMake 버전
cmake --version

# GCC 버전
gcc --version

# 빌드 로그
catkin_make 2>&1 | tee build_log.txt
```

---

**작성일**: 2025-12-16  
**대상 플랫폼**: Jetson Nano (Ubuntu 18.04/20.04)  
**ROS 버전**: Melodic / Noetic
