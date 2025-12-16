# Happy Robot ROS 프로젝트 - CMake 및 코드 분석 요약

## 🏗️ 프로젝트 아키텍처

이 프로젝트는 **4륜 독립 조향 로봇 (Swerve Drive)** 을 위한 ROS 기반 자율주행 시스템입니다.

### 시스템 구성도

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS 마스터 노드                          │
│                   (master_node.py)                          │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┬──────────────────┐
        │                           │                  │
┌───────▼───────┐         ┌────────▼────────┐  ┌─────▼──────┐
│ vel_converter │         │  Cartographer   │  │ IMU/Sensor │
│     _sdk      │         │    (SLAM)       │  │   Nodes    │
└───────┬───────┘         └─────────────────┘  └────────────┘
        │
┌───────▼────────────────┐
│  WoorobotSDK (C++)     │
│  (모터 제어 라이브러리)  │
└───────┬────────────────┘
        │
┌───────▼───────┐
│ 시리얼 통신    │
│ /dev/ttyUSB0  │
└───────┬───────┘
        │
┌───────▼───────────────────┐
│  모터 컨트롤러 (8개)       │
│  - 구동 모터 4개 (RPM)    │
│  - 조향 모터 4개 (각도)   │
└───────────────────────────┘
```

---

## 📦 패키지 구조 분석

### 1. happy_robo (메인 패키지)

**위치**: `happy_ws/src/happy_robot/`

#### CMake 설정 분석

| 항목 | 값 | 설명 |
|------|-----|------|
| CMake 최소 버전 | 2.8.3 | 오래된 버전이지만 ROS Melodic 호환 |
| 프로젝트 이름 | happy_robo | ROS 패키지 이름 |
| C++ 표준 | C++14 | `-std=c++14` |
| 최적화 플래그 | `-O3` | 최대 최적화 (속도 우선) |
| 서브디렉터리 | WoorobotSDK | 모터 제어 SDK 포함 |

#### 의존성 매트릭스

| 카테고리 | 패키지 | 필수 여부 | 설명 |
|----------|--------|----------|------|
| **ROS 기본** | roscpp | ✅ | C++ ROS 클라이언트 |
| | rospy | ✅ | Python ROS 클라이언트 |
| | std_msgs | ✅ | 표준 메시지 |
| | geometry_msgs | ✅ | 기하학 메시지 |
| **변환/좌표** | tf | ✅ | 좌표 변환 프레임워크 |
| | tf2 | ✅ | 차세대 TF |
| | tf2_ros | ✅ | TF2 ROS 통합 |
| | tf2_sensor_msgs | ✅ | 센서 메시지 변환 |
| **센서** | laser_geometry | ✅ | 레이저 스캔 처리 |
| | sensor_msgs | ✅ | 센서 메시지 타입 |
| **SLAM** | cartographer_ros | ✅ | Google Cartographer |
| | cartographer_ros_msgs | ✅ | Cartographer 메시지 |
| **포인트클라우드** | pcl_ros | ✅ | PCL-ROS 통합 |
| **제어** | dynamic_reconfigure | ✅ | 동적 파라미터 조정 |
| **통신** | serial | ⚠️ | 시리얼 통신 (WoorobotSDK용) |
| **기타** | message_generation | ✅ | 메시지 생성 |
| | message_runtime | ✅ | 메시지 런타임 |

#### 외부 라이브러리

| 라이브러리 | 버전 요구사항 | 용도 |
|-----------|-------------|------|
| **Eigen3** | 없음 | 선형대수 연산 |
| **PCL** | 1.10 이상 | 3D 포인트클라우드 처리 |

#### 빌드 타겟 상세

##### C++ 실행 파일

| 타겟 이름 | 소스 파일 | 의존성 | 기능 |
|----------|----------|--------|------|
| **conversion** | src/conversion.cpp | catkin | 데이터 변환 유틸리티 |
| **vel_converter_sdk** | src/vel_converter_sdk.cpp | motor_controller_lib, woorobot_sdk | 속도 명령을 모터 제어로 변환 |
| **dynamic_map_publisher** | src/dynamic_map_publisher.cpp | catkin | 동적 맵 퍼블리싱 |
| **imu_receiver** | src/imu_receiver.cpp | catkin | IMU 데이터 수신 |
| **async_scan_merger_real_2d** | src/async_scan_merger_real_2d.cpp | catkin | 2D 레이저 스캔 비동기 병합 |
| **odom_publisher** | src/odom_publisher.cpp | catkin | Odometry 데이터 퍼블리싱 |
| **filter_pcd** | src/filter_pcd.cpp | catkin | 포인트클라우드 필터링 |

##### C++ 라이브러리

| 라이브러리 이름 | 소스 | 타입 | 용도 |
|-------------|------|------|------|
| **motor_controller_lib** | src/motor_controller.cpp | PRIVATE | 모터 제어 추상화 레이어 |

##### Python 스크립트

설치 위치: `${CATKIN_PACKAGE_BIN_DESTINATION}`

| 스크립트 | 기능 |
|---------|------|
| **master_node.py** | 메인 제어 노드 (22KB, 핵심 로직) |
| **obb_calculator.py** | Oriented Bounding Box 계산 |
| **mode_converter.py** | 주행 모드 변환 |
| **myactionlib.py** | 커스텀 액션 라이브러리 |
| **map_regenerater.py** | 맵 재생성 |
| **server_fastapi.py** | FastAPI 웹 서버 |
| **server_ws.py** | WebSocket 서버 |
| **record_rosbag.py** | ROSbag 기록 |

#### 커스텀 메시지 및 서비스

**메시지**:
```
AlignAndGoalPoseStamped.msg
├── geometry_msgs/PoseStamped goal_pose
└── (정렬 및 목표 위치 정보)
```

**서비스**:
```
TriggerWithCommand.srv
├── int32 command
├── float64 degree
├── ---
├── bool success
└── string message
```

---

### 2. WoorobotSDK (모터 제어 라이브러리)

**위치**: `happy_ws/src/happy_robot/WoorobotSDK/`

#### CMake 설정 분석

| 항목 | 값 | 설명 |
|------|-----|------|
| CMake 최소 버전 | 3.16 | ⚠️ 메인 패키지(2.8.3)보다 높음 |
| 프로젝트 이름 | woorobot_sdk | 독립 SDK |
| C++ 표준 | C++17 | ⚠️ 메인 패키지(C++14)보다 높음 |
| 라이브러리 타입 | SHARED | 공유 라이브러리 (.so) |
| PIC | ON | Position Independent Code |

#### 소스 구조

```
WoorobotSDK/
├── include/
│   └── SerialPortConnector.h       # 공개 API
├── woorobot_sdk_cpp/               # 구현부
│   ├── Controller.cpp
│   ├── Firmware.cpp/h
│   ├── MainCommConfig.cpp/h
│   ├── MotorConfig.cpp/h
│   ├── MotorControl.cpp/h
│   ├── MotorGain.cpp/h
│   ├── MotorLimit.cpp/h
│   ├── MotorParam.cpp/h
│   ├── MotorStatus.cpp/h
│   └── BoardConfig.h
├── example/cpp/
│   ├── main.cpp                    # 예제 실행 파일
│   └── config.cpp                  # 설정 예제
└── woorobot_sdk_python/            # Python 바인딩 (주석 처리)
    └── bindings.cpp
```

#### 클래스 다이어그램 (추론)

```
┌─────────────────────┐
│  SerialPortConnector│ (시리얼 통신 인터페이스)
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│    Controller       │ (메인 컨트롤러)
└──────────┬──────────┘
           │
    ┌──────┴──────┬──────────┬─────────┬─────────┐
    │             │          │         │         │
┌───▼───┐  ┌─────▼─────┐  ┌─▼──┐  ┌──▼──┐  ┌──▼───┐
│Firmware│  │MotorControl│  │Gain│  │Limit│  │Config│
└────────┘  └────────────┘  └────┘  └─────┘  └──────┘
```

#### 기능 요약

| 클래스 | 역할 | 주요 메서드 (추론) |
|--------|------|------------------|
| **Controller** | 전체 모터 제어 조율 | init(), setRPM(), getDegree() |
| **MotorControl** | 개별 모터 제어 | setSpeed(), setPosition() |
| **MotorGain** | PID 게인 관리 | setP(), setI(), setD() |
| **MotorLimit** | 속도/토크 제한 | setMaxSpeed(), setMaxTorque() |
| **MotorParam** | 모터 파라미터 | setGearRatio(), setEncoderResolution() |
| **MotorStatus** | 모터 상태 읽기 | getPosition(), getSpeed(), getCurrent() |
| **Firmware** | 펌웨어 통신 | sendCommand(), receiveResponse() |

---

### 3. jackal_control (제어 패키지)

**위치**: `happy_ws/src/happy_robot/control/`

#### CMake 설정

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(jackal_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)
catkin_package()

# Launch 파일 검증
roslaunch_add_file_check(launch/control.launch)
roslaunch_add_file_check(launch/teleop.launch)
```

- **매우 간단한 패키지**: Launch 파일만 제공
- **의존성**: roslaunch만 필요
- **역할**: Jackal 로봇 컨트롤러 설정 제공

---

## 🔍 핵심 코드 분석

### vel_converter_sdk.cpp 상세 분석

**파일**: `happy_ws/src/happy_robot/src/vel_converter_sdk.cpp` (720줄)

#### 클래스 구조

```cpp
class VelConverter {
public:
    VelConverter(ros::NodeHandle& n);
    ~VelConverter();
    
    // 콜백 함수
    void smoothSwerveDriveControllerCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void stopModeCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // 서비스 핸들러
    bool handlePing(...);
    bool handleBrake(...);
    bool handleMotorInit(...);
    bool handleTank(...);
    
    // 스레드 함수
    void outputMotorValuesLoop();       // 20Hz로 모터 상태 읽기
    void subscribeControlThread();      // ROS 메시지 처리
    
private:
    void outputMotorValuesSlow();
    void updateOdometry(std::vector<float> wheel_motor_values);
    void publishOdometry(const ros::Time& time);
};
```

#### 주요 기능 설명

##### 1. Swerve Drive 제어 알고리즘

```cpp
// 각 바퀴의 속도와 각도 계산 (4륜 독립 조향)
for (int i = 0; i < 4; i++) {
    double x = WHEEL_POSITIONS[i].first;   // 바퀴 위치
    double y = WHEEL_POSITIONS[i].second;
    
    // 바퀴 속도 성분 계산 (역기구학)
    double wheel_vx = vx - wz * y;
    double wheel_vy = vy + wz * x;
    
    // 속도 크기와 각도 계산
    double speed = sqrt(wheel_vx^2 + wheel_vy^2);
    double angle_rad = atan2(wheel_vy, wheel_vx);
    double angle_deg = angle_rad * 180.0 / PI;
    
    // RPM 변환
    double rpm = (speed / (2π * WHEEL_RADIUS)) * 60.0;
    
    // 각도 최적화: 90° 이상 회전 시 방향 반전
    if (|angle_deg| > 90.0) {
        angle_deg -= 180.0;
        rpm = -rpm;
    }
}
```

**핵심 개념**:
- **Inverse Kinematics**: 로봇 속도 → 각 바퀴 속도/각도
- **각도 최적화**: 조향각을 최소화하여 기계적 부담 감소
- **RPM 계산**: 선속도를 모터 RPM으로 변환

##### 2. Odometry 업데이트 (순기구학)

```cpp
void updateOdometry(std::vector<float> wheel_motor_values) {
    // Forward Kinematics: 각 바퀴 상태 → 로봇 속도
    for (int i = 0; i < 4; i++) {
        double wheel_rpm = wheel_motor_values[i];
        double wheel_degree = wheel_motor_values[i + 4];
        
        // 바퀴 선속도
        double wheel_linear_vel = wheel_rpm * WHEEL_RADIUS * (2π / 60.0);
        
        // x, y 성분 분해
        double wheel_vx = wheel_linear_vel * cos(wheel_degree * π / 180.0);
        double wheel_vy = wheel_linear_vel * sin(wheel_degree * π / 180.0);
    }
    
    // 평균 속도
    vx_ = vx_sum / 4.0;
    vy_ = vy_sum / 4.0;
    
    // 각속도 추정
    double omega_est = (vx_ - wheel_vx_vec[i]) / y;
    
    // 위치 적분
    x_ += (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
    y_ += (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
    theta_ += vtheta_ * dt;
}
```

##### 3. 멀티스레드 아키텍처

```cpp
int main() {
    VelConverter velConverter(nh);
    
    // 스레드 1: 모터 상태 읽기 (20Hz)
    std::thread loop_thread(&VelConverter::outputMotorValuesLoop, &velConverter);
    
    // 스레드 2: ROS 메시지 처리
    std::thread subscribe_thread(&VelConverter::subscribeControlThread, &velConverter);
    
    loop_thread.join();
    subscribe_thread.join();
}
```

**동기화 메커니즘**:
- `std::mutex odom_mutex`: Odometry 데이터 보호
- `std::mutex service_mutex`: 서비스 호출 직렬화
- `std::condition_variable cv`: 스레드 간 신호
- `wait_for_steer`: 조향각 안정화 대기

##### 4. 서비스 인터페이스

| 서비스 | 기능 | 파라미터 |
|--------|------|---------|
| **ping_service** | 8개 모터 통신 테스트 | - |
| **brake_service** | 브레이크 제어 | command: 0~4 |
| **motor_init_service** | 모터 초기화 | command: 0~8 (모터 ID) |
| **tank_service** | 탱크 모드 (미구현) | command, degree |

##### 5. 로봇 파라미터

```cpp
// 로봇 기하학
double WHEELBASE = 1.135;         // 전후 휠베이스 (m)
double TRACK_WIDTH = 0.8;         // 좌우 트랙 너비 (m)
double WHEEL_RADIUS = 0.102;      // 바퀴 반지름 (m)

// 바퀴 위치 (로봇 중심 기준)
WHEEL_POSITIONS = {
    {0.5675, 0.4},    // Front-left
    {0.5675, -0.4},   // Front-right
    {-0.5675, 0.4},   // Rear-left
    {-0.5675, -0.4}   // Rear-right
};

// 속도 제한
MAX_LINEAR_SPEED = 0.5;   // m/s
MAX_ANGULAR_SPEED = 0.5;  // rad/s

// 시리얼 통신
serial_port = "/dev/ttyUSB0";
speed = 1000000;  // 1 Mbps
motor_number = 8;
```

##### 6. 모터 제어 프로토콜 (하드코딩)

```cpp
// 모터 브레이크 명령
std::vector<uint8_t> motor_brake = {
    0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5E, 0x03, 0x00, 0x75, 0x00
};

// 외부 브레이크 해제 (48V)
std::vector<uint8_t> outter_non_brake = {
    0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x64, 0x25, 0x2B
};
```

---

## 🔧 Jetson Nano 최적화 포인트

### 1. 메모리 최적화

**문제점**:
- Cartographer는 메모리 집약적 (1GB 이상)
- PCL 처리도 메모리 소비 큼
- 4GB Jetson Nano는 SWAP 필수

**해결책**:
```cmake
# CMakeLists.txt에 최적화 플래그 추가
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")

# 디버그 심볼 제거
set(CMAKE_BUILD_TYPE Release)
```

### 2. C++ 표준 버전 충돌 해결

**문제**:
- WoorobotSDK: C++17 요구
- happy_robo: C++14 사용

**해결책**:
```cmake
# WoorobotSDK/CMakeLists.txt
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 메인 CMakeLists.txt
target_compile_features(vel_converter_sdk PRIVATE cxx_std_17)
```

### 3. 병렬 빌드 제한

```bash
# Jetson Nano는 4코어지만 메모리 부족 위험
catkin_make -j2  # 최대 2개 병렬

# 또는 메모리 제한
catkin build -j1 --mem-limit 80%
```

### 4. PCL 버전 다운그레이드

```cmake
# 원본
find_package(PCL 1.10 REQUIRED)

# Jetson Nano용 (Ubuntu 18.04는 PCL 1.8)
find_package(PCL 1.8 REQUIRED)
```

---

## 📊 성능 프로파일

### 실행 주기

| 노드/스레드 | 주기 | CPU 사용률 (예상) |
|------------|------|-----------------|
| outputMotorValuesLoop | 20Hz (50ms) | ~15% |
| subscribeControlThread | 이벤트 기반 | ~5% |
| Cartographer | 변동 | ~30-50% |
| master_node.py | 10Hz (추정) | ~10% |

### 메모리 사용량 (예상)

| 구성 요소 | RAM 사용량 |
|----------|----------|
| ROS Master | ~50MB |
| vel_converter_sdk | ~100MB |
| Cartographer | ~500MB-1GB |
| Python 노드들 | ~200MB |
| **총합** | **~1.5-2GB** |

⚠️ **4GB Jetson Nano에서 SWAP 필수**

---

## 🎯 컴파일 전략 요약

### 단계별 접근

1. **의존성 먼저**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **WoorobotSDK 단독 빌드**:
   ```bash
   catkin_make --only-pkg-with-deps woorobot_sdk -j1
   ```

3. **메인 패키지 빌드**:
   ```bash
   catkin_make --pkg happy_robo -j2
   ```

4. **전체 빌드**:
   ```bash
   catkin_make -j2
   ```

### 실패 시 대응

| 에러 타입 | 원인 | 해결 |
|----------|------|------|
| **OOM Killed** | 메모리 부족 | SWAP 증설, -j1 |
| **C++17 에러** | 컴파일러 구버전 | GCC 7.5+ 확인 |
| **PCL 버전** | 버전 불일치 | find_package 수정 |
| **Cartographer** | 패키지 없음 | 소스 빌드 |

---

## 📝 핵심 파일 체크리스트

빌드 전 반드시 확인할 파일:

- [ ] `happy_ws/src/happy_robot/CMakeLists.txt` - 메인 빌드 설정
- [ ] `happy_ws/src/happy_robot/package.xml` - 의존성 정의
- [ ] `happy_ws/src/happy_robot/WoorobotSDK/CMakeLists.txt` - SDK 빌드
- [ ] `happy_ws/src/happy_robot/src/vel_converter_sdk.cpp` - 핵심 제어 로직
- [ ] `happy_ws/src/happy_robot/script/master_node.py` - Python 제어 노드

---

**분석 완료일**: 2025-12-16  
**코드 라인 수**: ~3000줄 (C++), ~2000줄 (Python)  
**복잡도**: 중상 (멀티스레드, 역/순기구학, SLAM 통합)
