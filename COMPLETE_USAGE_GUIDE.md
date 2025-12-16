# Happy Robot 완전 사용 가이드

## 📋 목차

1. [저장소 분석](#1-저장소-분석)
2. [컴파일 방법](#2-컴파일-방법)
3. [실행 방법](#3-실행-방법)
4. [제어 UI 프로그램 사용](#4-제어-ui-프로그램-사용)
5. [자동 반복 동작 설정](#5-자동-반복-동작-설정)
6. [문제 해결](#6-문제-해결)

---

## 1. 저장소 분석

### 프로젝트 구조

```
happy_ws/
├── src/
│   ├── happy_robot/                    # 메인 패키지
│   │   ├── CMakeLists.txt             # 빌드 설정
│   │   ├── package.xml                # 패키지 정보
│   │   ├── src/                       # C++ 소스
│   │   │   ├── vel_converter_sdk.cpp  # 속도 변환 (핵심)
│   │   │   ├── odom_publisher.cpp     # Odometry
│   │   │   ├── imu_receiver.cpp       # IMU
│   │   │   └── ...
│   │   ├── script/                    # Python 스크립트
│   │   │   ├── master_node.py         # 메인 노드
│   │   │   ├── server_fastapi.py      # FastAPI 서버
│   │   │   └── ...
│   │   ├── launch/                    # Launch 파일
│   │   │   ├── run.launch             # 실제 로봇 실행
│   │   │   ├── happy.launch           # 시뮬레이션
│   │   │   └── ...
│   │   ├── WoorobotSDK/               # 모터 제어 SDK
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include/
│   │   │   └── woorobot_sdk_cpp/
│   │   ├── config/                    # 설정 파일
│   │   ├── maps/                      # 맵 파일
│   │   └── urdf/                      # 로봇 모델
│   └── ros_imu_ws/                    # IMU 워크스페이스
└── build/                             # 빌드 출력
```

### 핵심 구성 요소

#### 1. C++ 노드 (8개)

| 노드 | 기능 | 중요도 |
|------|------|--------|
| **vel_converter_sdk** | 속도 명령을 모터 제어로 변환 | ⭐⭐⭐⭐⭐ |
| **odom_publisher** | Odometry 발행 | ⭐⭐⭐⭐ |
| **imu_receiver** | IMU 데이터 수신 | ⭐⭐⭐ |
| **dynamic_map_publisher** | 동적 맵 발행 | ⭐⭐⭐ |
| **async_scan_merger_real_2d** | 레이저 스캔 병합 | ⭐⭐⭐ |
| **filter_pcd** | 포인트클라우드 필터링 | ⭐⭐ |
| **conversion** | 데이터 변환 | ⭐⭐ |

#### 2. Python 노드 (주요 8개)

| 노드 | 기능 | 포트/토픽 |
|------|------|----------|
| **master_node.py** | 메인 제어 로직 | - |
| **server_fastapi.py** | REST API 서버 | 8000 |
| **server_ws.py** | WebSocket 서버 | 9000 |
| **obb_calculator.py** | OBB 계산 (주차) | - |
| **mode_converter.py** | 모드 변환 | - |
| **myactionlib.py** | 액션 클라이언트 | - |

#### 3. ROS 토픽 및 서비스

**주요 토픽:**
```
/cmd_vel              (geometry_msgs/Twist)      # 속도 명령
/odom/raw             (nav_msgs/Odometry)        # Odometry
/merged/scan          (sensor_msgs/LaserScan)    # 레이저 스캔
/control_mode         (std_msgs/Bool)            # 제어 모드
/stop_mode            (std_msgs/Bool)            # 정지 모드
```

**주요 서비스:**
```
/ping_service         (std_srvs/Trigger)                # 모터 Ping
/brake_service        (happy_robo/TriggerWithCommand)   # 브레이크
/motor_init_service   (happy_robo/TriggerWithCommand)   # 모터 초기화
/tank_service         (happy_robo/TriggerWithCommand)   # 탱크 제어
```

---

## 2. 컴파일 방법

### 방법 1: 자동 스크립트 (권장)

```bash
# 1. 스크립트 실행
cd ~/happy_ws
chmod +x JETSON_NANO_COMPILE_COMMANDS.sh
./JETSON_NANO_COMPILE_COMMANDS.sh
```

이 스크립트는 다음을 자동으로 수행합니다:
- Ubuntu 버전 감지
- ROS 설치 (없는 경우)
- 의존성 설치
- SWAP 메모리 설정
- 빌드 실행
- 환경 설정

### 방법 2: 수동 컴파일

#### Step 1: 의존성 설치

```bash
# ROS 환경 소싱
source /opt/ros/melodic/setup.bash  # 또는 noetic

# 필수 패키지
sudo apt install -y \
    ros-melodic-tf \
    ros-melodic-cartographer-ros \
    ros-melodic-pcl-ros \
    libpcl-dev \
    libeigen3-dev \
    ros-melodic-serial

# Python 패키지
pip3 install --user fastapi uvicorn websockets scipy PyQt5
```

#### Step 2: SWAP 설정 (필수!)

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

#### Step 3: 빌드

```bash
cd ~/happy_ws

# 클린 빌드 (권장)
rm -rf build devel

# 빌드 실행
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release

# 환경 소싱
source devel/setup.bash
echo "source ~/happy_ws/devel/setup.bash" >> ~/.bashrc
```

#### Step 4: 빌드 확인

```bash
# 실행 파일 확인
ls devel/lib/happy_robo/

# 패키지 확인
rospack find happy_robo

# 출력: /home/user/happy_ws/src/happy_robot (성공)
```

---

## 3. 실행 방법

### 3.1 기본 실행 (수동)

#### 터미널 1: ROS Master

```bash
roscore
```

#### 터미널 2: 속도 변환기 (핵심 노드)

```bash
source ~/happy_ws/devel/setup.bash
rosrun happy_robo vel_converter_sdk
```

**출력 예시:**
```
Robot twist controller is running...
✅ Motor controller initialized
Serial port: /dev/ttyUSB0 @ 1000000 baud
Waiting for cmd_vel...
```

#### 터미널 3: FastAPI 서버 (선택)

```bash
source ~/happy_ws/devel/setup.bash
rosrun happy_robo server_fastapi.py
```

**접속:** http://localhost:8000

#### 터미널 4: 제어 UI (새로 만든 프로그램)

```bash
cd ~/happy_ws
python3 robot_control_ui.py
```

### 3.2 Launch 파일 실행

#### 실제 로봇 실행

```bash
source ~/happy_ws/devel/setup.bash
roslaunch happy_robo run.launch
```

**이 명령은 다음을 실행합니다:**
- robot_state_publisher
- dynamic_map_publisher
- async_scan_merger_real_2d
- cyglidar_node (레이저 센서)
- rviz (시각화)

#### 시뮬레이션 실행

```bash
roslaunch happy_robo happy.launch
```

**Gazebo 시뮬레이터가 실행됩니다**

---

## 4. 제어 UI 프로그램 사용

### 4.1 설치

```bash
# PyQt5 설치
pip3 install PyQt5

# UI 프로그램 복사
cp robot_control_ui.py ~/happy_ws/
cd ~/happy_ws
chmod +x robot_control_ui.py
```

### 4.2 실행

```bash
# 방법 1: Python 직접 실행
python3 robot_control_ui.py

# 방법 2: 실행 파일로 실행
./robot_control_ui.py
```

### 4.3 UI 구성

#### 탭 1: 기본 제어

```
┌─────────────────────────────────┐
│  속도 설정                       │
│  ┌─────────────────────────┐   │
│  │ 선속도: [========] 0.20  │   │
│  │ 각속도: [========] 0.20  │   │
│  └─────────────────────────┘   │
│                                  │
│  방향 제어                       │
│  ┌─────────────────────────┐   │
│  │  ↖      ↑      ↗        │   │
│  │         전진             │   │
│  │  ←      ■      →        │   │
│  │   좌    정지    우       │   │
│  │  ↙      ↓      ↘        │   │
│  │         후진             │   │
│  │  ↶             ↷        │   │
│  │ 좌회전        우회전     │   │
│  └─────────────────────────┘   │
└─────────────────────────────────┘
```

**사용법:**
1. 속도 슬라이더로 원하는 속도 설정
2. 방향 버튼 클릭 (마우스 누르는 동안 이동)
3. 버튼에서 손을 떼면 자동 정지

#### 탭 2: 자동 반복

```
┌─────────────────────────────────┐
│  좌우 반복 동작 설정             │
│  ┌─────────────────────────┐   │
│  │ 동작 방향:  [좌우 ▼]    │   │
│  │ 이동 거리:  [1.0] m     │   │
│  │ 반복 횟수:  [5]         │   │
│  │ 대기 시간:  [1.0] 초    │   │
│  │ 속도:       [0.2] m/s   │   │
│  └─────────────────────────┘   │
│                                  │
│  진행 상태                       │
│  [████████░░░░░░] 50%           │
│  반복 3/5                        │
│                                  │
│  [🚀 반복 시작] [⏹ 반복 중지]   │
└─────────────────────────────────┘
```

**동작 방식:**
1. **좌우 모드**: 좌 → 우 → 좌 → 우 반복
2. **전후 모드**: 전진 → 후진 → 전진 반복
3. **회전 모드**: 좌회전 → 우회전 반복

**예시 시나리오:**
```python
# 설정: 좌우, 1m, 5회, 1초 대기, 0.2m/s
1. 우측으로 1m 이동 (5초)
2. 1초 대기
3. 좌측으로 1m 이동 (5초)
4. 1초 대기
5. 위 과정을 5회 반복
```

#### 탭 3: 고급 설정

```
┌─────────────────────────────────┐
│  모터 상태 및 제어               │
│  ┌─────────────────────────┐   │
│  │ [🔍 모터 Ping 테스트]    │   │
│  │                          │   │
│  │ 모터 ID: [1 ▲▼]         │   │
│  │ [🔧 모터 초기화]         │   │
│  │                          │   │
│  │ 브레이크: [선택 ▼]       │   │
│  │ [🛑 브레이크 실행]       │   │
│  └─────────────────────────┘   │
│                                  │
│  주행 모드                       │
│  ┌─────────────────────────┐   │
│  │ ☑ 수직 모드              │   │
│  │ ☐ 정지 모드              │   │
│  └─────────────────────────┘   │
└─────────────────────────────────┘
```

**기능:**
- **Ping 테스트**: 8개 모터 통신 확인
- **모터 초기화**: 개별 모터 초기화 (1~8번)
- **브레이크**: 
  - 0: Motor Brake (모터 브레이크)
  - 1: Motor Non-Brake (모터 브레이크 해제)
  - 2: Outer Brake (외부 브레이크)
  - 3: Outer Non-Brake (외부 브레이크 해제)
  - 4: Reset Pose (포즈 리셋)

### 4.4 모니터링 패널

```
┌─────────────────────────┐
│  로봇 상태              │
│  ┌─────────────────┐   │
│  │ 위치 X:  1.23 m │   │
│  │ 위치 Y: -0.45 m │   │
│  │ 방향:    45.0°  │   │
│  │ 최소 거리: 2.1 m│   │
│  └─────────────────┘   │
│                          │
│  현재 명령              │
│  ┌─────────────────┐   │
│  │ 선속도: 0.20 m/s│   │
│  │ 각속도: 0.00 rd/s│  │
│  └─────────────────┘   │
│                          │
│  로그                   │
│  ┌─────────────────┐   │
│  │ [12:34:56] ✅   │   │
│  │ 방향: forward   │   │
│  │ [12:34:57] 🚀   │   │
│  │ 자동 반복 시작  │   │
│  │                  │   │
│  └─────────────────┘   │
│  [로그 지우기]          │
└─────────────────────────┘
```

---

## 5. 자동 반복 동작 설정

### 5.1 좌우 반복 (패트롤)

**시나리오**: 복도를 좌우로 왔다갔다

```python
동작 방향: 좌우
이동 거리: 2.0 m
반복 횟수: 10
대기 시간: 0.5 초
속도: 0.3 m/s
```

**실행 순서:**
```
시작 위치 (0, 0)
    ↓
좌로 2m 이동 → (0, 2) [6.7초]
    ↓ 0.5초 대기
우로 2m 이동 → (0, 0) [6.7초]
    ↓ 0.5초 대기
(10회 반복)
```

### 5.2 전후 반복 (테스트)

**시나리오**: 센서 테스트를 위한 전후 이동

```python
동작 방향: 전후
이동 거리: 1.0 m
반복 횟수: 5
대기 시간: 2.0 초
속도: 0.2 m/s
```

### 5.3 회전 반복 (주변 스캔)

**시나리오**: 주변 환경 스캔

```python
동작 방향: 회전
이동 거리: 1.57 (90도)
반복 횟수: 4  # 360도
대기 시간: 1.0 초
속도: 0.3 rad/s
```

### 5.4 프로그래밍 방식 제어

Python 코드로 직접 제어:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def repeat_left_right(distance=1.0, speed=0.2, count=5):
    """좌우 반복 동작"""
    rospy.init_node('auto_repeat')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Publisher 준비
    
    duration = distance / speed
    
    for i in range(count):
        # 우측 이동
        twist = Twist()
        twist.linear.y = -speed
        pub.publish(twist)
        time.sleep(duration)
        
        # 정지
        twist = Twist()
        pub.publish(twist)
        time.sleep(1)
        
        # 좌측 이동
        twist = Twist()
        twist.linear.y = speed
        pub.publish(twist)
        time.sleep(duration)
        
        # 정지
        twist = Twist()
        pub.publish(twist)
        time.sleep(1)
        
        rospy.loginfo(f"Completed cycle {i+1}/{count}")
    
    rospy.loginfo("✅ Repeat motion completed")

if __name__ == '__main__':
    repeat_left_right(distance=1.0, speed=0.2, count=5)
```

**실행:**
```bash
chmod +x repeat_motion.py
python3 repeat_motion.py
```

---

## 6. 문제 해결

### 6.1 컴파일 문제

#### 문제: "Permission denied: /dev/ttyUSB0"

```bash
# 해결
sudo usermod -aG dialout $USER
newgrp dialout

# 확인
ls -l /dev/ttyUSB0
```

#### 문제: "c++: fatal error: Killed"

```bash
# 메모리 부족 - SWAP 증가
sudo swapoff /swapfile
sudo fallocate -l 10G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 단일 코어로 재빌드
catkin_make -j1
```

### 6.2 실행 문제

#### 문제: "Could not find service ping_service"

```bash
# vel_converter_sdk가 실행 중인지 확인
rosnode list | grep vel_converter

# 실행되지 않았다면
rosrun happy_robo vel_converter_sdk
```

#### 문제: "No module named 'happy_robo.srv'"

```bash
# 환경 소싱
source ~/happy_ws/devel/setup.bash

# 메시지 재생성
cd ~/happy_ws
catkin_make -j1
source devel/setup.bash
```

### 6.3 UI 문제

#### 문제: "ModuleNotFoundError: No module named 'PyQt5'"

```bash
# PyQt5 설치
pip3 install PyQt5

# 시스템 패키지로 설치 (대안)
sudo apt install python3-pyqt5
```

#### 문제: "로봇이 움직이지 않음"

**체크리스트:**
1. ✅ roscore 실행 중?
2. ✅ vel_converter_sdk 실행 중?
3. ✅ /dev/ttyUSB0 연결됨?
4. ✅ 모터 컨트롤러 전원 ON?
5. ✅ 브레이크 해제됨?

**확인 명령:**
```bash
# ROS 노드 확인
rosnode list

# 토픽 확인
rostopic list

# cmd_vel 토픽 테스트
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

### 6.4 긴급 상황

#### 로봇 즉시 정지

**방법 1: UI에서**
```
🚨 긴급 정지 버튼 클릭
```

**방법 2: 터미널에서**
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

**방법 3: 정지 모드 활성화**
```bash
rostopic pub -1 /stop_mode std_msgs/Bool "data: true"
```

---

## 7. 고급 사용법

### 7.1 ROS API를 통한 제어

#### Python 예제

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
    
    def move_forward(self, speed=0.2, duration=2.0):
        """전진"""
        twist = Twist()
        twist.linear.x = speed
        
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        
        while (rospy.Time.now() - start).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()
        
        self.stop()
    
    def rotate(self, angular_speed=0.3, duration=2.0):
        """회전"""
        twist = Twist()
        twist.angular.z = angular_speed
        
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        
        while (rospy.Time.now() - start).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()
        
        self.stop()
    
    def stop(self):
        """정지"""
        self.pub.publish(Twist())

if __name__ == '__main__':
    robot = RobotController()
    
    # 전진 2초
    robot.move_forward(speed=0.2, duration=2.0)
    rospy.sleep(1)
    
    # 좌회전 90도
    robot.rotate(angular_speed=0.5, duration=math.pi/2/0.5)
    rospy.sleep(1)
    
    # 전진 2초
    robot.move_forward(speed=0.2, duration=2.0)
```

### 7.2 FastAPI를 통한 원격 제어

```bash
# 서버 실행
rosrun happy_robo server_fastapi.py

# 브라우저에서 접속
http://localhost:8000
```

**API 엔드포인트:**

```bash
# 전진
curl -X POST "http://localhost:8000/move" \
  -H "Content-Type: application/json" \
  -d '{"direction":"forward", "linear":0.2, "angular":0, "mode":"manual", "degree":0}'

# 정지
curl -X POST "http://localhost:8000/move" \
  -H "Content-Type: application/json" \
  -d '{"direction":"stop", "linear":0, "angular":0, "mode":"manual", "degree":0}'

# Ping 테스트
curl http://localhost:8000/ping

# 브레이크
curl http://localhost:8000/brake?command=1
```

---

## 8. 체크리스트

### 첫 실행 전 체크리스트

- [ ] ROS 설치 완료
- [ ] 프로젝트 빌드 완료
- [ ] 환경 소싱 (.bashrc에 추가)
- [ ] USB 시리얼 권한 설정
- [ ] /dev/ttyUSB0 장치 연결 확인
- [ ] 모터 컨트롤러 전원 확인
- [ ] roscore 실행
- [ ] vel_converter_sdk 실행

### 안전 체크리스트

- [ ] 로봇 주변에 장애물 없음
- [ ] 긴급 정지 방법 숙지
- [ ] 최대 속도 제한 설정 (0.5 m/s)
- [ ] 센서 데이터 모니터링 중
- [ ] 배터리 전압 확인

---

## 📞 추가 지원

- **문서**: 이 저장소의 다른 문서들 참조
  - CODE_ANALYSIS_SUMMARY.md
  - JETSON_NANO_BUILD_GUIDE.md
  - QUICK_COMPILE_GUIDE.md
  
- **GitHub Issues**: https://github.com/RedLeamp/happyrobo/issues

---

**작성일**: 2025-12-16  
**버전**: 1.0  
**대상**: Jetson Nano + Happy Robot
