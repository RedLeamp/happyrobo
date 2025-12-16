# 🚀 Jetson Nano 빠른 시작 가이드

> **Jetson Nano에서 5분 안에 시작하기**

---

## 📦 전제 조건

- ✅ Jetson Nano (4GB 권장)
- ✅ Ubuntu 18.04 또는 20.04
- ✅ 인터넷 연결
- ✅ 최소 10GB 여유 공간

---

## ⚡ 초고속 설치 (한 줄 명령어)

### 방법 1: 자동 설정 스크립트

```bash
# 저장소 클론
git clone https://github.com/RedLeamp/happyrobo.git
cd happyrobo

# 브랜치 전환
git checkout genspark_ai_developer

# 자동 설정 실행 (모든 것을 자동으로 설치)
chmod +x setup_jetson_nano.sh
./setup_jetson_nano.sh
```

**이 스크립트가 하는 일:**
- ✅ Ubuntu 버전 자동 감지
- ✅ ROS 자동 설치 (Melodic/Noetic)
- ✅ 모든 의존성 설치
- ✅ SWAP 8GB 자동 설정
- ✅ 프로젝트 자동 빌드
- ✅ 환경 변수 자동 설정
- ✅ 실행 스크립트 생성

**소요 시간**: 약 30-60분 (인터넷 속도에 따라)

---

## 🎮 실행 방법 (3가지)

### 방법 1: 웹 UI (추천! X11 불필요)

```bash
# 1. ROS Master 실행 (터미널 1)
roscore

# 2. 메인 노드 실행 (터미널 2)
rosrun happy_robo vel_converter_sdk

# 3. 웹 UI 실행 (터미널 3)
cd ~/happy_ws
python3 web_control_ui.py
```

**브라우저에서 접속:**
- 로컬: http://localhost:5000
- 원격: http://<JETSON_IP>:5000

**특징:**
- 🌐 웹 브라우저에서 제어
- 📱 모바일에서도 접근 가능
- 🖥️ X11 서버 불필요 (SSH만으로 가능)
- ⚡ 실시간 상태 모니터링

### 방법 2: PyQt5 GUI (X11 필요)

```bash
# 1. ROS Master 실행
roscore

# 2. 메인 노드 실행
rosrun happy_robo vel_converter_sdk

# 3. GUI 실행
cd ~/happy_ws
python3 robot_control_ui.py
```

**특징:**
- 🖥️ 데스크톱 애플리케이션
- 🎨 풍부한 UI
- ⚡ 빠른 응답 속도

### 방법 3: 실행 스크립트

```bash
# 자동 생성된 실행 스크립트
cd ~/happy_ws
./run_robot.sh
```

**메뉴 선택:**
```
[1] roscore 실행
[2] vel_converter_sdk 실행
[3] FastAPI 서버 실행
[4] 제어 UI 실행
[5] Launch 파일 실행
[6] 모든 노드 종료
```

---

## 🔧 SSH로 원격 제어하기

### SSH 포트 포워딩

```bash
# 로컬 컴퓨터에서
ssh -L 5000:localhost:5000 jetson@<JETSON_IP>

# Jetson에서 웹 UI 실행
cd ~/happy_ws
python3 web_control_ui.py

# 로컬 브라우저에서
# http://localhost:5000 접속
```

### 모바일에서 제어

```bash
# Jetson에서 웹 UI 실행
python3 web_control_ui.py

# 같은 Wi-Fi에 연결된 스마트폰 브라우저에서
# http://<JETSON_IP>:5000 접속
```

---

## 📊 시스템 상태 확인

### 메모리 확인
```bash
free -h
```

### SWAP 확인
```bash
sudo swapon --show
```

### ROS 노드 확인
```bash
rosnode list
```

### 토픽 확인
```bash
rostopic list
rostopic echo /cmd_vel
```

### USB 시리얼 확인
```bash
ls -l /dev/ttyUSB*
```

---

## 🎯 빠른 테스트

### 1. 모터 Ping 테스트
```bash
rosservice call /ping_service
```

**예상 출력:**
```
success: True
message: "True,True,True,True,True,True,True,True"
```

### 2. 로봇 이동 테스트
```bash
# 전진 (2초)
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0, z: 0}"
sleep 2

# 정지
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

### 3. 브레이크 해제
```bash
rosservice call /brake_service "command: 1"
```

---

## 🐛 문제 해결

### 문제 1: "Permission denied: /dev/ttyUSB0"

```bash
sudo usermod -aG dialout $USER
newgrp dialout

# 또는 재부팅
sudo reboot
```

### 문제 2: "roscore: command not found"

```bash
source /opt/ros/melodic/setup.bash  # 또는 noetic
source ~/happy_ws/devel/setup.bash

# .bashrc에 추가 (영구 적용)
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/happy_ws/devel/setup.bash" >> ~/.bashrc
```

### 문제 3: "ModuleNotFoundError: No module named 'flask'"

```bash
pip3 install --user flask
```

### 문제 4: 로봇이 움직이지 않음

**체크리스트:**
```bash
# 1. ROS Master 실행 중?
rosnode list

# 2. vel_converter_sdk 실행 중?
rosnode list | grep vel_converter

# 3. USB 연결 확인
ls -l /dev/ttyUSB0

# 4. 브레이크 해제
rosservice call /brake_service "command: 1"

# 5. 모터 초기화
rosservice call /motor_init_service "command: 0"
```

### 문제 5: 웹 UI 접속 안 됨

```bash
# 방화벽 확인
sudo ufw status

# 포트 5000 열기
sudo ufw allow 5000

# 또는 방화벽 비활성화
sudo ufw disable
```

---

## 📱 웹 UI 사용법

### 기본 제어 탭
```
1. 속도 슬라이더 조정
   - 선속도: 0-0.5 m/s
   - 각속도: 0-0.5 rad/s

2. 방향 버튼 클릭 또는 터치
   - 누르는 동안 이동
   - 손을 떼면 정지

3. 긴급 정지 버튼
   - 빨간색 버튼
   - 모든 동작 즉시 중단
```

### 자동 반복 탭
```
1. 동작 방향 선택
   - 좌우: 좌→우→좌 반복
   - 전후: 전진→후진 반복
   - 회전: 좌회전→우회전 반복

2. 파라미터 설정
   - 이동 거리: 0.1-10 m
   - 반복 횟수: 1-100
   - 대기 시간: 0.5-10 초
   - 속도: 0.1-0.5 m/s

3. 반복 시작/중지
   - 시작: 진행률 실시간 표시
   - 중지: 즉시 중단
```

### 고급 설정 탭
```
1. 모터 Ping 테스트
   - 8개 모터 통신 확인

2. 모터 초기화
   - ID 선택 (1-10)
   - 개별 초기화 실행

3. 브레이크 제어
   - 0: Motor Brake
   - 1: Motor Non-Brake
   - 2: Outer Brake
   - 3: Outer Non-Brake
   - 4: Reset Pose
```

---

## 🎓 학습 경로

### 5분 시작
```bash
./setup_jetson_nano.sh
```

### 10분 실행
```bash
roscore &
rosrun happy_robo vel_converter_sdk &
python3 web_control_ui.py
```

### 30분 마스터
- 웹 UI 모든 기능 테스트
- 자동 반복 동작 설정
- 모터 관리 실습

---

## 💡 팁과 트릭

### 1. 백그라운드 실행
```bash
# roscore 백그라운드
roscore &

# 메인 노드 백그라운드
rosrun happy_robo vel_converter_sdk &

# 로그 확인
tail -f ~/.ros/log/latest/rosout.log
```

### 2. 자동 시작 스크립트
```bash
# autostart.sh 생성
cat > ~/happy_ws/autostart.sh << 'EOF'
#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/happy_ws/devel/setup.bash
roscore &
sleep 3
rosrun happy_robo vel_converter_sdk &
sleep 2
cd ~/happy_ws && python3 web_control_ui.py &
EOF

chmod +x ~/happy_ws/autostart.sh

# 실행
~/happy_ws/autostart.sh
```

### 3. systemd 서비스 등록
```bash
sudo tee /etc/systemd/system/happy-robot.service << EOF
[Unit]
Description=Happy Robot Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$HOME/happy_ws
ExecStart=$HOME/happy_ws/autostart.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable happy-robot.service
sudo systemctl start happy-robot.service
```

### 4. 성능 모니터링
```bash
# CPU/메모리 실시간 모니터링
htop

# ROS 노드 모니터링
rosnode info /vel_converter_sdk

# 네트워크 트래픽
iftop
```

---

## 📞 도움말

### 문서
- **README_FINAL.md** - 전체 개요
- **COMPLETE_USAGE_GUIDE.md** - 상세 사용법
- **CODE_ANALYSIS_SUMMARY.md** - 코드 분석

### 온라인 리소스
- GitHub: https://github.com/RedLeamp/happyrobo
- Issues: https://github.com/RedLeamp/happyrobo/issues

### 로그 확인
```bash
# ROS 로그
cat ~/.ros/log/latest/rosout.log

# 시스템 로그
journalctl -xe

# dmesg (하드웨어)
dmesg | tail -50
```

---

## ✅ 설치 확인 체크리스트

- [ ] setup_jetson_nano.sh 실행 완료
- [ ] 빌드 성공 (~/happy_ws/devel 존재)
- [ ] roscore 실행 가능
- [ ] vel_converter_sdk 실행 가능
- [ ] 웹 UI 접속 가능 (http://localhost:5000)
- [ ] /dev/ttyUSB0 권한 OK
- [ ] 모터 Ping 성공
- [ ] 로봇 이동 테스트 성공

---

## 🎉 완료!

이제 Jetson Nano에서 Happy Robot을 완전히 제어할 수 있습니다!

**다음 단계:**
1. 웹 UI로 기본 제어 연습
2. 자동 반복 동작 테스트
3. 실제 환경에서 주행 테스트

**Happy Robot과 즐거운 시간 보내세요! 🤖✨**

---

**작성일**: 2025-12-16  
**버전**: 1.0  
**플랫폼**: Jetson Nano (Ubuntu 18.04/20.04)
