# 🤖 Happy Robot - 완전 통합 가이드

> Jetson Nano 기반 4륜 독립 조향 자율주행 로봇  
> **컴파일부터 실행, 제어까지 모든 것**

---

## 🎯 이 프로젝트에서 할 수 있는 것

✅ **Jetson Nano에서 컴파일**  
✅ **ROS 노드 실행**  
✅ **GUI로 로봇 제어**  
✅ **좌우 반복 동작 자동화**  
✅ **모터 상태 모니터링**  
✅ **브레이크 및 초기화 제어**

---

## 📚 문서 목록 (전체 7개)

| # | 문서 | 크기 | 용도 |
|---|------|------|------|
| 1 | **COMPLETE_USAGE_GUIDE.md** | 13KB | 📖 **여기서 시작!** 전체 사용법 |
| 2 | **QUICK_COMPILE_GUIDE.md** | 13KB | ⚡ 빠른 컴파일 명령어 |
| 3 | **JETSON_NANO_BUILD_GUIDE.md** | 11KB | 🔧 상세 빌드 가이드 |
| 4 | **CODE_ANALYSIS_SUMMARY.md** | 12KB | 🔍 코드 구조 분석 |
| 5 | **JETSON_NANO_COMPILE_COMMANDS.sh** | 16KB | 🤖 자동 빌드 스크립트 |
| 6 | **README_COMPILE.md** | 4KB | 📋 문서 인덱스 |
| 7 | **robot_control_ui.py** | 26KB | 🎮 제어 UI 프로그램 |

---

## 🚀 빠른 시작 (3단계)

### 1️⃣ 컴파일

```bash
cd ~/happy_ws
chmod +x JETSON_NANO_COMPILE_COMMANDS.sh
./JETSON_NANO_COMPILE_COMMANDS.sh
```

### 2️⃣ 실행

```bash
# 터미널 1: ROS Master
roscore

# 터미널 2: 메인 노드
source ~/happy_ws/devel/setup.bash
rosrun happy_robo vel_converter_sdk

# 터미널 3: 제어 UI
python3 robot_control_ui.py
```

### 3️⃣ 제어

UI에서:
- **기본 제어 탭**: 방향 버튼으로 수동 제어
- **자동 반복 탭**: 좌우 왕복 동작 설정
- **고급 설정 탭**: 모터 Ping, 브레이크, 초기화

---

## 🎮 제어 UI 미리보기

### 기본 제어
```
┌────────────────────────────┐
│    속도 설정               │
│  선속도: [====] 0.20 m/s  │
│  각속도: [====] 0.20 rad/s│
│                            │
│    방향 제어               │
│    ↖    ↑    ↗            │
│    ←    ■    →            │
│    ↙    ↓    ↘            │
│    ↶         ↷            │
└────────────────────────────┘
```

### 자동 반복
```
┌────────────────────────────┐
│  좌우 반복 동작 설정       │
│  동작: [좌우 ▼]           │
│  거리: [1.0] m            │
│  횟수: [5]                │
│  속도: [0.2] m/s          │
│                            │
│  [████░░░░░] 50%          │
│  반복 3/5                  │
│                            │
│  [🚀 시작] [⏹ 중지]       │
└────────────────────────────┘
```

---

## 📖 상세 가이드

### 🔰 초보자
1. **COMPLETE_USAGE_GUIDE.md** 읽기
   - 저장소 구조 이해
   - 컴파일 방법
   - 실행 방법
   - UI 사용법

### ⚡ 빠른 참조
**QUICK_COMPILE_GUIDE.md**
- Ubuntu 18.04 명령어
- Ubuntu 20.04 명령어
- 복사-붙여넣기 가능

### 🛠️ 상세 빌드
**JETSON_NANO_BUILD_GUIDE.md**
- Step-by-step 가이드
- 의존성 설치
- 문제 해결

### 🔍 코드 이해
**CODE_ANALYSIS_SUMMARY.md**
- CMake 분석
- ROS 패키지 구조
- C++ 알고리즘

---

## 🎯 주요 기능

### 1. 실시간 로봇 제어
- **8방향 이동**: 전후좌우, 대각선
- **회전**: 제자리 좌/우 회전
- **속도 조절**: 슬라이더로 실시간 조정
- **긴급 정지**: 한 번의 클릭

### 2. 자동 반복 동작
- **좌우 왕복**: 설정 거리만큼 반복
- **전후 이동**: 센서 테스트용
- **회전 스캔**: 360도 환경 스캔
- **진행률 표시**: 실시간 상태 확인

### 3. 모터 관리
- **Ping 테스트**: 8개 모터 통신 확인
- **개별 초기화**: 모터 1~8번 선택 초기화
- **브레이크 제어**: 5가지 모드
- **상태 모니터링**: 실시간 위치/속도

### 4. 안전 기능
- **긴급 정지**: 즉시 모든 동작 중단
- **거리 경고**: 최소 거리 < 0.5m 시 경고
- **로그 기록**: 모든 동작 기록
- **정지 모드**: 모든 명령 무시

---

## 📊 시스템 요구사항

### 하드웨어
- **Jetson Nano** (4GB 권장)
- **저장 공간**: 10GB 이상
- **메모리**: 4GB RAM + 8GB SWAP

### 소프트웨어
- **OS**: Ubuntu 18.04 또는 20.04
- **ROS**: Melodic (18.04) / Noetic (20.04)
- **Python**: 3.6+ (PyQt5)
- **CMake**: 3.16+

---

## 🔧 기술 스택

### 로봇 플랫폼
- **구동 시스템**: 4륜 독립 조향 (Swerve Drive)
- **모터**: 8개 (구동 4 + 조향 4)
- **센서**: 레이저 스캐너, IMU
- **통신**: 시리얼 (1Mbps)

### 소프트웨어
- **ROS**: 로봇 운영 체제
- **Cartographer**: Google SLAM
- **PCL**: 포인트클라우드 처리
- **FastAPI**: REST API 서버
- **PyQt5**: GUI 프레임워크

---

## 📝 컴파일 옵션

### 자동 빌드 (권장)
```bash
./JETSON_NANO_COMPILE_COMMANDS.sh
```
- ✅ 자동 의존성 설치
- ✅ SWAP 자동 설정
- ✅ 빌드 시간 측정
- ✅ 에러 처리

### 수동 빌드
```bash
# 초간단 (의존성 설치됨)
sudo fallocate -l 8G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
cd ~/happy_ws && catkin_make -j1 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

## 🎬 실행 방법

### 방법 1: 개별 노드
```bash
# 터미널 1
roscore

# 터미널 2
rosrun happy_robo vel_converter_sdk

# 터미널 3
python3 robot_control_ui.py
```

### 방법 2: Launch 파일
```bash
roslaunch happy_robo run.launch
```

### 방법 3: FastAPI 서버
```bash
rosrun happy_robo server_fastapi.py
# http://localhost:8000
```

---

## 🔍 자동 반복 동작 예시

### 예시 1: 복도 패트롤
```python
동작: 좌우
거리: 2.0 m
횟수: 10
속도: 0.3 m/s
대기: 0.5 초

결과: 좌우로 2m씩 10회 왕복 (약 2분)
```

### 예시 2: 센서 테스트
```python
동작: 전후
거리: 1.0 m
횟수: 5
속도: 0.2 m/s
대기: 2.0 초

결과: 전후로 1m씩 5회 왕복 (약 1분)
```

### 예시 3: 360도 스캔
```python
동작: 회전
거리: 1.57 (90도)
횟수: 4
속도: 0.3 rad/s
대기: 1.0 초

결과: 90도씩 4회 회전 = 360도 (약 30초)
```

---

## ⚠️ 문제 해결

### "Permission denied: /dev/ttyUSB0"
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

### "c++: fatal error: Killed"
```bash
# SWAP 증가
sudo swapoff /swapfile
sudo fallocate -l 10G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 재빌드
catkin_make -j1
```

### "No module named 'PyQt5'"
```bash
pip3 install PyQt5
```

### "로봇이 움직이지 않음"
1. ✅ roscore 실행?
2. ✅ vel_converter_sdk 실행?
3. ✅ USB 연결?
4. ✅ 모터 전원?
5. ✅ 브레이크 해제?

---

## 📞 지원 및 문서

### 문서
- **COMPLETE_USAGE_GUIDE.md**: 전체 사용법
- **CODE_ANALYSIS_SUMMARY.md**: 코드 분석
- **JETSON_NANO_BUILD_GUIDE.md**: 빌드 가이드
- **QUICK_COMPILE_GUIDE.md**: 빠른 참조

### GitHub
- **저장소**: https://github.com/RedLeamp/happyrobo
- **Issues**: https://github.com/RedLeamp/happyrobo/issues
- **Pull Request**: https://github.com/RedLeamp/happyrobo/pull/1

---

## 🎉 프로젝트 특징

### ✨ 완전성
- 🔨 컴파일 자동화
- 🚀 즉시 실행 가능
- 🎮 GUI 제어 프로그램
- 📖 완벽한 문서화

### 🛡️ 안전성
- 🚨 긴급 정지 버튼
- ⚠️ 거리 경고 시스템
- 📝 전체 동작 로깅
- 🔒 정지 모드

### 🎯 편의성
- 🖱️ 직관적 UI
- ⚡ 자동 반복 기능
- 📊 실시간 모니터링
- 🔧 쉬운 설정

---

## 📈 프로젝트 통계

- **문서**: 7개 (총 95KB)
- **코드 라인**: 4,000+ 줄
- **ROS 노드**: 15+ 개
- **지원 플랫폼**: Jetson Nano
- **개발 기간**: 2025
- **최종 업데이트**: 2025-12-16

---

## 👥 기여

이 프로젝트는 다음을 포함합니다:
- ✅ 완전한 빌드 시스템
- ✅ 자동화 스크립트
- ✅ GUI 제어 프로그램
- ✅ 상세한 문서
- ✅ 예제 코드
- ✅ 문제 해결 가이드

---

## 🏆 시작하기

### 1분 요약
```bash
# 1. 클론
git clone https://github.com/RedLeamp/happyrobo.git
cd happyrobo

# 2. 컴파일
./JETSON_NANO_COMPILE_COMMANDS.sh

# 3. 실행
roscore &
rosrun happy_robo vel_converter_sdk &
python3 robot_control_ui.py
```

### 완벽 가이드
👉 **COMPLETE_USAGE_GUIDE.md** 읽기

---

## 📜 라이선스

이 프로젝트는 오픈소스입니다.

---

## 🙏 감사합니다!

Happy Robot으로 즐거운 로봇 개발 하세요! 🤖✨

**문의**: GitHub Issues  
**업데이트**: 2025-12-16
