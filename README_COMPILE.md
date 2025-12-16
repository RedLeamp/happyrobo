# Happy Robot - 컴파일 문서 총정리

## 📚 문서 구조

이 저장소에는 Jetson Nano에서 Happy Robot ROS 프로젝트를 컴파일하기 위한 완전한 문서 세트가 포함되어 있습니다.

### 📄 문서 목록

| 문서 | 크기 | 용도 | 대상 독자 |
|------|------|------|----------|
| **QUICK_COMPILE_GUIDE.md** | 13KB | 빠른 명령어 참조 | 경험있는 개발자 |
| **JETSON_NANO_BUILD_GUIDE.md** | 11KB | 상세한 단계별 가이드 | 초보자 |
| **CODE_ANALYSIS_SUMMARY.md** | 12KB | 코드 구조 분석 | 시스템 이해 필요 시 |
| **JETSON_NANO_COMPILE_COMMANDS.sh** | 16KB | 자동화 스크립트 | 자동 빌드 원하는 사용자 |

---

## 🚀 빠른 시작

### 방법 1: 자동화 스크립트 사용 (가장 쉬움)

```bash
# 1. 스크립트 다운로드 (이미 저장소에 있음)
cd ~/happy_ws

# 2. 실행 권한 부여
chmod +x JETSON_NANO_COMPILE_COMMANDS.sh

# 3. 스크립트 실행
./JETSON_NANO_COMPILE_COMMANDS.sh
```

**이 스크립트는 자동으로:**
- Ubuntu 버전 감지 (18.04/20.04)
- ROS 배포판 선택 (Melodic/Noetic)
- 모든 의존성 설치
- SWAP 메모리 설정
- 프로젝트 빌드
- 환경 설정

### 방법 2: 수동 명령어 (빠른 참조)

**Ubuntu 18.04 (ROS Melodic):**
```bash
# QUICK_COMPILE_GUIDE.md의 "Ubuntu 18.04" 섹션 참조
export ROS_DISTRO=melodic
# ... (나머지 명령어)
```

**Ubuntu 20.04 (ROS Noetic):**
```bash
# QUICK_COMPILE_GUIDE.md의 "Ubuntu 20.04" 섹션 참조
export ROS_DISTRO=noetic
# ... (나머지 명령어)
```

### 방법 3: 단계별 상세 가이드

**JETSON_NANO_BUILD_GUIDE.md** 참조 - 모든 단계를 자세히 설명

---

## 📖 사용 시나리오별 가이드

### 🆕 처음 컴파일하는 경우

1. **JETSON_NANO_BUILD_GUIDE.md** 읽기
   - 시스템 요구사항 확인
   - 의존성 이해
   - 단계별 진행

2. **JETSON_NANO_COMPILE_COMMANDS.sh** 실행
   - 자동으로 모든 설정 처리

3. **문제 발생 시** → QUICK_COMPILE_GUIDE.md의 "에러 해결" 참조

### ⚡ 빠르게 명령어만 필요한 경우

**QUICK_COMPILE_GUIDE.md** 사용
- 복사해서 붙여넣기 가능한 명령어
- Ubuntu 버전별로 정리

### 🔍 코드 구조를 이해하고 싶은 경우

**CODE_ANALYSIS_SUMMARY.md** 읽기
- CMake 빌드 시스템 분석
- ROS 패키지 의존성
- C++ 코드 아키텍처
- 알고리즘 설명

### 🛠️ 문제 해결이 필요한 경우

1. **QUICK_COMPILE_GUIDE.md** → "자주 발생하는 에러 해결" 섹션
2. **JETSON_NANO_BUILD_GUIDE.md** → "알려진 문제 및 해결 방법" 섹션

---

## 🎯 핵심 컴파일 명령어 (초간단)

```bash
# 이미 의존성이 모두 설치된 경우

# 1. SWAP 설정 (필수!)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 2. 전력 최대
sudo nvpmodel -m 0
sudo jetson_clocks

# 3. 빌드
cd ~/happy_ws
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release

# 4. 환경 설정
source devel/setup.bash
```

---

## 📊 시스템 요구사항 요약

### 하드웨어
- **Jetson Nano** (4GB 권장)
- **저장 공간**: 10GB 이상
- **메모리**: 4GB RAM + 8GB SWAP

### 소프트웨어
- **OS**: Ubuntu 18.04 또는 20.04
- **ROS**: Melodic (18.04) 또는 Noetic (20.04)
- **CMake**: 3.16 이상
- **GCC**: 7.5 이상 (C++17 지원)

### 주요 의존성
- ROS 기본 패키지 (tf, tf2, laser_geometry 등)
- Cartographer (SLAM)
- PCL 1.8+ (Point Cloud Library)
- Eigen3 (선형대수)
- Serial (시리얼 통신)

---

## ⚡ 빌드 시간 예상

| 옵션 | 예상 시간 | 메모리 사용 |
|------|----------|------------|
| `-j1` (단일 코어) | 50-70분 | 낮음 (~2GB) |
| `-j2` (2코어) | 35-50분 | 중간 (~3GB) |
| `-j4` (4코어) | 25-40분 | 높음 (~4GB+) ⚠️ |

**권장**: Jetson Nano에서는 `-j1` 또는 `-j2` 사용

---

## 🛡️ 문제 해결 체크리스트

빌드가 실패하면 다음을 확인하세요:

### 메모리 관련
- [ ] SWAP이 8GB 이상 설정되어 있는가?
- [ ] 빌드 옵션이 `-j1` 또는 `-j2`인가?
- [ ] 다른 프로그램이 메모리를 많이 사용하고 있지 않은가?

### 의존성 관련
- [ ] ROS가 올바르게 설치되었는가?
- [ ] `rosdep install` 명령을 실행했는가?
- [ ] Cartographer가 설치되었는가?
- [ ] PCL 버전이 호환되는가?

### 권한 관련
- [ ] `/dev/ttyUSB0`에 접근 권한이 있는가?
- [ ] dialout 그룹에 속해 있는가?

### 설정 관련
- [ ] ROS 환경이 소싱되었는가? (`source /opt/ros/$ROS_DISTRO/setup.bash`)
- [ ] Jetson Nano 전력 모드가 최대로 설정되었는가?

---

## 📞 도움말

### 빌드 에러 발생 시

1. **에러 메시지 확인**
   ```bash
   catkin_make -j1 2>&1 | tee build_error.log
   ```

2. **문서 참조**
   - QUICK_COMPILE_GUIDE.md → "자주 발생하는 에러 해결"
   - JETSON_NANO_BUILD_GUIDE.md → "알려진 문제 및 해결 방법"

3. **시스템 정보 수집**
   ```bash
   uname -a
   lsb_release -a
   cmake --version
   gcc --version
   free -h
   ```

### 추가 리소스

- [ROS Wiki](http://wiki.ros.org/)
- [Jetson Forums](https://forums.developer.nvidia.com/)
- [GitHub Issues](https://github.com/RedLeamp/happyrobo/issues)

---

## 🎓 학습 경로

### 초급 (처음 시작)
1. JETSON_NANO_BUILD_GUIDE.md 읽기
2. JETSON_NANO_COMPILE_COMMANDS.sh 실행
3. 빌드 완료 후 테스트

### 중급 (이해 필요)
1. CODE_ANALYSIS_SUMMARY.md 읽기
2. CMake 설정 이해
3. ROS 패키지 구조 파악

### 고급 (수정/최적화)
1. 코드 분석 문서 심화 학습
2. 성능 프로파일링
3. 빌드 옵션 최적화

---

## ✅ 성공 확인

빌드가 성공적으로 완료되면:

1. **파일 확인**
   ```bash
   ls ~/happy_ws/devel/lib/happy_robo/
   ```

2. **패키지 확인**
   ```bash
   source ~/happy_ws/devel/setup.bash
   rospack find happy_robo
   ```

3. **노드 실행 테스트**
   ```bash
   # 터미널 1
   roscore
   
   # 터미널 2
   rosrun happy_robo vel_converter_sdk
   ```

---

## 📝 문서 업데이트 이력

| 날짜 | 변경 내용 |
|------|----------|
| 2025-12-16 | 초기 문서 세트 생성 |
| | - CMake/ROS 코드 분석 |
| | - Jetson Nano 빌드 가이드 |
| | - 자동화 스크립트 |
| | - 빠른 참조 가이드 |

---

## 🤝 기여

문서 개선 제안이나 오류 발견 시:
1. GitHub Issues 등록
2. Pull Request 제출
3. 문서 수정 제안

---

**프로젝트**: Happy Robot ROS  
**플랫폼**: Jetson Nano (ARM64)  
**ROS 버전**: Melodic / Noetic  
**문서 버전**: 1.0  
**마지막 업데이트**: 2025-12-16
