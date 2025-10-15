# 🔥 FireDrone-Final: 산불 감지 및 자동 투하 시스템

## 📋 프로젝트 개요

드론 기반 산불 감지 및 소화탄 자동 투하 시스템입니다.  
멀티센서 퓨전, YOLO 객체 검출, 정밀 투하 알고리즘, 진압 검증까지 완전 자동화된 시스템입니다.

---

## 🚁 시스템 구성

### 하드웨어
- **드론 플랫폼**: Pixhawk 4 + PX4 펌웨어
- **센서**:
  - TF-Luna LiDAR (UART, 거리 측정)
  - MLX90640 IR Array (I2C, 열화상 감지)
  - MQ-2 Gas Sensor (Arduino 연동, 가스 농도)
  - 카메라 (YOLO 화재 검출)
- **컴퓨터**: Jetson Nano

### 소프트웨어
- **ROS Melodic/Noetic**
- **MAVROS** (Pixhawk 통신)
- **Python 3.7+**
- **YOLOv5/v8** (화재 검출, 선택)

---

## 📁 파일 구조

```
firedrone-final/
├── config.py                 # 설정 파라미터 (센서 포트, 알고리즘 상수)
├── utils.py                  # 유틸리티 함수 (좌표 변환, 수학 계산)
├── sensor_readers.py         # 센서 입력 함수 (LiDAR, IR, Gas)
├── alpha_beta_filter.py      # 알파-베타 필터 (목표 추적)
├── drop_controller.py        # 투하 제어 로직 (핵심 알고리즘)
├── main.py                   # 메인 실행 노드
├── yolo_bridge.py            # YOLO 검출 → ROS 토픽 브릿지
├── arduino_mq2.ino           # 아두이노 MQ-2 센서 스케치
├── requirements.txt          # Python 의존성
└── README.md                 # 본 문서
```

---

## 🔧 설치 방법

### 1. ROS 및 MAVROS 설치

```bash
# ROS Noetic (Ubuntu 20.04)
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

# MAVROS GeographicLib 데이터셋
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 2. Python 패키지 설치

```bash
cd firedrone-final
pip3 install -r requirements.txt
```

### 3. 아두이노 설정

`arduino_mq2.ino` 파일을 Arduino IDE로 열고 업로드합니다.

---

## ⚙️ 설정

### 센서 포트 설정

`config.py` 파일에서 시리얼 포트를 확인/수정하세요:

```python
TFLUNA_PORT = "/dev/ttyUSB0"    # TF-Luna 포트
ARDUINO_PORT = "/dev/ttyUSB1"   # Arduino 포트
```

포트 확인:
```bash
ls /dev/ttyUSB*
dmesg | grep tty
```

### 알고리즘 파라미터 튜닝

`config.py`에서 투하 조건 조정:

```python
DROP_K_D = 1.15          # 공기저항 보정 (낙하시간)
DROP_LATENCY = 0.15      # 시스템 지연 (초)
DROP_E_MAX = 0.5         # 허용 측면 오차 (m)
DROP_H_MIN = 2.0         # 최소 투하 고도 (m)
DROP_H_MAX = 4.0         # 최대 투하 고도 (m)
```

---

## 🚀 실행 방법

### 1. ROS 마스터 실행

```bash
roscore
```

### 2. MAVROS 실행 (새 터미널)

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

### 3. YOLO 브릿지 실행 (새 터미널)

```bash
cd firedrone-final
python3 yolo_bridge.py
```

### 4. 메인 드롭 컨트롤러 실행 (새 터미널)

```bash
cd firedrone-final
python3 main.py
```

---

## 📡 ROS 토픽

### 구독 (Subscribe)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mavros/local_position/pose` | `PoseStamped` | 드론 위치 (ENU) |
| `/mavros/local_position/velocity_local` | `TwistStamped` | 드론 속도 |
| `/mavros/imu/data` | `Imu` | 자세 (Roll, Pitch, Yaw) |
| `/fire_detector/target_local` | `PointStamped` | YOLO 화재 좌표 |
| `/fire_detector/confidence` | `Float32` | YOLO 신뢰도 |

### 발행 (Publish)

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/mavros/cmd/command` | `CommandLong` | 서보 제어 (투하) |

---

## 🎯 동작 원리

### Phase 1: 탐색
```
드론 자율 비행 → 센서 스캔 → 화재 감지 대기
```

### Phase 2: 감지
```
IR 온도 -10°C 이하 + 가스 농도 350ppm 이상
→ FireDetector 화재 신뢰도 60% 이상
→ YOLO 화재 bbox 검출 → 좌표 계산
```

### Phase 3: 접근
```
드론이 목표로 이동 (자동/수동)
→ 알파-베타 필터로 좌표 추적 (노이즈 제거)
→ 고도 2~4m, 수평 속도 2~5m/s 유지
```

### Phase 4: 투하
```
안전 게이트 체크:
  ✓ 고도 범위 OK?
  ✓ 자세 안정 (Roll/Pitch < 15°)?
  ✓ 수직 속도 < 0.5m/s?

탄도 계산:
  낙하시간 = sqrt(2h/g) * 1.15
  리드거리 = 수평속도 * (낙하시간 + 0.15초)

조건 만족 → 서보 PWM 1900us → 투하!
```

### Phase 5: 검증
```
투하 후 3초 대기
→ IR 온도 > 0°C & YOLO 신뢰도 < 0.2
→ 진압 성공! → 다음 목표 탐색
```

---

## 🛠️ 트러블슈팅

### 센서 연결 실패

```bash
# 권한 부여
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

# 사용자를 dialout 그룹에 추가
sudo usermod -aG dialout $USER
# 로그아웃 후 재로그인
```

### MAVROS 연결 실패

```bash
# Pixhawk 포트 확인
ls /dev/ttyACM*

# 연결 상태 확인
rostopic echo /mavros/state
```

### LiDAR 데이터 없음

```bash
# 시리얼 모니터로 원시 데이터 확인
python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 115200); print(s.read(100))"
```

---

TEAM 산불을 막아조 🚁

