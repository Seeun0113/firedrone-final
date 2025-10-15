#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
설정 파라미터 모음
센서 포트, I2C 주소, 필터 계수, 투하 조건 등
"""

# === 센서 설정 ===
TFLUNA_PORT = "/dev/ttyUSB0"
TFLUNA_BAUDRATE = 115200

ARDUINO_PORT = "/dev/ttyUSB1"
ARDUINO_BAUDRATE = 9600

MLX90640_I2C_BUS = 1
MLX90640_ADDRESS = 0x33

# === 알파-베타 필터 계수 ===
FILTER_ALPHA = 0.6  # 위치 보정 가중치
FILTER_BETA = 0.4   # 속도 보정 가중치

# === 투하 조건 파라미터 ===
DROP_K_D = 1.15              # 공기저항 보정 계수
DROP_LATENCY = 0.15          # 시스템 지연 시간 (초)
DROP_E_MAX = 0.5             # 최대 측면 오차 허용 (m)
DROP_H_MIN = 2.0             # 최소 고도 (m)
DROP_H_MAX = 3.0             # 최대 고도 (m)
DROP_MAX_ROLL = 15.0         # 최대 롤 각도 (도)
DROP_MAX_PITCH = 15.0        # 최대 피치 각도 (도)
DROP_MAX_VZ = 0.5            # 최대 수직 속도 (m/s)

# === 투하 검증 설정 ===
DROP_COOLDOWN = 3.0              # 투하 후 재투하 금지 시간 (초)
VERIFY_TEMP_THRESHOLD = 60.0     # 진압 확인 온도 임계값 (°C)
VERIFY_YOLO_THRESHOLD = 0.2      # 진압 확인 YOLO 신뢰도 임계값

# === 화재 감지 임계값 ===
FIRE_TEMP_THRESHOLD = 60.0   # IR 온도 임계값 (℃)
FIRE_GAS_THRESHOLD = 300.0   # 가스 농도 임계값 (ppm)

# === ROS 설정 ===
ROS_RATE_HZ = 10             # 메인 루프 주기 (Hz)

# === 아두이노 서보+가스 설정 (Windows) ===
ARDUINO_SERVO_GAS_PORT = "COM4"      # 실제 포트 확인 필요 (Windows: COM3, COM4 등)
ARDUINO_SERVO_GAS_BAUDRATE = 9600

# === YOLO 설정 ===
YOLO_MODEL_PATH = "C:/path/to/fire_best.pt"  # 실제 YOLO 모델 경로로 수정
YOLO_CONF_THRESHOLD = 0.75                   # 화재 감지 신뢰도 임계값

# === 투하 검증 설정 (개선 버전) ===
VERIFY_ENABLED = True                # 투하 후 검증 활성화
VERIFY_WAIT_TIME = 5.0               # 투하 후 대기 시간 (초)
VERIFY_TEMP_DROP = 20.0              # 온도 하락 기준 (℃)
VERIFY_GAS_DROP_RATIO = 0.5          # 가스 농도 하락 비율 (50%)
VERIFY_YOLO_CONF_DROP = 0.3          # YOLO 신뢰도 하락 기준

