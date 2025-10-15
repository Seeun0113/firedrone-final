#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센서 입력 함수 모음 (실제 하드웨어 버전)
TF-Luna LiDAR, MLX90640 IR Array, Arduino MQ-2 Gas Sensor
"""

import struct
import time
import rospy
from smbus2 import SMBus, i2c_msg

# === TF-Luna LiDAR (최신 프레임만 읽기) ===
def read_tfluna_latest(ser):
    """
    UART 버퍼에서 가장 최신 TF-Luna 프레임만 읽기
    
    Args:
        ser: serial.Serial 객체 (논블로킹 모드)
    
    Returns:
        (거리(m), 신호강도, 온도(℃)) 또는 (None, None, None)
    """
    latest = None
    
    # 버퍼에 패킷 한 개(9B) 이상 있을 때만 처리
    while ser.in_waiting >= 9:
        # 헤더 동기화: 0x59 0x59
        b0 = ser.read(1)
        if b0 != b'Y':  # 0x59
            continue
        if ser.in_waiting < 8:
            break
        b1 = ser.read(1)
        if b1 != b'Y':  # 0x59
            continue
        
        # 나머지 7바이트 읽기
        data = ser.read(7)
        if len(data) < 7:
            break
        
        dist_cm = data[0] | (data[1] << 8)
        strength = data[2] | (data[3] << 8)
        temp = (data[4] | (data[5] << 8)) / 8.0 - 256.0
        
        # cm → m 변환
        dist_m = dist_cm / 100.0
        
        # 유효 범위 체크 (0.2m ~ 8m)
        if 0.2 <= dist_m <= 8.0:
            latest = (dist_m, strength, temp)
    
    return latest if latest else (None, None, None)


# === MLX90640 IR (16비트 레지스터 직접 읽기) ===
class MLX90640Reader:
    """
    MLX90640 IR 센서 리더 (16비트 레지스터 방식)
    """
    def __init__(self, bus_num=1, address=0x33, temp_offset=-322.0):
        self.address = address
        self.bus = SMBus(bus_num)
        self.temp_offset = temp_offset  # 실제 온도 보정값
    
    def read_word16(self, reg16):
        """16비트 레지스터 읽기"""
        w = i2c_msg.write(self.address, [(reg16 >> 8) & 0xFF, reg16 & 0xFF])
        r = i2c_msg.read(self.address, 2)
        self.bus.i2c_rdwr(w, r)
        b = list(r)
        return (b[0] << 8) | b[1]
    
    def trigger_frame(self):
        """프레임 캡처 트리거"""
        self.bus.i2c_rdwr(i2c_msg.write(self.address, [0x80, 0x0D, 0x00, 0x01]))
    
    def read_temperature(self):
        """
        중심 픽셀(12, 16) 온도 읽기
        
        Returns:
            온도(℃) 또는 None
        """
        try:
            self.trigger_frame()
            time.sleep(0.02)  # 프레임 캡처 대기
            
            # 중심 픽셀 주소: 0x0400 + (12*32 + 16)
            raw = self.read_word16(0x0400 + (12 * 32 + 16))
            temp_c = raw * 0.01 - 273.15 + self.temp_offset
            
            # 유효 범위 클리핑 (-10°C ~ 150°C)
            temp_c = max(min(temp_c, 150.0), -10.0)
            
            return temp_c
        
        except Exception as e:
            rospy.logwarn(f"MLX90640 read error: {e}")
            return None


# === Arduino MQ-2 Gas Sensor ===
def read_mq2_from_arduino(ser):
    """
    아두이노에서 MQ-2 가스 센서 값 읽기
    
    Args:
        ser: serial.Serial 객체 (Arduino)
    
    Returns:
        가스 농도(ppm) 또는 None
    """
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            # 숫자만 포함된 라인만 파싱
            if line and line.replace('.', '').replace('-', '').isdigit():
                gas_value = float(line)
                return gas_value
    except (ValueError, UnicodeDecodeError) as e:
        rospy.logwarn_throttle(5, f"Arduino MQ-2 parse error: {e}")
    return None


# === Fire Detector (멀티센서 융합) ===
class FireDetector:
    """
    IR 온도 + Gas 농도 기반 화재 감지
    """
    def __init__(self, temp_threshold= -10.0, gas_threshold=350.0):
        self.temp_threshold = temp_threshold
        self.gas_threshold = gas_threshold
    
    def detect(self, ir_temp, gas_value):
        """
        화재 감지 및 신뢰도 계산
        
        Returns:
            (is_fire: bool, confidence: float)
        """
        if ir_temp is None and gas_value is None:
            return False, 0.0
        
        # 개별 점수 계산
        temp_score = 0.0
        gas_score = 0.0
        
        if ir_temp is not None:
            temp_score = min(ir_temp / self.temp_threshold, 1.0) if ir_temp < 0 else 0.0
        
        if gas_value is not None:
            gas_score = min(gas_value / self.gas_threshold, 1.0) if gas_value > 0 else 0.0
        
        # 가중 평균 (온도 70%, 가스 30%)
        confidence = temp_score * 0.7 + gas_score * 0.3
        
        # 화재 판단 (신뢰도 60% 이상)
        is_fire = confidence > 0.6
        
        return is_fire, confidence
