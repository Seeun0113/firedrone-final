#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
유틸리티 함수 모음
쿼터니언 변환, 거리 계산 등
"""

import math

def quat_to_euler(qx, qy, qz, qw):
    """
    쿼터니언을 오일러 각도(Roll, Pitch, Yaw)로 변환
    
    Args:
        qx, qy, qz, qw: 쿼터니언 성분
    
    Returns:
        (roll, pitch, yaw): 라디안 단위의 오일러 각도
    """
    # Roll (x축 회전)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y축 회전)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    # Yaw (z축 회전)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def calculate_distance_2d(x1, y1, x2, y2):
    """
    2D 평면에서 두 점 사이의 거리 계산
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_fall_time(height, k_d=1.15):
    """
    자유낙하 시간 계산 (공기저항 보정 포함)
    
    Args:
        height: 낙하 고도 (m)
        k_d: 공기저항 보정 계수
    
    Returns:
        낙하 시간 (초)
    """
    if height <= 0:
        return 0.0
    return math.sqrt(2 * height / 9.81) * k_d

