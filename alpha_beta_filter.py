#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
알파-베타 필터 (Alpha-Beta Filter)
목표물의 위치와 속도를 추정하여 추적 성능 향상
칼만 필터의 단순화 버전
"""

class AlphaBetaFilter:
    """
    1차원 알파-베타 필터
    
    동작 원리:
    1. 예측 단계: x_pred = x + v * dt
    2. 보정 단계: x = x_pred + α * (측정값 - x_pred)
                  v = v + β * (측정값 - x_pred) / dt
    
    α (알파): 위치 보정 가중치 (0~1, 높을수록 측정값 신뢰)
    β (베타): 속도 보정 가중치 (0~1, 높을수록 변화율 민감)
    """
    
    def __init__(self, alpha=0.6, beta=0.4):
        """
        Args:
            alpha: 위치 보정 계수 (권장 0.5~0.8)
            beta: 속도 보정 계수 (권장 0.3~0.5)
        """
        self.alpha = alpha
        self.beta = beta
        self.x = None  # 현재 위치 추정값
        self.v = 0.0   # 현재 속도 추정값
    
    def update(self, z, dt):
        """
        새로운 측정값으로 상태 업데이트
        
        Args:
            z: 센서 측정값 (위치)
            dt: 이전 업데이트 이후 경과 시간 (초)
        
        Returns:
            (x, v): 필터링된 위치와 속도
        """
        # 첫 측정값은 그대로 초기화
        if self.x is None:
            self.x = z
            self.v = 0.0
            return self.x, self.v
        
        # 예측 단계
        x_pred = self.x + self.v * dt
        v_pred = self.v
        
        # 보정 단계 (잔차 기반)
        residual = z - x_pred
        self.x = x_pred + self.alpha * residual
        self.v = v_pred + (self.beta / max(dt, 1e-6)) * residual
        
        return self.x, self.v
    
    def reset(self):
        """필터 상태 초기화"""
        self.x = None
        self.v = 0.0

