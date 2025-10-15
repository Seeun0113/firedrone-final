#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드롭 컨트롤러 (개선 버전)
- YOLO 타깃 수신 추가
- 투하 쿨다운 메커니즘
- 투하 후 검증 로직
"""

import math
import time
import rospy
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import PointStamped
from alpha_beta_filter import AlphaBetaFilter
from utils import calculate_fall_time, quat_to_euler
import config

class DropController:
    """
    투하 제어 시스템 (개선 버전)
    
    입력:
      - LiDAR 고도
      - IMU 자세 (Roll, Pitch)
      - GPS/비전 위치 (x, y)
      - 드론 속도 (vx, vy, vz)
      - 목표물 위치 (target_x, target_y)
    
    출력:
      - 투하 명령 (Servo PWM 신호)
    """
    
    def __init__(self, arduino_serial=None):
        # X, Y 축 각각 필터링
        self.filter_x = AlphaBetaFilter(config.FILTER_ALPHA, config.FILTER_BETA)
        self.filter_y = AlphaBetaFilter(config.FILTER_ALPHA, config.FILTER_BETA)
        
        # 아두이노 시리얼 연결 (투하 서보 제어용)
        self.arduino_serial = arduino_serial
        
        # 상태 변수
        self.h_f = None  # 필터링된 고도
        self.roll = 0.0
        self.pitch = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.px = 0.0
        self.py = 0.0
        
        # YOLO에서 받은 목표 좌표
        self.target_x = None
        self.target_y = None
        
        # 투하 쿨다운 (연속 투하 방지)
        self.last_drop_time = 0.0
        self.drop_cooldown = config.DROP_COOLDOWN
        
        # 투하 검증 모드
        self.drop_active = False
        
        # 가스 센서 값 저장 (검증용)
        self.last_gas_value = None
        
        rospy.loginfo("DropController 초기화 완료")
    
    def update_pose(self, pose_msg):
        """위치 업데이트 (MAVROS local_position/pose)"""
        self.px = pose_msg.pose.position.x
        self.py = pose_msg.pose.position.y
    
    def update_velocity(self, vel_msg):
        """속도 업데이트 (MAVROS local_position/velocity_local)"""
        self.vx = vel_msg.twist.linear.x
        self.vy = vel_msg.twist.linear.y
        self.vz = vel_msg.twist.linear.z
    
    def update_imu(self, imu_msg):
        """자세 업데이트 (MAVROS imu/data)"""
        q = imu_msg.orientation
        self.roll, self.pitch, _ = quat_to_euler(q.x, q.y, q.z, q.w)
    
    def update_lidar(self, dist):
        """
        LiDAR 고도 업데이트 및 기울기 보정
        
        실제 수직 고도 = 측정 거리 * cos(roll) * cos(pitch)
        """
        if dist:
            self.h_f = dist * math.cos(self.roll) * math.cos(self.pitch)
    
    def update_gas(self, gas_value):
        """가스 센서 값 업데이트 (검증용)"""
        if gas_value is not None:
            self.last_gas_value = gas_value
    
    def update_target(self, msg):
        """
        YOLO 검출 노드에서 발행한 목표 좌표 수신
        
        토픽: /fire_detector/target_local (PointStamped)
        좌표계: ENU (동-북-상) 로컬 프레임
        
        Args:
            msg: PointStamped 메시지
        """
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        
        # 필터로 좌표 추적 (노이즈 제거)
        dt = 0.1  # 토픽 발행 주기에 따라 조정 (예: 10Hz → 0.1초)
        self.target_x, _ = self.filter_x.update(self.target_x, dt)
        self.target_y, _ = self.filter_y.update(self.target_y, dt)
        
        rospy.logdebug(f"Target updated: ({self.target_x:.2f}, {self.target_y:.2f})")
    
    def compute_drop(self):
        """
        투하 조건 계산 및 판단 (쿨다운 추가 버전)
        
        Returns:
            bool: 투하 실행 여부
        
        투하 알고리즘:
        1. 안전 게이트: 고도/자세/수직속도 체크
        2. 낙하 시간 계산: t_f = sqrt(2h/g) * k_d
        3. 리드 거리 계산: Lead = 수평속도 * (낙하시간 + 지연시간)
        4. 진행 방향 오차 계산: forward, lateral
        5. 조건 만족 시 투하 실행
        """
        # 목표 좌표 유효성 체크
        if self.target_x is None or self.target_y is None:
            return False
        
        if self.h_f is None:
            return False
        
        # === 안전 게이트 ===
        # 고도 범위 체크
        if not (config.DROP_H_MIN <= self.h_f <= config.DROP_H_MAX):
            return False
        
        # 자세 안정성 체크
        if abs(self.roll) > math.radians(config.DROP_MAX_ROLL):
            return False
        if abs(self.pitch) > math.radians(config.DROP_MAX_PITCH):
            return False
        
        # 수직 속도 체크 (급상승/하강 방지)
        if abs(self.vz) > config.DROP_MAX_VZ:
            return False
        
        # === 탄도 계산 ===
        t_f = calculate_fall_time(self.h_f, config.DROP_K_D)
        V = math.sqrt(self.vx**2 + self.vy**2)  # 수평 속도
        Lead = V * (t_f + config.DROP_LATENCY)  # 리드 거리
        
        # === 목표 오차 계산 ===
        dx = self.target_x - self.px
        dy = self.target_y - self.py
        
        if V < 1e-3:  # 정지 상태
            forward = math.sqrt(dx**2 + dy**2)
            lateral = 0.0
        else:
            # 진행 방향 단위 벡터
            ux = self.vx / V
            uy = self.vy / V
            
            # 진행 방향 성분 (forward) / 측면 성분 (lateral)
            forward = dx * ux + dy * uy
            lateral = abs(-dx * uy + dy * ux)
        
        # === 투하 조건 판단 + 쿨다운 체크 ===
        if forward <= Lead and lateral <= config.DROP_E_MAX:
            # 마지막 투하 이후 충분한 시간이 지났는지 확인
            if time.time() - self.last_drop_time > self.drop_cooldown:
                rospy.loginfo(f"🔥 투하 실행! Lead={Lead:.2f}m, Forward={forward:.2f}m, Lateral={lateral:.2f}m")
                self.trigger_servo()
                self.last_drop_time = time.time()  # 현재 시각 저장
                self.drop_active = True  # 검증 모드 진입
                return True
            else:
                remaining = self.drop_cooldown - (time.time() - self.last_drop_time)
                rospy.logwarn(f"⏳ 쿨다운 중... {remaining:.1f}초 남음")
        
        return False
    
    def trigger_servo(self):
        """
        아두이노를 통한 서보 모터 제어 (투하 장치 작동)
        
        시리얼로 'fire' 명령 전송 → 아두이노가 서보 작동
        """
        if self.arduino_serial is None:
            rospy.logerr("❌ 아두이노 시리얼 연결 없음 (투하 불가)")
            return
        
        try:
            # 'fire' 명령 전송
            self.arduino_serial.write(b'fire\n')
            rospy.loginfo("✅ 투하 명령 전송 (Arduino)")
            
            # 아두이노 응답 대기 (선택사항)
            time.sleep(0.1)
            if self.arduino_serial.in_waiting > 0:
                response = self.arduino_serial.readline().decode('utf-8').strip()
                rospy.loginfo(f"Arduino: {response}")
        
        except Exception as e:
            rospy.logerr(f"❌ 서보 제어 실패: {e}")
    
    def verify_fire_extinguish(self, ir_temp, yolo_confidence):
        """
        투하 후 화재 진압 검증 (개선 버전)
        
        검증 조건:
        1. 투하 전 온도 대비 20°C 이상 하락
        2. 투하 전 가스 농도 대비 50% 이상 하락
        3. YOLO 신뢰도 0.3 이하로 하락
        
        Args:
            ir_temp: 현재 IR 온도 (°C)
            yolo_confidence: 현재 YOLO 신뢰도 (0~1)
        
        Returns:
            True: 진압 성공
            False: 아직 진압 안됨
            None: 검증 모드 아님
        """
        if not self.drop_active:
            return None
        
        # 투하 직후 대기 시간 체크
        if not hasattr(self, 'drop_verify_start_time'):
            self.drop_verify_start_time = time.time()
            # 투하 직전 센서 값 저장
            self.temp_before_drop = ir_temp if ir_temp else 999.0
            self.gas_before_drop = self.last_gas_value if self.last_gas_value else 999.0
            rospy.loginfo(f"📊 투하 전 상태: 온도={self.temp_before_drop:.1f}°C, 가스={self.gas_before_drop:.1f}ppm")
            return False
        
        # 대기 시간이 지나지 않았으면 계속 대기
        elapsed = time.time() - self.drop_verify_start_time
        if elapsed < config.VERIFY_WAIT_TIME:
            rospy.loginfo_throttle(1, f"⏳ 투하 후 {elapsed:.1f}초 경과... (대기 중)")
            return False
        
        # === 진압 검증 ===
        success_count = 0
        total_checks = 3
        
        # 1. 온도 체크
        if ir_temp is not None:
            temp_drop = self.temp_before_drop - ir_temp
            if temp_drop >= config.VERIFY_TEMP_DROP:
                success_count += 1
                rospy.loginfo(f"✅ 온도 하락 확인: {temp_drop:.1f}°C")
            else:
                rospy.loginfo(f"❌ 온도 하락 부족: {temp_drop:.1f}°C (필요: {config.VERIFY_TEMP_DROP}°C)")
        
        # 2. 가스 농도 체크 (옵션)
        current_gas = self.last_gas_value
        if current_gas is not None and self.gas_before_drop > 0:
            gas_ratio = current_gas / self.gas_before_drop
            if gas_ratio < config.VERIFY_GAS_DROP_RATIO:
                success_count += 1
                rospy.loginfo(f"✅ 가스 농도 하락: {gas_ratio*100:.0f}%")
            else:
                rospy.loginfo(f"❌ 가스 농도 유지: {gas_ratio*100:.0f}%")
        else:
            # 가스 센서 없으면 패스
            total_checks = 2
        
        # 3. YOLO 신뢰도 체크
        if yolo_confidence is not None:
            if yolo_confidence < config.VERIFY_YOLO_CONF_DROP:
                success_count += 1
                rospy.loginfo(f"✅ 화염 미검출: YOLO 신뢰도 {yolo_confidence:.2f}")
            else:
                rospy.loginfo(f"❌ 화염 여전히 존재: YOLO 신뢰도 {yolo_confidence:.2f}")
        else:
            total_checks = 2
        
        # === 최종 판정 ===
        if success_count >= total_checks * 0.67:  # 67% 이상 조건 만족
            rospy.loginfo("🎉 ✅ 화재 진압 성공!")
            self.drop_active = False
            self.target_x = None
            self.target_y = None
            delattr(self, 'drop_verify_start_time')
            return True
        else:
            rospy.logwarn(f"⚠️ 진압 실패 ({success_count}/{total_checks}개 조건 만족)")
            # 10초 후에도 진압 안되면 포기하고 다음 목표로
            if elapsed > 10.0:
                rospy.logwarn("❌ 진압 실패로 판정, 다음 목표로 이동")
                self.drop_active = False
                self.target_x = None
                self.target_y = None
                delattr(self, 'drop_verify_start_time')
                return False
            return False
    
    def reset(self):
        """투하 상태 리셋 (재사용 시)"""
        self.drop_active = False
        self.target_x = None
        self.target_y = None
        self.filter_x.reset()
        self.filter_y.reset()

