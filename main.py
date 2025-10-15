#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
메인 실행 노드 (간소화 버전)
센서 읽기 → 화재 감지 → 투하 판단 → 진압 검증
"""

import rospy
import serial
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import State

import config
from sensor_readers import read_tfluna_latest, MLX90640Reader, read_mq2_from_arduino, FireDetector
from drop_controller import DropController

class FireDroneSystem:
    """화재 감지 및 자동 투하 시스템"""
    
    def __init__(self):
        rospy.init_node("fire_drone_system")
        
        # === 센서 초기화 ===
        self.init_sensors()
        
        # === 컨트롤러 초기화 ===
        self.controller = DropController(arduino_serial=self.ser_arduino)
        self.fire_detector = FireDetector(config.FIRE_TEMP_THRESHOLD, config.FIRE_GAS_THRESHOLD)
        
        # YOLO 신뢰도
        self.yolo_confidence = None
        
        # 드론 상태
        self.current_state = None
        self.is_armed = False
        
        # 웨이포인트 컨트롤러 (순회 비행용)
        self.waypoint_controller = None
        
        # === ROS 구독자 ===
        self.subscribe_topics()
        
        # === 이륙 조건 체크 ===
        if config.TAKEOFF_ENABLED:
            self.check_takeoff_conditions()
            # 자동 이륙 실행
            self.auto_takeoff()
            # 이륙 완료 후 순회 비행 시작
            self.start_waypoint_patrol()
        
        rospy.loginfo("🚁 FireDrone 시스템 초기화 완료")
    
    def check_takeoff_conditions(self):
        """이륙 전 안전 조건 체크"""
        rospy.loginfo("🔍 이륙 조건 체크 중...")
        
        # === 필수 센서 체크 ===
        if config.TAKEOFF_SENSOR_CHECK:
            # LiDAR 필수 체크
            if config.REQUIRED_LIDAR and self.ser_lidar is None:
                rospy.logerr("❌ 이륙 불가: LiDAR 센서 연결 실패 (필수 센서)")
                rospy.signal_shutdown("LiDAR 센서 필요")
                return False
            
            # Arduino 필수 체크
            if config.REQUIRED_ARDUINO and self.ser_arduino is None:
                rospy.logerr("❌ 이륙 불가: Arduino 센서 연결 실패 (필수 센서)")
                rospy.signal_shutdown("Arduino 센서 필요")
                return False
            
            # IR 센서 선택 체크
            if config.REQUIRED_IR and self.ir_reader is None:
                rospy.logerr("❌ 이륙 불가: IR 센서 연결 실패 (필수 센서)")
                rospy.signal_shutdown("IR 센서 필요")
                return False
            elif not config.REQUIRED_IR and self.ir_reader is None:
                rospy.logwarn("⚠️ IR 센서 연결 실패 - 화재 감지 정확도 저하")
        
        # === 배터리 체크 (예시) ===
        # 실제로는 MAVROS에서 배터리 정보를 받아야 함
        rospy.loginfo("✅ 센서 상태: 정상")
        rospy.loginfo("✅ 이륙 조건 만족")
        return True
    
    def init_sensors(self):
        """센서 초기화"""
        # TF-Luna LiDAR
        try:
            self.ser_lidar = serial.Serial(config.TFLUNA_PORT, config.TFLUNA_BAUDRATE, timeout=0)
            self.ser_lidar.reset_input_buffer()
            rospy.loginfo(f"✅ TF-Luna: {config.TFLUNA_PORT}")
        except Exception as e:
            rospy.logerr(f"❌ TF-Luna 실패: {e}")
            self.ser_lidar = None
        
        # MLX90640 IR
        try:
            self.ir_reader = MLX90640Reader(config.MLX90640_I2C_BUS, config.MLX90640_ADDRESS)
            rospy.loginfo(f"✅ MLX90640: I2C Bus {config.MLX90640_I2C_BUS}")
        except Exception as e:
            rospy.logerr(f"❌ MLX90640 실패: {e}")
            self.ir_reader = None
        
        # Arduino (서보 + 가스)
        try:
            self.ser_arduino = serial.Serial(config.ARDUINO_SERVO_GAS_PORT, config.ARDUINO_SERVO_GAS_BAUDRATE, timeout=0.1)
            rospy.loginfo(f"✅ Arduino: {config.ARDUINO_SERVO_GAS_PORT}")
        except Exception as e:
            rospy.logerr(f"❌ Arduino 실패: {e}")
            self.ser_arduino = None
    
    def subscribe_topics(self):
        """ROS 토픽 구독"""
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.controller.update_pose)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.controller.update_velocity)
        rospy.Subscriber("/mavros/imu/data", Imu, self.controller.update_imu)
        rospy.Subscriber("/fire_detector/target_local", PointStamped, self.controller.update_target)
        rospy.Subscriber("/fire_detector/confidence", Float32, self.yolo_conf_callback)
        rospy.Subscriber("/mavros/state", State, self.state_callback)
    
    def yolo_conf_callback(self, msg):
        """YOLO 신뢰도 콜백"""
        self.yolo_confidence = msg.data
    
    def state_callback(self, msg):
        """드론 상태 콜백"""
        self.current_state = msg
        self.is_armed = msg.armed
    
    def set_mode(self, mode):
        """드론 모드 설정"""
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_srv(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"✅ 모드 변경: {mode}")
                return True
            else:
                rospy.logerr(f"❌ 모드 변경 실패: {mode}")
                return False
        except Exception as e:
            rospy.logerr(f"모드 변경 에러: {e}")
            return False
    
    def arm_drone(self):
        """드론 Armed"""
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_srv(True)
            if response.success:
                rospy.loginfo("✅ 드론 Armed")
                return True
            else:
                rospy.logerr("❌ Armed 실패")
                return False
        except Exception as e:
            rospy.logerr(f"Arming 에러: {e}")
            return False
    
    def auto_takeoff(self):
        """자동 이륙 실행"""
        rospy.loginfo("🚁 자동 이륙 시작...")
        
        # 1. GUIDED 모드로 전환
        if not self.set_mode("GUIDED"):
            rospy.logerr("❌ GUIDED 모드 전환 실패")
            return False
        
        rospy.sleep(2)  # 모드 전환 대기
        
        # 2. 드론 Armed
        if not self.arm_drone():
            rospy.logerr("❌ Armed 실패")
            return False
        
        rospy.sleep(2)  # Armed 대기
        
        # 3. 이륙 명령
        try:
            rospy.wait_for_service('/mavros/cmd/takeoff', timeout=5)
            takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_srv(
                altitude=config.TAKEOFF_ALTITUDE,
                latitude=0,  # 현재 위치에서 이륙
                longitude=0,
                min_pitch=0
            )
            
            if response.success:
                rospy.loginfo(f"✅ 이륙 명령 전송: {config.TAKEOFF_ALTITUDE}m")
                
                # 이륙 완료 대기
                self.wait_for_takeoff_complete()
                return True
            else:
                rospy.logerr("❌ 이륙 명령 실패")
                return False
                
        except Exception as e:
            rospy.logerr(f"이륙 명령 에러: {e}")
            return False
    
    def wait_for_takeoff_complete(self):
        """이륙 완료 대기"""
        rospy.loginfo("⏳ 이륙 중...")
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30)  # 30초 타임아웃
        
        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > timeout:
                rospy.logerr("❌ 이륙 타임아웃")
                return False
            
            # 현재 고도 확인
            if hasattr(self.controller, 'h_f') and self.controller.h_f is not None:
                if self.controller.h_f >= config.TAKEOFF_ALTITUDE * 0.9:  # 90% 도달
                    rospy.loginfo(f"✅ 이륙 완료: {self.controller.h_f:.2f}m")
                    return True
            
            rospy.sleep(0.5)
        
        return False
    
    def start_waypoint_patrol(self):
        """웨이포인트 순회 비행 시작"""
        if not config.PATROL_ENABLED:
            return
        
        rospy.loginfo("🚁 웨이포인트 순회 비행 시작...")
        
        # 웨이포인트 컨트롤러를 별도 스레드에서 실행
        import threading
        from waypoint_controller import WaypointController
        
        self.waypoint_controller = WaypointController()
        patrol_thread = threading.Thread(target=self.waypoint_controller.run)
        patrol_thread.daemon = True
        patrol_thread.start()
    
    def stop_waypoint_patrol(self):
        """웨이포인트 순회 비행 중지"""
        rospy.loginfo("🛑 웨이포인트 순회 비행 중지")
        if self.waypoint_controller:
            # OFFBOARD 모드에서 POSHOLD 모드로 전환
            self.set_mode("POSHOLD")
    
    def emergency_land(self):
        """비상 착륙"""
        rospy.loginfo("🚨 비상 착륙 시작!")
        try:
            # LAND 모드로 전환
            self.set_mode("LAND")
            rospy.loginfo("✅ LAND 모드로 전환됨")
        except Exception as e:
            rospy.logerr(f"❌ 비상 착륙 실패: {e}")
    
    def safe_shutdown(self):
        """안전한 시스템 종료"""
        rospy.loginfo("🛑 안전한 시스템 종료 시작...")
        
        # 1. 순회 비행 중지
        self.stop_waypoint_patrol()
        
        # 2. POSHOLD 모드로 전환 (호버링)
        self.set_mode("POSHOLD")
        
        # 3. 2초 대기
        rospy.sleep(2)
        
        rospy.loginfo("✅ 안전한 종료 완료")
    
    def read_sensors(self):
        """센서 값 읽기"""
        # LiDAR
        dist, strength, temp = None, None, None
        if self.ser_lidar:
            dist, strength, temp = read_tfluna_latest(self.ser_lidar)
            if dist:
                self.controller.update_lidar(dist)
        
        # IR 온도
        ir_temp = None
        if self.ir_reader:
            ir_temp = self.ir_reader.read_temperature()
        
        # 가스 농도
        gas_value = None
        if self.ser_arduino:
            gas_value = read_mq2_from_arduino(self.ser_arduino)
            self.controller.update_gas(gas_value)
        
        return dist, ir_temp, gas_value
    
    def run(self):
        """메인 루프"""
        rate = rospy.Rate(config.ROS_RATE_HZ)
        rospy.loginfo("🔥 메인 루프 시작...")
        
        try:
            while not rospy.is_shutdown():
                # 1. 센서 읽기
                dist, ir_temp, gas_value = self.read_sensors()
                
                # 2. 화재 감지
                is_fire, fire_conf = self.fire_detector.detect(ir_temp, gas_value)
                if is_fire:
                    rospy.loginfo_throttle(2, f"🔥 화재 감지! ({fire_conf:.2%})")
                
                # 3. 투하 판단
                self.controller.compute_drop()
                
                # 4. 투하 검증
                verify_result = self.controller.verify_fire_extinguish(ir_temp, self.yolo_confidence)
                if verify_result is True:
                    rospy.loginfo("✅ 진압 완료! 다음 목표 탐색...")
                
                # 5. 로그 출력 (5초마다)
                log_str = "📊 "
                log_str += f"LiDAR={dist:.2f}m " if dist else "LiDAR=N/A "
                log_str += f"IR={ir_temp:.1f}°C " if ir_temp else "IR=N/A "
                log_str += f"Gas={gas_value:.0f}ppm " if gas_value else "Gas=N/A "
                log_str += f"Fire={fire_conf:.1%}"
                rospy.loginfo_throttle(5, log_str)
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 키보드 인터럽트 감지!")
            self.safe_shutdown()
        except Exception as e:
            rospy.logerr(f"❌ 시스템 오류: {e}")
            self.emergency_land()
    
    def shutdown(self):
        """종료 처리"""
        rospy.loginfo("🛑 시스템 종료...")
        if self.ser_lidar: self.ser_lidar.close()
        if self.ser_arduino: self.ser_arduino.close()
        rospy.loginfo("✅ 모든 시리얼 포트 종료 완료")

if __name__ == "__main__":
    try:
        system = FireDroneSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("사용자 중단")
    finally:
        if 'system' in locals():
            system.shutdown()
