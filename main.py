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
        
        # === ROS 구독자 ===
        self.subscribe_topics()
        
        rospy.loginfo("🚁 FireDrone 시스템 초기화 완료")
    
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
    
    def yolo_conf_callback(self, msg):
        """YOLO 신뢰도 콜백"""
        self.yolo_confidence = msg.data
    
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
    
    def shutdown(self):
        """종료 처리"""
        rospy.loginfo("🛑 시스템 종료...")
        if self.ser_lidar: self.ser_lidar.close()
        if self.ser_arduino: self.ser_arduino.close()

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
