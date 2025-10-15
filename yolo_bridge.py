#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 화재 검출 → ROS 토픽 발행 브릿지
YOLOv5/YOLOv8로 화재를 검출하고 중심 좌표를 /fire_detector/target_local로 발행
"""

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import cv2
import numpy as np

class YOLOFireBridge:
    """
    YOLO 검출 결과를 ROS 토픽으로 브릿지
    
    발행 토픽:
    - /fire_detector/target_local (PointStamped): 화재 중심 좌표 (ENU)
    - /fire_detector/confidence (Float32): 검출 신뢰도
    """
    
    def __init__(self):
        rospy.init_node("yolo_fire_bridge")
        
        # YOLO 모델 로드 (예시)
        # 실제 사용 시 아래 주석 해제:
        # import torch
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
        #                             path='weights/fire_yolo.pt')
        
        # ROS 퍼블리셔
        self.pub_target = rospy.Publisher(
            '/fire_detector/target_local', 
            PointStamped, 
            queue_size=10
        )
        self.pub_conf = rospy.Publisher(
            '/fire_detector/confidence', 
            Float32, 
            queue_size=10
        )
        
        # 카메라 파라미터 (예시)
        self.fov_h = 60.0  # 수평 FOV (도)
        self.fov_v = 45.0  # 수직 FOV (도)
        self.img_width = 640
        self.img_height = 480
        
        rospy.loginfo("✅ YOLOFireBridge 초기화 완료")
    
    def pixel_to_enu(self, bbox_center_x, bbox_center_y, drone_alt, drone_x=0.0, drone_y=0.0):
        """
        픽셀 좌표를 ENU 좌표로 변환
        
        간단한 핀홀 카메라 모델 가정
        (실제 구현은 카메라 내부/외부 파라미터 필요)
        
        Args:
            bbox_center_x: bbox 중심 x 픽셀
            bbox_center_y: bbox 중심 y 픽셀
            drone_alt: 드론 고도 (m)
            drone_x: 드론 현재 x 위치 (ENU)
            drone_y: 드론 현재 y 위치 (ENU)
        
        Returns:
            (x, y): ENU 좌표 (미터)
        """
        # 이미지 중심으로부터의 픽셀 오프셋
        pixel_offset_x = bbox_center_x - (self.img_width / 2)
        pixel_offset_y = bbox_center_y - (self.img_height / 2)
        
        # 픽셀 → 각도 변환
        angle_x = (pixel_offset_x / self.img_width) * self.fov_h
        angle_y = (pixel_offset_y / self.img_height) * self.fov_v
        
        # 각도 → 지면 거리 변환 (탄젠트)
        # 카메라가 아래를 향한다고 가정 (gimbal 각도 고려 필요)
        offset_x = drone_alt * np.tan(np.radians(angle_x))
        offset_y = drone_alt * np.tan(np.radians(angle_y))
        
        # 드론 위치 기준 절대 좌표
        enu_x = drone_x + offset_x
        enu_y = drone_y - offset_y  # 이미지 y축은 아래가 양수
        
        return enu_x, enu_y
    
    def run(self):
        """
        카메라 스트림에서 YOLO 검출 실행
        """
        # 카메라 초기화
        cap = cv2.VideoCapture(0)  # 카메라 장치 번호
        
        if not cap.isOpened():
            rospy.logerr("❌ 카메라 열기 실패")
            return
        
        rospy.loginfo("🎥 카메라 스트리밍 시작...")
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("프레임 읽기 실패")
                continue
            
            # === YOLO 추론 ===
            # 실제 사용 시:
            # results = self.model(frame)
            # detections = results.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2, conf, cls]
            
            # === 데모용 가짜 검출 ===
            # 실제 화재 검출 시 이 부분을 YOLO 결과로 교체
            fake_detection = True  # 화재 검출 여부
            
            if fake_detection:
                # 가짜 bbox (중앙에 화재가 있다고 가정)
                x1, y1, x2, y2 = 280, 200, 360, 280
                conf = 0.85  # 신뢰도
                
                # bbox 중심 계산
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # 픽셀 → ENU 변환 (드론 고도 3m 가정)
                # 실제로는 /mavros/local_position/pose에서 고도를 받아야 함
                drone_alt = 3.0
                enu_x, enu_y = self.pixel_to_enu(center_x, center_y, drone_alt)
                
                # === PointStamped 발행 ===
                msg_target = PointStamped()
                msg_target.header.stamp = rospy.Time.now()
                msg_target.header.frame_id = "map"
                msg_target.point.x = enu_x
                msg_target.point.y = enu_y
                msg_target.point.z = 0.0  # 지면 고도
                
                self.pub_target.publish(msg_target)
                
                # === 신뢰도 발행 ===
                msg_conf = Float32()
                msg_conf.data = conf
                self.pub_conf.publish(msg_conf)
                
                rospy.loginfo_throttle(2, f"🎯 Fire detected: ENU=({enu_x:.2f}, {enu_y:.2f}), conf={conf:.2f}")
                
                # 시각화 (선택)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(frame, f"Fire {conf:.2f}", (int(x1), int(y1)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 화면 출력 (선택)
            cv2.imshow("YOLO Fire Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rate.sleep()
        
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        bridge = YOLOFireBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 인터럽트")
    except KeyboardInterrupt:
        rospy.loginfo("사용자 중단")

