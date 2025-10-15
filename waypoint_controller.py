#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
웨이포인트 기반 자동 비행 컨트롤러
로컬 좌표계 기반 (GPS 불필요)
"""

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import config

class WaypointController:
    """
    웨이포인트 자동 순회 비행 컨트롤러
    
    동작 순서:
    1. OFFBOARD 모드 전환
    2. 웨이포인트 순차 이동
    3. 각 지점에서 호버링
    4. 다음 지점으로 이동
    """
    
    def __init__(self):
        self.current_pose = None
        self.current_state = None
        
        self.waypoints = config.PATROL_WAYPOINTS
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.hover_start_time = None
        
        # Publisher
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        rospy.loginfo("WaypointController 초기화 완료")
    
    def pose_callback(self, msg):
        """드론 현재 위치 업데이트"""
        self.current_pose = msg
    
    def state_callback(self, msg):
        """드론 상태 업데이트"""
        self.current_state = msg
    
    def distance_to_waypoint(self, waypoint):
        """현재 위치에서 웨이포인트까지 거리 계산"""
        if self.current_pose is None:
            return float('inf')
        
        dx = waypoint['x'] - self.current_pose.pose.position.x
        dy = waypoint['y'] - self.current_pose.pose.position.y
        dz = waypoint['z'] - self.current_pose.pose.position.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def set_offboard_mode(self):
        """OFFBOARD 모드로 전환"""
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            
            response = set_mode_srv(custom_mode='OFFBOARD')
            if response.mode_sent:
                rospy.loginfo("✅ OFFBOARD 모드 활성화")
                return True
            else:
                rospy.logerr("❌ OFFBOARD 모드 전환 실패")
                return False
        except Exception as e:
            rospy.logerr(f"모드 전환 에러: {e}")
            return False
    
    def arm(self):
        """드론 Armed 상태로 전환"""
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=5)
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            
            response = arm_srv(True)
            if response.success:
                rospy.loginfo("✅ 드론 Armed")
                return True
            else:
                rospy.logerr("❌ Arming 실패")
                return False
        except Exception as e:
            rospy.logerr(f"Arming 에러: {e}")
            return False
    
    def publish_setpoint(self, waypoint):
        """목표 위치 발행"""
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        
        setpoint.pose.position.x = waypoint['x']
        setpoint.pose.position.y = waypoint['y']
        setpoint.pose.position.z = waypoint['z']
        
        # 자세는 기본값 (yaw=0)
        setpoint.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(setpoint)
    
    def run(self):
        """메인 자동 비행 루프"""
        rate = rospy.Rate(20)  # 20Hz (OFFBOARD 모드 유지를 위해 높은 주기 필요)
        
        rospy.loginfo("🚁 자동 비행 준비 중...")
        
        # OFFBOARD 모드 진입 전 setpoint를 먼저 전송해야 함 (PX4 요구사항)
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self.publish_setpoint(self.waypoints[0])
            rate.sleep()
        
        # OFFBOARD 모드 전환
        self.set_offboard_mode()
        
        rospy.loginfo("🛫 웨이포인트 순회 비행 시작!")
        
        while not rospy.is_shutdown():
            if self.current_waypoint_idx >= len(self.waypoints):
                rospy.loginfo("✅ 모든 웨이포인트 완료!")
                break
            
            current_wp = self.waypoints[self.current_waypoint_idx]
            
            # 현재 웨이포인트로 setpoint 발행
            self.publish_setpoint(current_wp)
            
            # 도달 판정
            dist = self.distance_to_waypoint(current_wp)
            
            if dist < 0.5:  # 0.5m 이내 도달
                if not self.waypoint_reached:
                    rospy.loginfo(f"📍 웨이포인트 {self.current_waypoint_idx} 도착! (x={current_wp['x']}, y={current_wp['y']}, z={current_wp['z']})")
                    self.waypoint_reached = True
                    self.hover_start_time = time.time()
                
                # 호버링 시간 체크
                if time.time() - self.hover_start_time >= config.PATROL_HOVER_TIME:
                    rospy.loginfo(f"⏭️ 다음 웨이포인트로 이동...")
                    self.current_waypoint_idx += 1
                    self.waypoint_reached = False
                    self.hover_start_time = None
            else:
                rospy.loginfo_throttle(2, f"🎯 웨이포인트 {self.current_waypoint_idx}로 이동 중... (거리: {dist:.2f}m)")
            
            rate.sleep()
        
        rospy.loginfo("🏁 자동 비행 종료")

if __name__ == "__main__":
    try:
        rospy.init_node("waypoint_controller")
        controller = WaypointController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 인터럽트")
    except KeyboardInterrupt:
        rospy.loginfo("사용자 중단")
