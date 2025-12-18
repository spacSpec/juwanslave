#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

import rospy, math, tf
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped  # 이 부분을 수정
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import Imu
# import geometry_msgs  # 만약 위처럼 안 하고 싶으면 이 줄을 추가해도 됩니다.

class GoalInterceptor:
    def __init__(self):
        rospy.init_node('goal_interceptor')
        
        self.current_pose = None
        self.current_yaw = 0.0
        
        # 구독 설정 수정 (geometry_msgs.msg. 가 아니라 그냥 클래스 이름만 쓰면 됩니다)
        rospy.Subscriber('/imu/data_filtered', Imu, self.cb_imu)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.cb_pose) # 여기 수정
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_goal)
        # 발행: 속도 제어, move_base 취소
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
        rospy.loginfo("🎯 Goal Interceptor with IMU Backward ready")

    def cb_imu(self, msg):
        orientation_q = msg.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def cb_pose(self, msg):
        self.current_pose = msg.pose.pose

    def cb_goal(self, msg):
        if self.current_pose is None: return

        # 1. 내 현재 위치와 목표 지점 사이의 거리/각도 계산
        dx = msg.pose.position.x - self.current_pose.position.x
        dy = msg.pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # 2. 내 현재 헤딩과 목표 방향의 차이(Relative Angle)
        rel_angle = target_angle - self.current_yaw
        while rel_angle > math.pi: rel_angle -= 2.0 * math.pi
        while rel_angle < -math.pi: rel_angle += 2.0 * math.pi

        # 3. 조건 판단: 거리가 2m 이내이고, 목표가 내 뒤쪽(90도~270도 사이)일 때
        if dist < 2.0 and abs(rel_angle) > math.radians(140):
            rospy.logwarn("⚠️ 목표가 뒤에 있음! move_base 취소 후 IMU 보정 후진 시작")
            self.cancel_pub.publish(GoalID()) # move_base 중단
            rospy.sleep(0.5)
            self.move_backward_imu(dist) # 거리만큼 후진 실행
        else:
            rospy.loginfo("✅ 정상 전진 주행 모드 (move_base가 처리함)")

    def move_backward_imu(self, target_dist):
        target_yaw = self.current_yaw
        kp = 1.5
        start_pos = self.current_pose
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 거리 계산
            curr_dist = math.sqrt((self.current_pose.position.x - start_pos.position.x)**2 + 
                                  (self.current_pose.position.y - start_pos.position.y)**2)
            
            if curr_dist >= target_dist: break
            
            error = target_yaw - self.current_yaw # Yaw 오차
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi

            t = Twist()
            t.linear.x = -0.1  # 후진 속도
            t.angular.z = error * kp # IMU 보정
            self.cmd_vel_pub.publish(t)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist()) # 정지
        rospy.loginfo("🏁 후진 도착 완료")

if __name__ == '__main__':
    GoalInterceptor()
    rospy.spin()