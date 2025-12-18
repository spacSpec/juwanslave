#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class BackwardTester:
    def __init__(self):
        rospy.init_node('backward_test_node')
        
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        
        # 1. IMU 데이터 구독 (로봇에 맞게 토픽 확인: /imu/data_filtered)
        rospy.Subscriber('/imu/data_filtered', Imu, self.cb_imu)
        
        # 2. 속도 명령 발행
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("⏳ IMU 데이터를 기다리는 중...")
        rospy.sleep(2.0) # IMU 데이터 수신 대기
        
    def cb_imu(self, msg):
        orientation_q = msg.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def get_yaw_error(self, target, current):
        error = target - current
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        return error

    def run_test(self, speed, duration, kp):
        self.target_yaw = self.current_yaw
        rospy.loginfo(f"🚀 테스트 시작! 목표 각도: {math.degrees(self.target_yaw):.2f}")
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            error = self.get_yaw_error(self.target_yaw, self.current_yaw)
            
            t = Twist()
            t.linear.x = -abs(speed) # 후진
            
            # 보정 방향 확인용 (부호를 바꿔가며 테스트)
            # 1. 만약 로봇이 더 꺾이면 아래 kp 앞에 '-'를 붙이거나 빼보세요.
            t.angular.z = error * kp 
            
            self.cmd_vel_pub.publish(t)
            
            # 현재 상태 출력 (디버깅용)
            rospy.loginfo(f"오차: {math.degrees(error):.2f}, 보정값(z): {t.angular.z:.2f}")
            rate.sleep()

        self.cmd_vel_pub.publish(Twist()) # 정지
        rospy.loginfo("🏁 테스트 완료")

if __name__ == '__main__':
    try:
        tester = BackwardTester()
        
        # 테스트 설정: (속도 0.1, 지속시간 3초, 보정강도 1.5)
        # 💡 만약 이상하면 1.5를 -1.5로 바꿔보세요.
        tester.run_test(speed=0.1, duration=5.0, kp=1.5)
        
    except rospy.ROSInterruptException:
        pass