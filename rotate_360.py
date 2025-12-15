#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi

# 1. 초기화: ROS 노드 초기화
rospy.init_node('rotate_360_test', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 2. 파라미터 설정
ANGULAR_SPEED = 0.5  # 회전 속도 (rad/s)
TARGET_ANGLE_RAD = 2 * pi  # 360도 = 2 * pi 라디안 (약 6.28 rad)
DURATION = TARGET_ANGLE_RAD / ANGULAR_SPEED # 필요한 시간: 6.28 / 0.5 = 12.56 s

rate = rospy.Rate(10)  # 10Hz

# 3. 명령 생성 및 실행
cmd = Twist()
cmd.angular.z = ANGULAR_SPEED # 시계 반대 방향 회전

rospy.loginfo(f"로봇이 {DURATION:.2f}초 동안 {ANGULAR_SPEED} rad/s로 회전합니다.")
start_time = rospy.Time.now().to_sec()

while (rospy.Time.now().to_sec() - start_time) < DURATION:
    pub.publish(cmd)
    rate.sleep()

# 4. 정지 명령
cmd.angular.z = 0.0
pub.publish(cmd)
rospy.loginfo("회전 명령 완료 및 정지.")