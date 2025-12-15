#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# 1. 초기화: ROS 노드 초기화
rospy.init_node('move_1m_test', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 2. 파라미터 설정
LINEAR_SPEED = 0.2  # 선형 속도 (m/s)
TARGET_DISTANCE = 1.0 # 목표 거리 (m)
DURATION = TARGET_DISTANCE / LINEAR_SPEED # 필요한 시간: 1.0 / 0.2 = 5.0 s

rate = rospy.Rate(10) # 10Hz

# 3. 명령 생성 및 실행
cmd = Twist()
cmd.linear.x = LINEAR_SPEED # 전진 속도 설정

rospy.loginfo(f"로봇이 {DURATION:.2f}초 동안 {LINEAR_SPEED} m/s로 전진합니다.")
start_time = rospy.Time.now().to_sec()

while (rospy.Time.now().to_sec() - start_time) < DURATION:
    pub.publish(cmd)
    rate.sleep()

# 4. 정지 명령
cmd.linear.x = 0.0
pub.publish(cmd)
rospy.loginfo("전진 명령 완료 및 정지.")