#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def odom_callback(msg):
    # 쿼터니언 데이터를 가져옴
    orientation_q = msg.pose.pose.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    # 쿼터니언을 오일러 각도(라디안)로 변환
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    
    # 라디안을 도(Degree) 단위로 변환
    yaw_degrees = math.degrees(yaw)
    
    # 출력 (-180 ~ 180도 범위)
    rospy.loginfo("Current Yaw: {:.2f} Degrees".format(yaw_degrees))

def main():
    rospy.init_node('yaw_checker_node', anonymous=True)
    # EKF 결과 토픽 구독 (본인의 토픽명에 맞게 수정 가능)
    rospy.Subscriber("/odometry/data_filtered", Odometry, odom_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass