#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

def callback(msg):
    # 1. 시간을 현재 시각으로 덮어쓰기
    msg.header.stamp = rospy.Time.now()
    # 2. 프레임을 imu_link가 아닌 base_footprint로 직접 지정 (에러 원인 제거)
    msg.header.frame_id = "base_footprint" 
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_time_fixer')
    pub = rospy.Publisher('/imu/data_fixed', Imu, queue_size=10)
    rospy.Subscriber('/imu/data_filtered', Imu, callback)
    rospy.spin()