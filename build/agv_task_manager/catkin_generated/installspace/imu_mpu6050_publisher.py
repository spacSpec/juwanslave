#!/usr/bin/env python3
import rospy
import serial
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

def imu_node():
    rospy.init_node("imu_mpu6050_publisher")
    pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
    imu = Imu()

    imu.header.frame_id = "imu_link"

    while not rospy.is_shutdown():
        line = ser.readline().decode().strip()
        data = line.split(",")

        if len(data) != 7:
            continue
        
        ax,ay,az,gx,gy,gz,yaw = map(float,data)

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = math.radians(gx) # deg/s → rad/s
        imu.angular_velocity.y = math.radians(gy)
        imu.angular_velocity.z = math.radians(gz)

        q = quaternion_from_euler(0,0,math.radians(yaw)) # roll,pitch,yaw → quat
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]

        imu.header.stamp = rospy.Time.now()
        pub.publish(imu)

if __name__ == "__main__":
    imu_node()
