#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import Imu
import tf

pub = None

def imu_callback(msg: Imu):
    global pub

    # ========== 1. raw quaternion 추출 ==========
    q = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]

    # ========== 2. quaternion → RPY 변환 ==========
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)

    # ★ 여기서 yaw 보정(원하는 방향으로 변경 가능)
    # 예: -180~180 → 0~360 로 정규화
    yaw_norm = (yaw + math.pi*2) % (math.pi*2)

    # ---- 필요 시 offset 적용 (예: IMU 장착 방향 틀어짐 보정)
    # offset_deg = 90         # 예시, 90도 회전 보정
    # yaw_norm = yaw_norm + math.radians(offset_deg)

    # ========== 3. yaw → quaternion 재생성 ==========
    q_new = tf.transformations.quaternion_from_euler(roll, pitch, yaw_norm)

    msg.orientation.x = q_new[0]
    msg.orientation.y = q_new[1]
    msg.orientation.z = q_new[2]
    msg.orientation.w = q_new[3]

    # ========== 4. publish ==========
    pub.publish(msg)

    # 출력 확인용 (deg 보기 쉽게)
    rospy.loginfo("Yaw raw: %.1f° → corrected: %.1f°",
                  yaw*180/math.pi, yaw_norm*180/math.pi)


def main():
    global pub
    rospy.init_node('imu_rpy_correct_pub', anonymous=False)
    rospy.loginfo("IMU Corrected Quaternion Publisher Started!")

    # 입력: madgwick 결과
    rospy.Subscriber("/imu/data_filtered", Imu, imu_callback)

    # 출력: 보정된 IMU 토픽
    pub = rospy.Publisher("/imu/data_corrected", Imu, queue_size=10)

    rospy.spin()


if __name__=="__main__":
    main()
