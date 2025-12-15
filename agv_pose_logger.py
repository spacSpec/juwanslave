#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import rospy
from nav_msgs.msg import Odometry

# 최신 값 저장용
odom_wheel = None
odom_ekf = None
odom_dr = None

def wheel_cb(msg):
    global odom_wheel
    odom_wheel = msg

def ekf_cb(msg):
    global odom_ekf
    odom_ekf = msg

def dr_cb(msg):
    global odom_dr
    odom_dr = msg

def main():
    global odom_wheel, odom_ekf, odom_dr

    rospy.init_node("agv_pose_logger")

    # 파라미터로 토픽 이름/파일 이름 바꿀 수 있게
    wheel_topic = rospy.get_param("~wheel_odom_topic", "/odom")
    ekf_topic   = rospy.get_param("~ekf_odom_topic", "/odometry/filtered")
    dr_topic    = rospy.get_param("~dr_odom_topic", "/dr_odom")
    filename    = rospy.get_param("~log_file", "agv_pose_log.csv")
    rate_hz     = rospy.get_param("~rate", 20.0)

    rospy.loginfo("AGV pose logger start")
    rospy.loginfo("  wheel_odom_topic: %s", wheel_topic)
    rospy.loginfo("  ekf_odom_topic:   %s", ekf_topic)
    rospy.loginfo("  dr_odom_topic:    %s", dr_topic)
    rospy.loginfo("  log_file:         %s", filename)

    rospy.Subscriber(wheel_topic, Odometry, wheel_cb, queue_size=10)
    rospy.Subscriber(ekf_topic,   Odometry, ekf_cb,   queue_size=10)
    rospy.Subscriber(dr_topic,    Odometry, dr_cb,    queue_size=10)

    f = open(filename, "w", newline="")
    writer = csv.writer(f)

    # CSV 헤더
    writer.writerow([
        "time",
        "wheel_x", "wheel_y",
        "ekf_x", "ekf_y",
        "dr_x", "dr_y"
    ])

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()

        # 값 없으면 None으로 기록
        def get_xy(odom_msg):
            if odom_msg is None:
                return "", ""
            p = odom_msg.pose.pose.position
            return f"{p.x:.6f}", f"{p.y:.6f}"

        wheel_x, wheel_y = get_xy(odom_wheel)
        ekf_x,   ekf_y   = get_xy(odom_ekf)
        dr_x,    dr_y    = get_xy(odom_dr)

        writer.writerow([
            f"{now:.6f}",
            wheel_x, wheel_y,
            ekf_x, ekf_y,
            dr_x, dr_y
        ])
        f.flush()

        rate.sleep()

    f.close()
    rospy.loginfo("AGV pose logger finished, file closed: %s", filename)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
