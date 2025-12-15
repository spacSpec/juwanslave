#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# 최대 몇 포인트까지 그릴지 (메모리 폭발 방지)
MAX_POINTS = 3000

# 각 궤적 저장용 (최근 N개)
wheel_x, wheel_y = deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS)
ekf_x,   ekf_y   = deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS)
dr_x,    dr_y    = deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS)

def wheel_cb(msg):
    p = msg.pose.pose.position
    wheel_x.append(p.x)
    wheel_y.append(p.y)

def ekf_cb(msg):
    p = msg.pose.pose.position
    ekf_x.append(p.x)
    ekf_y.append(p.y)

def dr_cb(msg):
    p = msg.pose.pose.position
    dr_x.append(p.x)
    dr_y.append(p.y)

def main():
    rospy.init_node("agv_pose_live_plot", anonymous=True)

    # 토픽명 필요하면 여기서 바꿔도 됨
    wheel_topic = "/odom"
    ekf_topic   = "/odometry/filtered"
    dr_topic    = "/dr_odom"   # 없으면 그냥 안 그려짐

    rospy.Subscriber(wheel_topic, Odometry, wheel_cb, queue_size=50)
    rospy.Subscriber(ekf_topic,   Odometry, ekf_cb,   queue_size=50)
    rospy.Subscriber(dr_topic,    Odometry, dr_cb,    queue_size=50)

    # ===== Matplotlib 초기 설정 =====
    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title("AGV trajectories (live)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal", adjustable="box")

    line_wheel, = ax.plot([], [], 'b-', label="Wheel odometry")
    line_ekf,   = ax.plot([], [], 'r-', label="Robot pose EKF")
    line_dr,    = ax.plot([], [], 'g-', label="Dead reckoning")

    ax.legend(loc="upper right")
    ax.grid(True)

    rate = rospy.Rate(10)  # 10 Hz로 화면 업데이트

    while not rospy.is_shutdown():
        # 데이터가 하나도 없으면 그냥 대기
        if len(wheel_x) == 0 and len(ekf_x) == 0 and len(dr_x) == 0:
            rate.sleep()
            continue

        # 각 궤적 데이터 세팅
        if len(wheel_x) > 0:
            line_wheel.set_data(wheel_x, wheel_y)
        if len(ekf_x) > 0:
            line_ekf.set_data(ekf_x, ekf_y)
        if len(dr_x) > 0:
            line_dr.set_data(dr_x, dr_y)

        # 축 범위 자동 조정
        all_x = list(wheel_x) + list(ekf_x) + list(dr_x)
        all_y = list(wheel_y) + list(ekf_y) + list(dr_y)

        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)

        margin = 0.3
        ax.set_xlim(xmin - margin, xmax + margin)
        ax.set_ylim(ymin - margin, ymax + margin)

        plt.draw()
        plt.pause(0.01)  # GUI 이벤트 처리

        rate.sleep()

    # 노드 종료 시 블록 해제
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
