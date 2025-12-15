#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math
import sys

# 그래프 저장 버퍼
max_len = 300
imu_raw_z = deque(maxlen=max_len)
imu_filt_z = deque(maxlen=max_len)
odom_z = deque(maxlen=max_len)
ekf_z = deque(maxlen=max_len)

# 콜백 함수
def imu_raw_cb(msg):
    imu_raw_z.append(msg.angular_velocity.z)

def imu_filt_cb(msg):
    imu_filt_z.append(msg.angular_velocity.z)

def odom_cb(msg):
    odom_z.append(msg.twist.twist.angular.z)

def ekf_cb(msg):
    ekf_z.append(msg.twist.twist.angular.z)

# 플롯 업데이트 함수
def update_graph(frame):
    plt.cla()
    plt.plot(list(imu_raw_z), label="IMU raw", color="red")
    plt.plot(list(imu_filt_z), label="IMU filtered", color="orange")
    plt.plot(list(odom_z), label="ODOM", color="blue")
    plt.plot(list(ekf_z), label="EKF fused", color="green")

    plt.legend(loc="upper left")
    plt.title("IMU / ODOM / EKF Angular Velocity.z")
    plt.ylim([-2, 2])  # 필요시 수정
    plt.grid(True)

# 키보드 이벤트 콜백 (Q 버튼 저장)
def on_key(event):
    if event.key == 'q':
        print("\n[INFO] Q pressed → Saving current graph as plot_output.png ...")

        # 현재 화면 그대로 렌더링
        fig.canvas.draw()

        # 현재 화면을 이미지로 저장
        fig.savefig("plot_output.png")

        print("[INFO] Image saved! Exiting...")
        rospy.signal_shutdown("User exit")
        plt.close()
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('plot_all')

    rospy.Subscriber("/imu/data_raw", Imu, imu_raw_cb)
    rospy.Subscriber("/imu/data_filtered", Imu, imu_filt_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.Subscriber("/odometry/filtered", Odometry, ekf_cb)

    fig = plt.figure()
    
    # 키보드 이벤트 등록
    fig.canvas.mpl_connect('key_press_event', on_key)

    ani = animation.FuncAnimation(fig, update_graph, interval=50)
    plt.show()
