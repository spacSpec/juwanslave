#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# -------------------------------
# 그래프 저장 버퍼
# -------------------------------
max_len = 500
imu_z = deque(maxlen=max_len)
odom_z = deque(maxlen=max_len)

# -------------------------------
# ROS 콜백 — 필요한 값 ONLY
# -------------------------------
def imu_cb(msg):
    imu_z.append(msg.angular_velocity.z)

def odom_cb(msg):
    odom_z.append(msg.twist.twist.angular.z)

# -------------------------------
# 플롯 업데이트
# -------------------------------
def update_graph(frame):
    plt.cla()

    if imu_z:
        plt.plot(list(imu_z), label="IMU filtered yaw rate", color="orange")
    if odom_z:
        plt.plot(list(odom_z), label="ODOM yaw rate", color="blue")

    plt.legend(loc="upper left")
    plt.title("IMU vs ODOM Yaw Angular Velocity (Z)")
    plt.xlabel("Samples")
    plt.ylabel("Yaw Rate (rad/s)")
    plt.ylim([-2, 2])
    plt.grid(True)

# -------------------------------
# Q 저장 기능
# -------------------------------
def on_key(event):
    if event.key == 'q':
        print("\n[INFO] Q pressed → Saving graph as yaw_compare.png ...")
        fig.canvas.draw()
        fig.savefig("yaw_compare.png")
        print("[INFO] Saved! Exiting...")
        rospy.signal_shutdown("User Exit")
        plt.close()
        sys.exit(0)

# -------------------------------
# 메인
# -------------------------------
if __name__ == '__main__':
    rospy.init_node('compare_imu_odom_yaw')

    rospy.Subscriber("/imu/data_filtered", Imu, imu_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)

    print("\n===============================")
    print(" 실시간 그래프 실행 중")
    print(" 비교 대상: IMU yaw rate vs ODOM yaw rate")
    print(" Q 키 → 그래프 저장 후 종료")
    print("===============================\n")

    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', on_key)

    ani = animation.FuncAnimation(fig, update_graph, interval=50)
    plt.show()
