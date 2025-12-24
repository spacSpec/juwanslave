#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque
import time

# 데이터 저장용 큐 (최근 100개의 데이터 유지)
max_points = 100
times = deque(maxlen=max_points)
linear_x = deque(maxlen=max_points)
angular_z = deque(maxlen=max_points)

start_time = time.time()

def cmd_vel_callback(msg):
    current_time = time.time() - start_time
    times.append(current_time)
    linear_x.append(msg.linear.x)
    angular_z.append(msg.angular.z)

def listener():
    rospy.init_node('cmd_vel_visualizer', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    # 그래프 설정
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    line1, = ax1.plot([], [], 'r-', label='Linear X (m/s)')
    line2, = ax2.plot([], [], 'b-', label='Angular Z (rad/s)')

    ax1.set_title("Real-time /cmd_vel Monitor")
    ax1.set_ylabel("Velocity")
    ax1.legend(loc='upper right')
    ax1.grid(True)

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angular Velocity")
    ax2.legend(loc='upper right')
    ax2.grid(True)

    while not rospy.is_shutdown():
        if len(times) > 0:
            # 데이터 업데이트
            line1.set_data(list(times), list(linear_x))
            line2.set_data(list(times), list(angular_z))

            # 축 범위 자동 조정
            for ax in [ax1, ax2]:
                ax.relim()
                ax.autoscale_view()
                if len(times) > 1:
                    ax.set_xlim(times[0], times[-1])

            fig.canvas.draw()
            fig.canvas.flush_events()
        
        plt.pause(0.1)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass