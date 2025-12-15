#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry

current_pos = None  # 항상 최신 위치 유지
start_pos = None    # 기록용
end_pos = None

def odom_callback(msg):
    global current_pos
    current_pos = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    )

def main():
    global start_pos, end_pos, current_pos

    print("=== AGV ODOM RECORD TOOL ===")

    rospy.init_node("odom_record_fixed", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    print("\n📌 AGV를 움직이지 말고 가만히 두세요.")
    input("▶ Enter 누르면 현재 위치를 START로 기록합니다.\n")
    start_pos = current_pos
    print(f"🟢 Start 기록완료: {start_pos}")

    input("\n📌 이제 AGV를 1m 이동시키고 Enter 누르세요.\n")
    end_pos = current_pos
    print(f"🔵 End 기록완료: {end_pos}")

    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]

    print("\n==========================")
    print(f"📌 이동 변화량")
    print(f"X 변화: {dx:.3f} m")
    print(f"Y 변화: {dy:.3f} m")
    print("==========================")

    with open("diff.txt", "w") as f:
        f.write(f"dx={dx:.3f}, dy={dy:.3f}\n")

    print("\n💾 diff.txt 저장 완료\n")

if __name__ == "__main__":
    main()
