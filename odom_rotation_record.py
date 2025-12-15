#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

current_yaw = None   # 항상 최신 yaw 유지 (라디안)
start_yaw = None     # 기록용
end_yaw = None


def normalize_angle(angle):
    """각도를 -pi ~ pi 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def odom_callback(msg):
    global current_yaw

    # 오도메트리에서 쿼터니언 추출
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]

    # roll, pitch, yaw 추출 (라디안)
    _, _, yaw = euler_from_quaternion(quat)
    current_yaw = yaw


def main():
    global start_yaw, end_yaw, current_yaw

    print("=== AGV ODOM ROTATION RECORD TOOL ===")

    rospy.init_node("odom_rotation_record", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    # 처음에 콜백이 한 번이라도 들어올 때까지 대기
    print("\n📡 /odom 데이터 수신 대기 중...")
    while current_yaw is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    print("✅ /odom 수신 시작됨!\n")

    print("📌 AGV를 움직이지 말고 가만히 두세요.")
    input("▶ Enter 누르면 현재 Yaw(회전각)를 START로 기록합니다.\n")
    start_yaw = current_yaw
    print(f"🟢 Start Yaw 기록완료: {start_yaw:.4f} rad  ({math.degrees(start_yaw):.2f} deg)")

    print("\n📌 이제 AGV를 원하는 만큼 회전시키세요.")
    print("   (예: 시계 방향으로 90도 회전 후 Enter)")
    input("▶ 회전이 끝나면 Enter를 누르세요.\n")
    end_yaw = current_yaw
    print(f"🔵 End Yaw 기록완료:   {end_yaw:.4f} rad  ({math.degrees(end_yaw):.2f} deg)")

    # 회전 변화량 계산 (wrap-around 고려)
    raw_diff = end_yaw - start_yaw
    diff = normalize_angle(raw_diff)
    diff_deg = math.degrees(diff)

    print("\n==========================")
    print("📌 회전 변화량")
    print(f"Yaw 변화(라디안): {diff:.4f} rad")
    print(f"Yaw 변화(도):     {diff_deg:.2f} deg")
    print("==========================")

    with open("rotation_diff.txt", "w") as f:
        f.write(f"start_yaw_rad={start_yaw:.4f}, end_yaw_rad={end_yaw:.4f}\n")
        f.write(f"diff_rad={diff:.4f}, diff_deg={diff_deg:.2f}\n")

    print("\n💾 rotation_diff.txt 저장 완료\n")


if __name__ == "__main__":
    main()
