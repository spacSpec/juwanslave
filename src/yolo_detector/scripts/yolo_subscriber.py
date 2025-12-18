#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

model = YOLO("/home/vboxuser/myagv_ros/src/yolo_detector/best.pt")
bridge = CvBridge()
decision_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

red_lower = np.array([0, 120, 70])
red_upper = np.array([10, 255, 255])
yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([30, 255, 255])

frame_global = None

def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, red_lower, red_upper)
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    red_count = cv2.countNonZero(mask_red)
    yellow_count = cv2.countNonZero(mask_yellow)

    if red_count > 300:
        return "RIGHT"
    elif yellow_count > 300:
        return "LEFT"
    return None

def callback(msg):
    global frame_global
    frame_global = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def listener():
    rospy.init_node('yolo_camera_subscriber', anonymous=True)
    rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, callback)
    rate = rospy.Rate(30)

    # 10초 동안 카메라만 표시하고 나서 판별 시작
    start_time = time.time()
    display_duration = 3  # 초 단위

    rospy.loginfo(f"📷 카메라 화면 표시 중... ({display_duration}초 후 판별 시작)")

    while not rospy.is_shutdown():
        if frame_global is not None:
            frame = frame_global.copy()
            elapsed = time.time() - start_time

            # 10초 동안은 단순히 영상만 보여줌
            if elapsed < display_duration:
                cv2.imshow("Camera View (Preview)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                rate.sleep()
                continue  # 판별하지 않고 다음 루프로 넘어감

            # 10초 이후부터 색 판별 시작
            detected = detect_colors(frame)

            if detected:
                rospy.loginfo(f"✅ Detected color: {detected}")
                twist = Twist()

                if detected == "LEFT":
                    twist.angular.z = 0.5
                elif detected == "RIGHT":
                    twist.angular.z = -0.5
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                # 회전 명령 2초 동안 유지
                end_time = time.time() + 2.0
                while time.time() < end_time and not rospy.is_shutdown():
                    decision_pub.publish(twist)
                    rate.sleep()

                # 정지 명령 한 번 보내기
                stop = Twist()
                decision_pub.publish(stop)

                rospy.loginfo("🟢 회전 완료, 자율주행 노드에 제어권 반환")
                rospy.signal_shutdown("Color detected and action done")
                break

            cv2.imshow("Camera View (Detecting...)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()
