#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage


class CompressedImageLatencyDebug(object):
    def __init__(self):
        rospy.init_node("camera_latency_debug", anonymous=False)
        self.sub = rospy.Subscriber(
            "/camera/image/compressed",
            CompressedImage,
            self.callback,
            queue_size=1
        )

        self.last_print_time = time.time()
        self.frame_count = 0

        rospy.loginfo("camera_latency_debug started. Subscribing /camera/image/compressed")

    def callback(self, msg):
        # JPEG → 이미지
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("cv2.imdecode() returned None")
            return

        h, w = frame.shape[:2]

        # ★ 카메라 촬영 시각과 현재 시각 차이
        delay = (rospy.Time.now() - msg.header.stamp).to_sec()

        self.frame_count += 1
        t = time.time()
        if t - self.last_print_time >= 1.0:
            fps = self.frame_count / (t - self.last_print_time)
            rospy.loginfo(
                "size=%dx%d, fps=%.1f, latency=%.3fs",
                w, h, fps, delay
            )
            self.frame_count = 0
            self.last_print_time = t

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = CompressedImageLatencyDebug()
        node.spin()
    except rospy.ROSInterruptException:
        pass
