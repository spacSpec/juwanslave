#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


class CompressedImageViewer(object):
    def __init__(self):
        rospy.init_node("camera_compressed_viewer", anonymous=False)

        self.sub = rospy.Subscriber(
            "/camera/image/compressed",
            CompressedImage,
            self.callback,
            queue_size=1
        )

        rospy.loginfo("camera_compressed_viewer started. Subscribing /camera/image/compressed")

    def callback(self, msg: CompressedImage):
        # JPEG 바이트 -> OpenCV 이미지(BGR)
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("cv2.imdecode() returned None")
            return

        # 화면에 표시
        cv2.imshow("Camera from AGV (/camera/image/compressed)", frame)
        key = cv2.waitKey(1) & 0xFF

        # ESC 누르면 노드 종료
        if key == 27:
            rospy.loginfo("ESC pressed. Shutting down camera_compressed_viewer.")
            rospy.signal_shutdown("ESC pressed")

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = CompressedImageViewer()
        node.spin()
    except rospy.ROSInterruptException:
        pass
