#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

class AgvTaskManager:
    def __init__(self):
        rospy.init_node('agv_task_manager', anonymous=False)

        rospy.loginfo("AGV Task Manager (ROS1) Started!")

        # 상태 변수
        self.is_empty = False
        self.fence_open = False
        self.door_open = False

        # 구독
        rospy.Subscriber('/agv/is_empty', Bool, self.cb_is_empty)
        rospy.Subscriber('/agv/fence_open', Bool, self.cb_fence_open)
        rospy.Subscriber('/agv/door_open', Bool, self.cb_door_open)

    # 콜백 함수
    def cb_is_empty(self, msg):
        self.is_empty = msg.data
        rospy.loginfo(f"[AGV] is_empty = {self.is_empty}")

    def cb_fence_open(self, msg):
        self.fence_open = msg.data
        rospy.loginfo(f"[AGV] fence_open = {self.fence_open}")

    def cb_door_open(self, msg):
        self.door_open = msg.data
        rospy.loginfo(f"[AGV] door_open = {self.door_open}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = AgvTaskManager()
    node.spin()
