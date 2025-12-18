#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DummyRobotArmDetect(Node):
    def __init__(self):
        super().__init__("dummy_robot_arm_detect")

        # /robot_arm/detect 서비스 서버 생성
        self.srv = self.create_service(
            Trigger,
            "/robot_arm/detect",
            self.cb_detect
        )
        self.get_logger().info("Dummy /robot_arm/detect server READY")

    def cb_detect(self, request, response):
        self.get_logger().info("Request from ros_controller received!")

        # 🔥 여기서 응답 값 **꼭** 채워주고
        response.success = True   # 또는 False
        response.message = "GOOD" # 또는 "BAD", "ERROR" 등

        # 🔥 그리고 마지막에 return response 가 꼭 필요함
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DummyRobotArmDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
