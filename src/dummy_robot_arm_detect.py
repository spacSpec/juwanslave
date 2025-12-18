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
        # 여기서 마음대로 응답 만들 수 있음
        self.get_logger().info("Request from ros_controller received!")

        # ✅ GOOD 보내고 싶으면
        response.success = True
        response.message = "GOOD"

        # ❌ BAD 보내고 싶으면 위 두 줄 대신:
        # response.success = True
        # response.message = "BAD"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DummyRobotArmDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
