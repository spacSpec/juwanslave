#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller')

        self.get_logger().info("ROS Controller Started.")

        # ─────────────────────────────────────────────
        #  PLC → ROS_CONTROLLER (subscribe)
        # ─────────────────────────────────────────────
        self.create_subscription(Bool, '/plc/is_empty', self.cb_plc_empty, 10)
        self.create_subscription(Bool, '/plc/fence_open', self.cb_plc_fence_open, 10)
        self.create_subscription(Bool, '/door_open', self.cb_stm_door_open, 10)

        # ─────────────────────────────────────────────
        #  ROS_CONTROLLER → AGV_TASK_MANAGER (publish)
        # ─────────────────────────────────────────────
        self.pub_empty = self.create_publisher(Bool, '/agv/is_empty', 10)
        self.pub_fence_open = self.create_publisher(Bool, '/agv/fence_open', 10)
        self.pub_door_open = self.create_publisher(Bool, '/agv/door_open', 10)

        # ─────────────────────────────────────────────
        #  ROBOT_ARM(task_manager) → ros_controller (service server)
        #  ros_controller → AGV(task_manager)          (service client)
        # ─────────────────────────────────────────────
        self.srv_request_dispatch = self.create_service(
            SetBool,
            '/ros_controller/request_dispatch',
            self.cb_request_dispatch
        )

        self.agv_client = self.create_client(SetBool, '/agv/request_dispatch')

        # AGV 서비스 서버가 뜰 때까지 기다려줌
        while not self.agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /agv/request_dispatch service...")

    # ─────────────────────────────────────────────
    #  CALLBACKS (topics)
    # ─────────────────────────────────────────────
    def cb_plc_empty(self, msg):
        self.pub_empty.publish(msg)
        self.get_logger().info(f'[PLC] is_empty → AGV: {msg.data}')

    def cb_plc_fence_open(self, msg):
        self.pub_fence_open.publish(msg)
        self.get_logger().info(f'[PLC] fence_open → AGV: {msg.data}')

    def cb_stm_door_open(self, msg):
        self.pub_door_open.publish(msg)
        self.get_logger().info(f'[STM] door_open → AGV: {msg.data}')

    # ─────────────────────────────────────────────
    #  SERVICE BRIDGE: robot_arm → ros_controller → AGV
    # ─────────────────────────────────────────────
    def cb_request_dispatch(self, request, response):
        """
        로봇암 task_manager에서 Bool 요청을 받으면
        AGV task_manager의 /agv/request_dispatch 서비스에 그대로 전달
        """
        req = SetBool.Request()
        req.data = request.data

        self.get_logger().info(f"[RobotArm] request_dispatch: {request.data} → forwarding to AGV")

        future = self.agv_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            agv_result = future.result()

            self.get_logger().info(
                f"[AGV] request_dispatch response: {agv_result.success}, message={agv_result.message}"
            )

            response.success = agv_result.success
            response.message = agv_result.message
        else:
            self.get_logger().error("Failed to call /agv/request_dispatch")
            response.success = False
            response.message = "AGV service call failed"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RosController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
