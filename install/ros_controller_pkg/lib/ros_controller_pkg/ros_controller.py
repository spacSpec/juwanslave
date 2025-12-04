#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from ros_controller_pkg.msg import PlcStatus


class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller')

        self.get_logger().info("ROS Controller Started.")

        # ─────────────────────────────────────────────
        # 1) PLC → ros_controller (통합 상태 /plc/status_ros)
        # ─────────────────────────────────────────────
        self.create_subscription(
            PlcStatus,
            '/plc/status_ros',
            self.cb_plc_status,
            10
        )

        # ─────────────────────────────────────────────
        # 2) ros_controller → AGV (publish)
        # ─────────────────────────────────────────────
        self.pub_empty = self.create_publisher(Bool, '/agv/is_empty', 10)
        self.pub_fence_open = self.create_publisher(Bool, '/agv/fence_open', 10)
        self.pub_door_open = self.create_publisher(Bool, '/agv/door_open', 10)

        # ─────────────────────────────────────────────
        # 3) ros_controller → RobotArm (publish)
        #    - door_open 상태 공유
        # ─────────────────────────────────────────────
        self.pub_robotarm_door_open = self.create_publisher(Bool, '/robotarm/door_open', 10)

        # ─────────────────────────────────────────────
        # 4) RobotArm → ros_controller (service server)
        #    ros_controller → AGV (service client)
        #    /ros_controller/request_dispatch <-> /agv/request_dispatch
        # ─────────────────────────────────────────────
        self.srv_request_dispatch = self.create_service(
            SetBool,
            '/ros_controller/request_dispatch',
            self.cb_request_dispatch
        )
        self.agv_client = self.create_client(SetBool, '/agv/request_dispatch')

        if not self.agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/agv/request_dispatch not available at startup")

        # ─────────────────────────────────────────────
        # 5) PLC → ros_controller (service server)
        #    ros_controller → RobotArm (service client)
        #    /plc/robotarm_detect <-> /robotarm/robotarm_detect
        # ─────────────────────────────────────────────
        self.srv_plc_robotarm_detect = self.create_service(
            SetBool,
            '/plc/robotarm_detect',
            self.cb_plc_robotarm_detect
        )
        self.robotarm_client = self.create_client(SetBool, '/robotarm/robotarm_detect')

        if not self.robotarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/robotarm/robotarm_detect not available at startup")

    # ─────────────────────────────────────────────
    #  PLC 통합 상태 콜백
    # ─────────────────────────────────────────────
    def cb_plc_status(self, msg: PlcStatus):
        """
        PLC에서 온 /plc/status_ros(PlcStatus)를 받아서
        AGV와 RobotArm 쪽으로 각각 필요한 토픽을 퍼블리시.
        """

        # AGV로 전달
        self.pub_empty.publish(Bool(data=msg.is_empty))
        self.pub_fence_open.publish(Bool(data=msg.fence_open))
        self.pub_door_open.publish(Bool(data=msg.door_open))

        # RobotArm 쪽에도 door_open 공유
        self.pub_robotarm_door_open.publish(Bool(data=msg.door_open))

        self.get_logger().info(
            f"[PLC STATUS] empty={msg.is_empty}, "
            f"fence_open={msg.fence_open}, door_open={msg.door_open}"
        )

    # ─────────────────────────────────────────────
    #  RobotArm → AGV 서비스 브릿지
    # ─────────────────────────────────────────────
    def cb_request_dispatch(self, request, response):
        """
        로봇암 task_manager에서 Bool 요청을 받으면
        AGV의 /agv/request_dispatch 서비스에 그대로 전달.
        """

        if not self.agv_client.service_is_ready():
            self.get_logger().warn("/agv/request_dispatch NOT ready")
            response.success = False
            response.message = "AGV service not available"
            return response

        req = SetBool.Request()
        req.data = request.data

        self.get_logger().info(
            f"[RobotArm] request_dispatch: {request.data} → /agv/request_dispatch"
        )

        future = self.agv_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            agv_result = future.result()

            self.get_logger().info(
                f"[AGV] response: success={agv_result.success}, "
                f"message={agv_result.message}"
            )

            response.success = agv_result.success
            response.message = agv_result.message
        else:
            self.get_logger().error("AGV service call failed")
            response.success = False
            response.message = "AGV service call failed"

        return response

    # ─────────────────────────────────────────────
    #  PLC → ros_controller → RobotArm 서비스 브릿지
    # ─────────────────────────────────────────────
    def cb_plc_robotarm_detect(self, request, response):
        """
        PLC가 /plc/robotarm_detect(SetBool)을 호출하면,
        ros_controller가 /robotarm/robotarm_detect(SetBool) 으로 그대로 전달.
        """

        if not self.robotarm_client.service_is_ready():
            self.get_logger().warn("/robotarm/robotarm_detect NOT ready")
            response.success = False
            response.message = "RobotArm service not available"
            return response

        req = SetBool.Request()
        req.data = request.data  # 보통 True: 검사 시작

        self.get_logger().info(
            f"[PLC] /plc/robotarm_detect: {request.data} → /robotarm/robotarm_detect"
        )

        future = self.robotarm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            arm_res = future.result()

            self.get_logger().info(
                f"[RobotArm] response: success={arm_res.success}, "
                f"message={arm_res.message}"
            )

            # RobotArm 응답을 그대로 PLC에게 반환
            response.success = arm_res.success
            response.message = arm_res.message
        else:
            self.get_logger().error("RobotArm service call failed")
            response.success = False
            response.message = "RobotArm service call failed"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RosController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
