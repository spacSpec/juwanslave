#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

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
        # 2) PLC(M0) → ros_controller (topic)
        #    ros_controller → STM (service)
        #    /plc/door_state (Bool topic) → /plc/door_state (SetBool service)
        # ─────────────────────────────────────────────
        self.create_subscription(
            Bool,
            '/plc/door_state',
            self.cb_plc_door_state,
            10
        )

        # STM 쪽 SetBool 서비스 클라이언트 (/plc/door_state)
        self.stm_door_client = self.create_client(SetBool, '/plc/door_state')
        if not self.stm_door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "/plc/door_state SetBool service (STM) not available at startup"
            )

        # ─────────────────────────────────────────────
        # 3) ros_controller → AGV (publish)
        #    ⚠ door_open 은 이제 PLC가 아니라 STM 결과로만 퍼블리시함
        # ─────────────────────────────────────────────
        self.pub_empty = self.create_publisher(Bool, '/agv/is_empty', 10)
        self.pub_fence_open = self.create_publisher(Bool, '/agv/fence_open', 10)
        self.pub_door_open = self.create_publisher(Bool, '/agv/door_open', 10)

        # ─────────────────────────────────────────────
        # 4) ros_controller → RobotArm (publish)
        #    - fence_open 상태 공유
        # ─────────────────────────────────────────────
        self.pub_robotarm_fence_open = self.create_publisher(
            Bool,
            '/robotarm/fence_open',
            10
        )

        # ─────────────────────────────────────────────
        # 5) RobotArm → ros_controller (service server)
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
        # 6) PLC → ros_controller (service server, SetBool)
        #    ros_controller → RobotArm (service client, Trigger)
        #    /plc/robotarm_detect <-> /robot_arm/detect
        # ─────────────────────────────────────────────
        self.srv_plc_robotarm_detect = self.create_service(
            SetBool,
            '/plc/robotarm_detect',
            self.cb_plc_robotarm_detect
        )
        self.robotarm_detect_client = self.create_client(
            Trigger,
            '/robot_arm/detect'
        )

        if not self.robotarm_detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/robot_arm/detect not available at startup")

    # ─────────────────────────────────────────────
    #  PLC 통합 상태 콜백 (/plc/status_ros)
    # ─────────────────────────────────────────────
    def cb_plc_status(self, msg: PlcStatus):
        """
        PLC에서 온 /plc/status_ros(PlcStatus)를 받아서
        AGV와 RobotArm 쪽으로 각각 필요한 토픽을 퍼블리시.

        ⚠ PlcStatus.msg 에서 door_open 필드를 제거했으므로
          여기서는 is_empty, fence_open 만 사용한다.
        """

        # AGV로 전달 (is_empty, fence_open만)
        self.pub_empty.publish(Bool(data=msg.is_empty))
        self.pub_fence_open.publish(Bool(data=msg.fence_open))
        # self.pub_door_open.publish(Bool(data=msg.door_open))  # ← 더 이상 사용 안 함

        # RobotArm 쪽에도 fence_open 공유
        self.pub_robotarm_fence_open.publish(Bool(data=msg.fence_open))

        # 로그도 door_open 없이 찍기
        self.get_logger().info(
            f"[PLC STATUS] empty={msg.is_empty}, fence_open={msg.fence_open}"
        )

    # ─────────────────────────────────────────────
    #  PLC(M0) door_state 토픽 → STM SetBool 서비스 브릿지
    #    그리고 STM 결과로만 /agv/door_open 을 퍼블리시
    # ─────────────────────────────────────────────
    def cb_plc_door_state(self, msg: Bool):
        """
        plc_node가 /plc/door_state (Bool)를 publish 하면
        ros_controller가 /plc/door_state (SetBool) 서비스를 STM에게 호출해줌.

        - msg.data == True  → 문 열어달라는 명령으로 해석 → 서비스 호출
        - msg.data == False → 현재는 무시
        """
        self.get_logger().info(
            f"[PLC] /plc/door_state topic received: {msg.data}"
        )

        if not msg.data:
            # False 는 현재 '명령 없음/닫기 명령 없음' 으로 처리
            self.get_logger().info(
                "[STM] door_state=False → STM 서비스 호출 생략"
            )
            return

        if not self.stm_door_client.service_is_ready():
            self.get_logger().warn(
                "/plc/door_state SetBool service (STM) NOT ready"
            )
            return

        req = SetBool.Request()
        req.data = True   # stm_node는 True일 때만 문 열기 시도

        self.get_logger().info(
            "[STM] call /plc/door_state SetBool service (open door)"
        )

        future = self.stm_door_client.call_async(req)
        future.add_done_callback(self._on_stm_door_state_result)

    def _on_stm_door_state_result(self, future):
        """STM /plc/door_state 서비스 결과 처리를 위한 콜백."""
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"[STM] /plc/door_state call exception: {e}")
            return

        if res is None:
            self.get_logger().error("[STM] /plc/door_state result is None")
            return

        self.get_logger().info(
            f"[STM] /plc/door_state response: "
            f"success={res.success}, message='{res.message}'"
        )

        # ★ 여기서 AGV로 door_open 상태를 전달한다.
        #    - success=True → 문 열림 성공 → door_open=True
        #    - success=False → 문 열림 실패 → door_open=False 로 간주
        door_open = bool(res.success)
        self.pub_door_open.publish(Bool(data=door_open))
        self.get_logger().info(
            f"[AGV] /agv/door_open publish: {door_open} (from STM result)"
        )

    # ─────────────────────────────────────────────
    #  RobotArm → AGV 서비스 브릿지
    # ─────────────────────────────────────────────
    def cb_request_dispatch(self, request, response):
        """
        로봇암 task_manager에서 Bool 요청을 받으면
        AGV의 /agv/request_dispatch(SetBool) 서비스에 그대로 전달.
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
    #  PLC(SetBool) → ros_controller → RobotArm(Trigger) 서비스 브릿지
    # ─────────────────────────────────────────────
    def cb_plc_robotarm_detect(self, request, response):
        """
        PLC가 /plc/robotarm_detect(SetBool)을 호출하면,
        ros_controller가 /robot_arm/detect(Trigger)로 검사 요청을 보내고,
        RobotArm의 결과("GOOD"/"BAD"/...)를 SetBool 응답(success/message)으로
        변환하여 PLC에게 반환.
        """

        if not self.robotarm_detect_client.service_is_ready():
            self.get_logger().warn("/robot_arm/detect NOT ready")
            response.success = False
            response.message = "RobotArm service not available"
            return response

        self.get_logger().info(
            f"[PLC] /plc/robotarm_detect called, data={request.data} "
            f"→ call /robot_arm/detect (Trigger)"
        )

        trigger_req = Trigger.Request()
        future = self.robotarm_detect_client.call_async(trigger_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("RobotArm /robot_arm/detect call failed")
            response.success = False
            response.message = "RobotArm service call failed"
            return response

        arm_res = future.result()
        self.get_logger().info(
            f"[RobotArm] /robot_arm/detect response: "
            f"success={arm_res.success}, message='{arm_res.message}'"
        )

        if not arm_res.success:
            response.success = False
            response.message = arm_res.message or "DETECT_FAILED"
            return response

        quality = (arm_res.message or "").upper()

        if quality == "GOOD":
            response.success = True
            response.message = "GOOD"
        elif quality == "BAD":
            response.success = False
            response.message = "BAD"
        else:
            self.get_logger().warn(
                f"[RobotArm] unknown quality '{arm_res.message}', treat as BAD"
            )
            response.success = False
            response.message = arm_res.message or "UNKNOWN"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RosController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
