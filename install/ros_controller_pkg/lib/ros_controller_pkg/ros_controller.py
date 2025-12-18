#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
# [수정 1] 멀티스레드 및 콜백 그룹 임포트
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool, Trigger

from ros_controller_pkg.msg import PlcStatus

# 🔥 DB 헬퍼 함수 (이미 만들어 둔 모듈)
from db_ros import insert_ros_quality


class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller')
        
        # [수정 2] 재진입 가능한 콜백 그룹 생성 (중첩 호출 허용)
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info("ROS Controller Started (Multi-Threaded Mode).")

        # ─────────────────────────────────────────────
        # 0) 양품 / 불량 카운트 & M0 상태
        # ─────────────────────────────────────────────
        self.good_count = 0
        self.bad_count = 0

        # PLC M0 상태 저장 (/plc/door_state 토픽 == M0)
        self.m0_state = False           # 현재 M0 상태
        self._last_m0_logged = None     # 직전에 DB에 기록한 M0 (변화 감지용)

        # 카운트 퍼블리셔
        self.pub_good_count = self.create_publisher(
            Int32, '/ros_controller/good_count', 10
        )
        self.pub_bad_count = self.create_publisher(
            Int32, '/ros_controller/bad_count', 10
        )

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
        # 2-1) PLC(M0) → ros_controller (topic)
        #      ros_controller → STM (service /plc/door_state, SetBool)
        # ─────────────────────────────────────────────
        self.create_subscription(
            Bool,
            '/plc/door_state',          # == PLC M0 상태라고 보면 됨
            self.cb_plc_door_state,
            10
        )

        # [수정 3] 클라이언트에 콜백 그룹 적용
        self.stm_door_client = self.create_client(
            SetBool, 
            '/plc/door_state',
            callback_group=self.cb_group
        )
        if not self.stm_door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "/plc/door_state SetBool service (STM) not available at startup"
            )

        # ─────────────────────────────────────────────
        # 2-2) PLC(M30) → ros_controller (topic)
        #      ros_controller → RobotArm start (service /system/start_work, Trigger)
        # ─────────────────────────────────────────────
        self.create_subscription(
            Bool,
            '/plc/start_task',
            self.cb_plc_start_task,
            10
        )

        self.robot_start_client = self.create_client(
            Trigger,
            '/system/start_work',
            callback_group=self.cb_group
        )
        if not self.robot_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/system/start_work not available at startup")

        # ─────────────────────────────────────────────
        # 3) ros_controller → AGV (publish)
        #    door_open 은 STM 결과로만 퍼블리시함
        # ─────────────────────────────────────────────
        self.pub_empty = self.create_publisher(Bool, '/agv/is_empty', 10)
        self.pub_fence_open = self.create_publisher(Bool, '/agv/fence_open', 10)
        self.pub_door_open = self.create_publisher(Bool, '/agv/door_open', 10)

        # ─────────────────────────────────────────────
        # 4) ros_controller → RobotArm (publish)
        #    fence_open 공유
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
            self.cb_request_dispatch,
            callback_group=self.cb_group
        )
        self.agv_client = self.create_client(
            SetBool, 
            '/agv/request_dispatch',
            callback_group=self.cb_group
        )

        if not self.agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/agv/request_dispatch not available at startup")

        # ─────────────────────────────────────────────
        # 6) PLC → ros_controller (service server, SetBool)
        #    ros_controller → RobotArm (service client, Trigger)
        #    /plc/robotarm_detect <-> /robot_arm/detect
        # ─────────────────────────────────────────

        self.srv_plc_robotarm_detect = self.create_service(
            SetBool,
            '/plc/robotarm_detect',
            self.cb_plc_robotarm_detect,
            callback_group=self.cb_group  # [핵심] 서버에 그룹 적용
        )
        self.robotarm_detect_client = self.create_client(
            Trigger,
            '/robot_arm/detect',
            callback_group=self.cb_group  # [핵심] 클라이언트에 그룹 적용
        )

        if not self.robotarm_detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/robot_arm/detect not available at startup")

    # ─────────────────────────────────────────────
    #  PLC 통합 상태 콜백 (/plc/status_ros)
    # ─────────────────────────────────────────────
    def cb_plc_status(self, msg: PlcStatus):
        self.pub_empty.publish(Bool(data=msg.is_empty))
        self.pub_fence_open.publish(Bool(data=msg.fence_open))
        self.pub_robotarm_fence_open.publish(Bool(data=msg.fence_open))

        self.get_logger().info(
            f"[PLC STATUS] empty={msg.is_empty}, fence_open={msg.fence_open}"
        )

    # ─────────────────────────────────────────────
    #  PLC(M0) door_state 토픽 → STM SetBool 서비스 브릿지
    #  + M0 상태 변화시 DB 로그
    # ─────────────────────────────────────────────
    def cb_plc_door_state(self, msg: Bool):
        self.get_logger().info(
            f"[PLC] /plc/door_state topic received (M0): {msg.data}"
        )

        self.m0_state = bool(msg.data)

        # 🔥 M0 상태가 이전과 다를 때만 DB에 한 줄 기록
        if self._last_m0_logged is None or self.m0_state != self._last_m0_logged:
            try:
                insert_ros_quality(
                    m0_state=int(self.m0_state),
                    good_count=self.good_count,
                    bad_count=self.bad_count
                )
                self.get_logger().info(
                    f"[DB] M0 changed → INSERT ros_quality_log "
                    f"(m0={int(self.m0_state)}, good={self.good_count}, bad={self.bad_count})"
                )
            except Exception as e:
                self.get_logger().error(f"[DB] insert_ros_quality 실패(M0): {e}")
            self._last_m0_logged = self.m0_state

        if not msg.data:
            return

        if not self.stm_door_client.service_is_ready():
            self.get_logger().warn("/plc/door_state SetBool service (STM) NOT ready")
            return

        req = SetBool.Request()
        req.data = True

        self.get_logger().info("[STM] call /plc/door_state SetBool service (open door)")

        # 토픽 콜백에서는 async 써도 됨 (하지만 동기 추천)
        future = self.stm_door_client.call_async(req)
        future.add_done_callback(self._on_stm_door_state_result)

    def _on_stm_door_state_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"[STM] /plc/door_state call exception: {e}")
            return

        if res is None:
            self.get_logger().error("[STM] /plc/door_state result is None")
            return

        self.get_logger().info(
            f"[STM] /plc/door_state response: success={res.success}, message='{res.message}'"
        )

        door_open = bool(res.success)
        self.pub_door_open.publish(Bool(data=door_open))
        self.get_logger().info(
            f"[AGV] /agv/door_open publish: {door_open} (from STM result)"
        )

    # ─────────────────────────────────────────────
    #  PLC(M30) start_task 토픽 → RobotArm /system/start_work Trigger 브릿지
    # ─────────────────────────────────────────────
    def cb_plc_start_task(self, msg: Bool):
        self.get_logger().info(
            f"[PLC] /plc/start_task topic received (M30): {msg.data}"
        )

        if not msg.data:
            return

        if not self.robot_start_client.service_is_ready():
            self.get_logger().warn("/system/start_work NOT ready")
            return

        req = Trigger.Request()
        self.get_logger().info("[RobotArm] call /system/start_work (Trigger)")

        future = self.robot_start_client.call_async(req)
        future.add_done_callback(self._on_start_work_result)

    def _on_start_work_result(self, future):
        try:
            res = future.result()
            self.get_logger().info(
                f"[RobotArm] /system/start_work response: success={res.success}, message='{res.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"[RobotArm] /system/start_work call exception: {e}")

    # ─────────────────────────────────────────────
# ─────────────────────────────────────────────
    #  RobotArm → AGV 서비스 브릿지
    # ─────────────────────────────────────────────
    def cb_request_dispatch(self, request, response):
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

        # [수정 4] spin_until_future_complete 대신 동기 호출(call) 사용 (데드락 방지)
        try:
            agv_result = self.agv_client.call(req)
            self.get_logger().info(
                f"[AGV] response: success={agv_result.success}, message={agv_result.message}"
            )
            response.success = agv_result.success
            response.message = agv_result.message
        except Exception as e:
            self.get_logger().error(f"AGV service call failed: {e}")
            response.success = False
            response.message = "AGV service call failed"

        return response

    # ─────────────────────────────────────────────
    #  PLC(SetBool) → ros_controller → RobotArm(Trigger)
    #  + GOOD / BAD 카운트 & DB 기록
    # ─────────────────────────────────────────────
    def cb_plc_robotarm_detect(self, request, response):
        if not self.robotarm_detect_client.service_is_ready():
            self.get_logger().warn("/robot_arm/detect NOT ready")
            response.success = False
            response.message = "RobotArm service not available"
            return response

        self.get_logger().info(
            f"[PLC] /plc/robotarm_detect called, data={request.data} → call /robot_arm/detect (Trigger)"
        )

        trigger_req = Trigger.Request()
        
        # [수정 5] 데드락 해결의 핵심! 
        # spin_until_future_complete 대신 call() 사용
        try:
            arm_res = self.robotarm_detect_client.call(trigger_req)
        except Exception as e:
            self.get_logger().error(f"RobotArm Call Failed: {e}")
            response.success = False
            response.message = "RobotArm Call Error"
            return response

        self.get_logger().info(
            f"[RobotArm] /robot_arm/detect response: success={arm_res.success}, message='{arm_res.message}'"
        )

        if not arm_res.success:
            response.success = False
            response.message = arm_res.message or "DETECT_FAILED"
            return response

        quality = (arm_res.message or "").upper()

        # [수정 6] "DEFECT"를 "BAD"로 인식하도록 추가
        if quality == "GOOD":
            self.good_count += 1
            response.success = True
            response.message = "GOOD"
        elif quality == "BAD" or quality == "DEFECT":  # <--- 여기 수정됨!
            self.bad_count += 1
            response.success = False
            response.message = "BAD"
        else:
            self.get_logger().warn(
                f"[RobotArm] unknown quality '{arm_res.message}', treat as BAD"
            )
            self.bad_count += 1
            response.success = False
            response.message = arm_res.message or "UNKNOWN"

        # 퍼블리시 + DB 기록
        self._publish_quality_counts()

        return response

    def _publish_quality_counts(self):
        """GOOD / BAD 누적 개수를 토픽으로 내보내고, DB에도 한 줄 기록."""
        msg_g = Int32()
        msg_g.data = self.good_count
        self.pub_good_count.publish(msg_g)

        msg_b = Int32()
        msg_b.data = self.bad_count
        self.pub_bad_count.publish(msg_b)

        self.get_logger().info(
            f"[COUNT] GOOD={self.good_count}, BAD={self.bad_count}"
        )

        try:
            insert_ros_quality(
                m0_state=int(self.m0_state),
                good_count=self.good_count,
                bad_count=self.bad_count
            )
            self.get_logger().info(
                f"[DB] INSERT ros_quality_log "
                f"(m0={int(self.m0_state)}, good={self.good_count}, bad={self.bad_count})"
            )
        except Exception as e:
            self.get_logger().error(f"[DB] insert_ros_quality 실패(COUNT): {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RosController()
    
    # [수정 7] Single Thread Spin 대신 MultiThreadedExecutor 사용
    # 이것이 있어야 ReentrantCallbackGroup이 효과를 발휘함
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()