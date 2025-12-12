import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from my_robot_interfaces.msg import DetectionResult
from std_srvs.srv import Trigger, SetBool  # SetBool 추가
from std_msgs.msg import Int32

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        # ---------------------------------------------------------
        # 1. 상위(Ros Controller)와의 통신
        # ---------------------------------------------------------
        self.start_srv = self.create_service(Trigger, '/system/start_work', self.handle_system_start)
        self.trigger_srv = self.create_service(Trigger, '/robot_arm/detect', self.handle_controller_trigger)

        # ---------------------------------------------------------
        # 2. 하위(Arm, Vision)와의 통신
        # ---------------------------------------------------------
        self.vision_sub = self.create_subscription(DetectionResult, '/vision/result', self.vision_callback, 10)
        self.arm_client = self.create_client(ArmCommand, '/arm/execute_cmd')
        
        # ---------------------------------------------------------
        # 3. AGV 통신 (SetBool 타입으로 변경)
        # ---------------------------------------------------------
        # [Request] data=True(호출), data=False(취소)
        # [Response] success, message
        self.agv_client = self.create_client(SetBool, '/agv/request_dispatch')
        
        self.count_pub = self.create_publisher(Int32, '/robot/work_cnt', 10)
        
        self.latest_vision_msg = None
        self.is_system_active = False
        self.box_item_count = 0
        self.total_count = 0
        self.is_waiting_agv = False

        self.get_logger().info('✅ Task Manager Ready. (AGV: SetBool Type)')

    # ========================== Callbacks ==========================

    def vision_callback(self, msg):
        self.latest_vision_msg = msg

    def handle_system_start(self, request, response):
        """시스템 시작 시 초기화 (안전 기능 강화!)"""
        self.get_logger().info("📢 System Start Command Received")
        
        # 1. 로봇 팔 홈으로 이동 (이건 해야지)
        self.send_arm_command("home", [0.0, 0.0, 0.0])
        
        self.is_system_active = True
        
        # 2. [수정됨] 무조건 초기화가 아니라, '박스 상태'를 보고 결정!
        # "만약 박스가 이미 꽉 차 있는 상태라면?"
        if self.box_item_count >= 3:
            self.is_waiting_agv = True   # 🔴 계속 기다려! (안전장치)
            self.get_logger().warn("⚠️ System Started, but Box is FULL! Waiting for AGV...")
            
            # (선택) 만약 AGV 요청이 끊겼을까봐 걱정되면 여기서 AGV를 한번 더 불러도 됨
            # self.control_agv(enable=True) 
            
        else:
            self.is_waiting_agv = False  # 🟢 박스 자리 남았으니 일 해도 됨!
            self.get_logger().info("✅ System Ready. Robot is Active.")
        
        response.success = True
        response.message = f"Started. (Box Count: {self.box_item_count})"
        return response

    def handle_controller_trigger(self, request, response):
        """PLC 센서 감지 신호 처리"""
        if not self.is_system_active:
            response.success = False
            response.message = "SYSTEM_NOT_ACTIVE"
            return response
        
        if self.is_waiting_agv:
            response.success = False
            response.message = "PAUSED_FOR_AGV" # "지금 AGV 기다리는 중이라 못해요"
            self.get_logger().info("⏳ Work Skipped: Waiting for AGV...")
            return response

        if self.latest_vision_msg is None:
            response.success = False
            response.message = "NO_VISION_DATA"
            return response

        # Vision 감지 결과 확인
        if not self.latest_vision_msg.is_detected:
            response.success = False
            response.message = "NO_OBJECT"
            return response

        # 1. Vision 정보 파싱
        quality = self.latest_vision_msg.quality
        
        # 2. Ros Controller 보고 (DB 저장용)
        response.success = True
        response.message = quality
        
        # 3. 로봇 팔 명령 전송
        
        if quality == "GOOD":
            self.get_logger().info(f"🟢 Action: Pick Item ({quality})")
            
            # [수정됨] 인터페이스가 수정되었으므로 Vision이 준 angle을 그대로 사용
            t_x = self.latest_vision_msg.center[0]
            t_y = self.latest_vision_msg.center[1]
            t_angle = self.latest_vision_msg.angle  # 이제 Vision이 각도를 줌
            
            self.send_arm_command("pick_good", [t_x, t_y, t_angle])
        else:
            self.get_logger().info(f"🔴 Action: Discard Item ({quality})")
            self.send_arm_command("discard_bad", [0.0, 0.0, 0.0])
            
        return response

    # ========================== Helper Methods ==========================

    def send_arm_command(self, cmd, coord):
        if not self.arm_client.wait_for_service(1.0):
            self.get_logger().error("❌ Arm Service Unavailable")
            return

        req = ArmCommand.Request()
        req.command = cmd
        req.target_coord = coord
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_done_callback)

    # def arm_done_callback(self, future):
    #     try:
    #         result = future.result()
    #         if result.success and "Pick" in result.message:
    #             self.box_item_count += 1
    #             self.total_count += 1
                
    #             # 카운트 발행
    #             msg = Int32()
    #             msg.data = self.total_count
    #             self.count_pub.publish(msg)
                
    #             self.get_logger().info(f"📦 Count: {self.box_item_count}/3")

    #             # 박스 만재 시 AGV 호출
    #             if self.box_item_count >= 3:
    #                 self.get_logger().warn("🛑 Box Full! Pausing Robot & Calling AGV...")
    #                 self.is_waiting_agv = True  # 빨간불 켜기! (이제부터 작업 요청 거부)
    #                 self.control_agv(enable=True) # AGV 호출
                    
    #     except Exception as e:
    #         self.get_logger().error(f"❌ Arm Error: {e}")
    def arm_done_callback(self, future):
        try:
            result = future.result()
            
            # -----------------------------------------------------------
            # [수정됨] 강제 성공 처리 모드
            # 원래는 if result.success and "Pick" in result.message: 해야 하지만
            # 지금은 실패해도(좌표 에러 등) 그냥 적재했다고 가정합니다.
            # -----------------------------------------------------------
            
            self.get_logger().info(f"🤖 Arm Status: {result.success}, Msg: {result.message}")
            
            # [중요] 로봇이 실패했어도(False), 그냥 성공한 척 로그 찍고 카운트 올림
            if not result.success:
                self.get_logger().warn("⚠️ Arm Failed (Coordinate Error?), but FORCING SUCCESS for simulation!")

            # 무조건 카운트 증가
            self.box_item_count += 1
            self.total_count += 1
            
            # 카운트 발행
            msg = Int32()
            msg.data = self.total_count
            self.count_pub.publish(msg)
            
            self.get_logger().info(f"📦 [SIMULATION] Box Count: {self.box_item_count}/3")

            # 박스 만재 시 AGV 호출 (3개 다 차면)
            if self.box_item_count >= 3:
                self.get_logger().warn("🛑 Box Full! Pausing Robot & Calling AGV...")
                self.is_waiting_agv = True  
                self.control_agv(enable=True) # AGV 호출
                    
        except Exception as e:
            self.get_logger().error(f"❌ Callback Error: {e}")

    def control_agv(self, enable: bool):
        """
        AGV 제어 함수 (SetBool 타입)
        enable=True  : 배차 요청 (Dispatch)
        enable=False : 배차 취소/대기 (Cancel/Wait)
        """
        if not self.agv_client.wait_for_service(1.0):
            self.get_logger().error("❌ AGV Service Unavailable")
            return

        req = SetBool.Request()
        req.data = enable  # True or False
        
        action_str = "CALL" if enable else "CANCEL"
        self.get_logger().info(f"🚚 Sending AGV Command: {action_str}...")
        
        future = self.agv_client.call_async(req)
        future.add_done_callback(lambda f: self.agv_done_callback(f, action_str))

    def agv_done_callback(self, future, action_str):
        try:
            res = future.result()
            
            # 🚦 [4. 재시작 신호]
            # AGV가 "성공(True)" 했다고 응답했을 때만 다시 가동
            if res.success and action_str == "CALL":
                self.get_logger().info(f"✅ AGV Arrived/Departed! Resuming Work...")
                
                self.box_item_count = 0     # 카운트 리셋
                self.is_waiting_agv = False # 초록불 켜기! (이제 다시 작업 가능)
                
            elif not res.success:
                self.get_logger().error(f"⚠️ AGV Failed: {res.message}. Robot still paused.")
                # 실패하면 is_waiting_agv를 False로 안 바꿉니다. 
                # 그래야 사람이 와서 해결할 때까지 로봇이 계속 멈춰있으니까요. (안전)
        except Exception as e:
            self.get_logger().error(f"❌ AGV Service Call Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()