import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from my_robot_interfaces.msg import DetectionResult
from std_srvs.srv import Trigger, SetBool # [변경] SetBool 추가
from std_msgs.msg import Int32
import threading
import sys

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        # ---------------------------------------------------------
        # 1. 상위(Ros Controller)와의 통신 (Server 역할)
        # ---------------------------------------------------------
        self.start_srv = self.create_service(Trigger, '/system/start_work', self.handle_system_start)
        self.trigger_srv = self.create_service(Trigger, '/robot_arm/detect', self.handle_controller_trigger)

        # ---------------------------------------------------------
        # 2. 하위(Arm, Vision)와의 통신 (Client/Subscriber 역할)
        # ---------------------------------------------------------
        self.vision_sub = self.create_subscription(DetectionResult, '/vision/result', self.vision_callback, 10)
        self.arm_client = self.create_client(ArmCommand, '/arm/execute_cmd')
        
        # ---------------------------------------------------------
        # 3. AGV 통신 (SetBool 타입으로 변경)
        # ---------------------------------------------------------
        # [Request] data=True(호출), data=False(취소)
        self.agv_client = self.create_client(SetBool, '/agv/request_dispatch')
        self.count_pub = self.create_publisher(Int32, '/robot/work_cnt', 10)
        
        self.latest_vision_msg = None
        self.is_system_active = False 
        self.is_waiting_agv = False # [추가] AGV 대기 상태 플래그
        
        self.box_item_count = 0
        self.total_count = 0 
        
        self.get_logger().info('✅ Task Manager (TEST MODE) Ready. (AGV: SetBool)')
        
        # [키보드 입력 쓰레드 시작]
        self.input_thread = threading.Thread(target=self._user_input_loop, daemon=True)
        self.input_thread.start()

    # ========================== [테스트용] 사용자 입력 처리 루프 ==========================
    def _user_input_loop(self):
        """별도의 쓰레드에서 엔터 키 입력을 감지합니다."""
        print("\n" + "="*40)
        print(" [TEST MODE COMMANDS]")
        print("  - 's' + 엔터: 시스템 시작 (Start & Check Box)")
        print("  - 그냥 엔터 : PLC 신호 감지 (작업 트리거)")
        print("  - 'q' + 엔터: 종료")
        print("="*40 + "\n")

        while rclpy.ok():
            try:
                cmd = input() # 입력 대기 (블로킹)
                
                if cmd == 's':
                    # 시스템 강제 시작
                    self.get_logger().info("⌨️ User Input: SYSTEM START")
                    self.start_system_logic() # 로직 분리
                    
                elif cmd == 'q':
                    self.get_logger().info("👋 Shutting down...")
                    rclpy.shutdown()
                    sys.exit(0)
                    
                else:
                    # 그냥 엔터치면 PLC 트리거 동작 수행
                    self.get_logger().info("⌨️ User Input: PLC TRIGGER RECEIVED")
                    self._execute_task_logic(source="KEYBOARD")
                    
            except Exception as e:
                print(f"Input Error: {e}")

    # ========================== Callbacks ==========================

    def vision_callback(self, msg):
        self.latest_vision_msg = msg

    def handle_system_start(self, request, response):
        """Ros Controller가 서비스를 호출했을 때"""
        self.get_logger().info("📢 Cmd from Ros Controller: SYSTEM START")
        self.start_system_logic()
        
        response.success = True
        msg = "Ready." if not self.is_waiting_agv else "Started but Waiting for AGV."
        response.message = msg
        return response

    def handle_controller_trigger(self, request, response):
        """Ros Controller가 트리거 서비스를 호출했을 때"""
        self.get_logger().info("📢 Cmd from Ros Controller: EXECUTE TASK")
        
        result_msg = self._execute_task_logic(source="SERVICE")
        
        if result_msg in ["NOT_ACTIVE", "WAIT_VISION", "PAUSED_FOR_AGV", "NO_OBJECT"]:
            response.success = False
        else:
            response.success = True
            
        response.message = result_msg
        return response

    # ========================== [수정] 핵심 로직 구현 ==========================
    
    def start_system_logic(self):
        """시스템 시작 시 초기화 및 상태 점검"""
        # 1. 홈 이동
        self.send_arm_command("home", [0.0, 0.0, 0.0])
        self.is_system_active = True
        
        # 2. 박스 상태 확인 (재가동 시 박스가 꽉 차있을 수 있음)
        if self.box_item_count >= 1:
            self.is_waiting_agv = True
            self.get_logger().warn("⚠️ System Started, but Box is FULL! Waiting for AGV...")
        else:
            self.is_waiting_agv = False
            self.get_logger().info("✅ System Ready. Robot is Active.")

    def _execute_task_logic(self, source):
        """서비스 호출이나 키보드 입력 모두 이 로직을 수행합니다."""
        
        # 1. 시스템 활성화 확인
        if not self.is_system_active:
            self.get_logger().warn(f"⚠️ [{source}] Ignored: System is NOT ACTIVE. (Press 's' to start)")
            return "NOT_ACTIVE"

        # 2. [중요] AGV 대기 중인지 확인 (박스 교체 중 작업 불가)
        if self.is_waiting_agv:
            self.get_logger().warn(f"⏳ [{source}] Ignored: Waiting for AGV... (Box is Full)")
            return "PAUSED_FOR_AGV"

        # 3. 비전 데이터 확인
        if self.latest_vision_msg is None:
            self.get_logger().warn(f"⚠️ [{source}] Ignored: No Vision Data yet.")
            return "WAIT_VISION"

        if not self.latest_vision_msg.is_detected:
            self.get_logger().info(f"⚪ [{source}] No Object Detected.")
            return "NO_OBJECT"

        # 4. 작업 수행
        quality = self.latest_vision_msg.quality # "GOOD" or "BAD"
        
        if quality == "GOOD":
            self.get_logger().info(f"🟢 [{source}] Action: Pick Item (Good)")
            
            # [수정] Vision에서 angle도 가져옴
            t_x = self.latest_vision_msg.center[0]
            t_y = self.latest_vision_msg.center[1]
            t_angle = self.latest_vision_msg.angle
            
            # [x, y, angle] 전송
            self.send_arm_command("pick_good", [t_x, t_y, t_angle])
        else:
            self.get_logger().info(f"🔴 [{source}] Action: Discard Item (Bad)")
            self.send_arm_command("discard_bad", [0.0, 0.0, 0.0])
            
        return quality

    # ========================== Helper Methods ==========================

    def send_arm_command(self, cmd, coord):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("❌ Arm service not available!")
            return

        req = ArmCommand.Request()
        req.command = cmd
        req.target_coord = coord
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_done_callback)

    # def arm_done_callback(self, future):
    #     try:
    #         result = future.result()
    #         if result.success:
    #             self.get_logger().info(f"✅ Arm Finished: {result.message}")
                
    #             if "Pick" in result.message:
    #                 self.box_item_count += 1
    #                 self.total_count += 1
                    
    #                 msg = Int32()
    #                 msg.data = self.total_count
    #                 self.count_pub.publish(msg)
    #                 self.get_logger().info(f"📦 Box Count: {self.box_item_count}/3")

    #                 # 박스 만재 시 AGV 호출
    #                 if self.box_item_count >= 3:
    #                     self.get_logger().warn("🛑 Box Full! Pausing & Calling AGV...")
    #                     self.is_waiting_agv = True # 로봇 일시정지 (안전)
    #                     self.control_agv(enable=True)
                        
    #     except Exception as e:
    #         self.get_logger().error(f"❌ Arm Failed: {e}")
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
            if self.box_item_count >= 2:
                self.get_logger().warn("🛑 Box Full! Pausing Robot & Calling AGV...")
                self.is_waiting_agv = True  
                self.control_agv(enable=True) # AGV 호출
                    
        except Exception as e:
            self.get_logger().error(f"❌ Callback Error: {e}")

    def control_agv(self, enable: bool):
        """AGV 제어 (SetBool)"""
        if not self.agv_client.wait_for_service(1.0):
            self.get_logger().error("❌ AGV Service Unavailable")
            return

        req = SetBool.Request()
        req.data = enable
        
        action_str = "CALL" if enable else "CANCEL"
        self.get_logger().info(f"🚚 Sending AGV Command: {action_str}...")
        
        future = self.agv_client.call_async(req)
        future.add_done_callback(lambda f: self.agv_done_callback(f, action_str))

    def agv_done_callback(self, future, action_str):
        try:
            res = future.result()
            
            # AGV가 "성공(True)" 했다고 응답했을 때만 로봇 작업 재개
            if res.success and action_str == "CALL":
                self.get_logger().info(f"✅ AGV Arrived/Departed! Resuming Work...")
                self.box_item_count = 0     # 카운트 리셋
                self.is_waiting_agv = False # 작업 재개 허용
                
            elif not res.success:
                self.get_logger().error(f"⚠️ AGV Failed: {res.message}. Robot still paused.")
                
        except Exception as e:
            self.get_logger().error(f"❌ AGV Service Call Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()