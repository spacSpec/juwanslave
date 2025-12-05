import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from my_robot_interfaces.msg import DetectionResult
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
import threading  # [추가] 키보드 입력을 위해 필요
import sys        # [추가] 종료 처리를 위해 필요

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
        # 3. 기타 (AGV 호출 및 상태 보고)
        # ---------------------------------------------------------
        self.agv_client = self.create_client(Trigger, '/agv/request_dispatch')
        self.count_pub = self.create_publisher(Int32, '/robot/work_cnt', 10)
        
        self.latest_vision_msg = None
        self.is_system_active = False 
        self.box_item_count = 0
        self.total_count = 0 
        
        self.get_logger().info('✅ Task Manager Ready.')
        
        # [추가] 키보드 입력을 기다리는 쓰레드 시작
        self.input_thread = threading.Thread(target=self._user_input_loop, daemon=True)
        self.input_thread.start()

    # ========================== [추가] 사용자 입력 처리 루프 ==========================
    def _user_input_loop(self):
        """별도의 쓰레드에서 엔터 키 입력을 감지합니다."""
        print("\n" + "="*40)
        print(" [TEST MODE COMMANDS]")
        print("  - 's' + 엔터: 시스템 시작 (가동)")
        print("  - 그냥 엔터 : PLC 신호 감지 (작업 시작)")
        print("  - 'q' + 엔터: 종료")
        print("="*40 + "\n")

        while rclpy.ok():
            try:
                cmd = input() # 여기서 입력 대기 (블로킹)
                
                if cmd == 's':
                    # 시스템 강제 시작 (Start 명령 흉내)
                    self.get_logger().info("⌨️ User Input: SYSTEM START")
                    self.send_arm_command("home", [0.0, 0.0, 0.0])
                    self.is_system_active = True
                    
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
        """Ros Controller가 '공장 가동' 신호를 보내면 실행"""
        self.get_logger().info("📢 Cmd from Ros Controller: SYSTEM START")
        self.send_arm_command("home", [0.0, 0.0, 0.0])
        self.is_system_active = True
        response.success = True
        response.message = "Robot System Started & Homing"
        return response

    def handle_controller_trigger(self, request, response):
        """Ros Controller가 서비스로 호출했을 때 실행"""
        self.get_logger().info("📢 Cmd from Ros Controller: EXECUTE TASK")
        
        # 실제 로직은 _execute_task_logic 함수에서 처리하고 결과만 받아옴
        result_msg = self._execute_task_logic(source="SERVICE")
        
        if result_msg in ["NOT_ACTIVE", "WAIT_VISION"]:
            response.success = False
        else:
            response.success = True
            
        response.message = result_msg
        return response

    # ========================== [수정] 핵심 로직 분리 ==========================
    def _execute_task_logic(self, source):
        """서비스 호출이나 키보드 입력 모두 이 로직을 수행합니다."""
        
        if not self.is_system_active:
            self.get_logger().warn(f"⚠️ [{source}] Ignored: System is NOT ACTIVE. (Press 's' to start)")
            return "NOT_ACTIVE"

        if self.latest_vision_msg is None:
            self.get_logger().warn(f"⚠️ [{source}] Ignored: No Vision Data yet.")
            return "WAIT_VISION"

        # 1. Vision 결과 확인
        quality = self.latest_vision_msg.quality # "GOOD" or "BAD"
        
        # 2. 로봇 팔 작업 지시
        if quality == "GOOD":
            self.get_logger().info(f"🟢 [{source}] Action: Pick Item (Good)")
            self.send_arm_command("pick_good", self.latest_vision_msg.center)
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

    def arm_done_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"✅ Arm Finished: {result.message}")
                
                if "Pick" in result.message:
                    self.box_item_count += 1
                    self.total_count += 1
                    
                    msg = Int32()
                    msg.data = self.total_count
                    self.count_pub.publish(msg)

                    if self.box_item_count >= 3:
                        self.call_agv_dispatch()
                        self.box_item_count = 0
        except Exception as e:
            self.get_logger().error(f"❌ Arm Failed: {e}")

    def call_agv_dispatch(self):
        self.get_logger().info("🚚 Requesting AGV Dispatch...")
        req = Trigger.Request()
        self.agv_client.call_async(req)

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