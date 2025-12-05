# ~/final_ws/src/final/final/task_manager_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from my_robot_interfaces.msg import DetectionResult
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        # ---------------------------------------------------------
        # 1. 상위(Ros Controller)와의 통신 (Server 역할)
        # ---------------------------------------------------------
        # 시스템 시작 명령 대기
        self.start_srv = self.create_service(Trigger, '/system/start_work', self.handle_system_start)
        # 작업 수행 명령 대기 (PLC 신호를 Ros Controller가 중계)
        self.trigger_srv = self.create_service(Trigger, '/robot_arm/detect', self.handle_controller_trigger)

        # ---------------------------------------------------------
        # 2. 하위(Arm, Vision)와의 통신 (Client/Subscriber 역할)
        # ---------------------------------------------------------
        # Vision 데이터 계속 받기
        self.vision_sub = self.create_subscription(DetectionResult, '/vision/result', self.vision_callback, 10)
        # Arm에게 명령 내리기
        self.arm_client = self.create_client(ArmCommand, '/arm/execute_cmd')
        
        # ---------------------------------------------------------
        # 3. 기타 (AGV 호출 및 상태 보고)
        # ---------------------------------------------------------
        # 박스 꽉 차면 Ros Controller에게 AGV 요청 (Service Client)
        self.agv_client = self.create_client(Trigger, '/agv/request_dispatch')
        # 생산량 카운트 (대시보드 표시용) - DB 저장은 Ros Controller가 함
        self.count_pub = self.create_publisher(Int32, '/robot/work_cnt', 10)
        
        self.latest_vision_msg = None
        self.is_system_active = False
        self.box_item_count = 0
        self.total_count = 0 # 로컬 표시용
        
        self.get_logger().info('✅ Task Manager (Robot Subsystem) Ready. Waiting for ROS Controller...')

    # ========================== Callbacks ==========================

    def vision_callback(self, msg):
        self.latest_vision_msg = msg

    def handle_system_start(self, request, response):
        

        """Ros Controller가 '공장 가동' 신호를 보내면 실행"""
        self.get_logger().info("📢 Cmd from Ros Controller: SYSTEM START")
        
        # 로봇 팔에게 홈 이동 명령
        self.send_arm_command("home", [0.0, 0.0, 0.0])
        
        self.is_system_active = True
        response.success = True
        response.message = "Robot System Started & Homing"
        return response

    def handle_controller_trigger(self, request, response):
        """Ros Controller가 'PLC 센서 감지됨, 작업해'라고 하면 실행"""
        
        if not self.is_system_active:
            response.success = False
            response.message = "NOT_ACTIVE"
            return response

        self.get_logger().info("📢 Cmd from Ros Controller: EXECUTE TASK")
        
        if self.latest_vision_msg is None:
            response.success = False
            response.message = "WAIT_VISION"
            return response

# [수정됨] 물체가 감지되었는지 확인하는 로직 추가!
        if not self.latest_vision_msg.is_detected:
            self.get_logger().warn(f"⚠️ [{source}] Vision says: NOTHING DETECTED.")
            return "NO_OBJECT"

        # 1. Vision 결과 확인
        quality = self.latest_vision_msg.quality # "GOOD" or "BAD"
        
        # 2. Ros Controller에게 즉시 결과 보고 (그래야 걔가 DB에 넣음)
        response.success = True
        response.message = quality
        
        # 3. 로봇 팔 작업 지시 (비동기 수행)
        if quality == "GOOD":
            self.get_logger().info("🟢 Action: Pick Item")
            self.send_arm_command("pick_good", self.latest_vision_msg.center)
        else:
            self.get_logger().info("🔴 Action: Discard Item")
            self.send_arm_command("discard_bad", [0.0, 0.0, 0.0])
            
        return response

    # ========================== Helper Methods ==========================

    def send_arm_command(self, cmd, coord):
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
                
                # 로봇 팔 작업이 '적재(Pick)'였으면 박스 카운트 관리
                if "Pick" in result.message:
                    self.box_item_count += 1
                    self.total_count += 1
                    
                    # 대시보드용 단순 카운트 전송
                    msg = Int32()
                    msg.data = self.total_count
                    self.count_pub.publish(msg)

                    # 박스 꽉 차면 AGV 호출 요청 (to Ros Controller)
                    if self.box_item_count >= 3:
                        self.call_agv_dispatch()
                        self.box_item_count = 0
        except Exception as e:
            self.get_logger().error(f"❌ Arm Failed: {e}")

    def call_agv_dispatch(self):
        self.get_logger().info("🚚 Requesting AGV Dispatch to Ros Controller...")
        req = Trigger.Request()
        # Ros Controller의 AGV 호출 서비스를 찌름
        self.agv_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()