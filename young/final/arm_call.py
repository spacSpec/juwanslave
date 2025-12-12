import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from pymycobot.mycobot320 import MyCobot320 
from std_srvs.srv import Trigger
import threading # [수정] 락(Lock) 기능을 위해 필요
import time
import math

# ===================== [설정] =====================
PORT = '/dev/ttyACM0'
BAUD = 115200
speed = 30 
high = 315
POSE_WAIT =[-91.4, 2.81, 6.85, 75.05, -89.03, 0.43]
PICK_ORIENTATION = [-180, 0, 0] 
BOX_BASE_POSE =  [-182.5, -29.1, 192.4, -163.96, 1.95, 85.04]
BOX_OFFSET_X = -60.0 

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')
        try:
            self.mc = MyCobot320(PORT, BAUD)
            time.sleep(0.5)
            self.mc.power_on()
            time.sleep(0.5)
            self.get_logger().info(f'✅ MyCobot320 Connected on {PORT}')
        except Exception as e:
            self.get_logger().error(f'❌ Connection Failed: {e}')
            return

        # [수정 1] 뮤텍스(Lock) 생성
        # 화장실 열쇠를 하나 만듭니다. 누군가 이 열쇠를 가지고 작업 중이면
        # 다른 명령은 밖에서 줄 서서 기다려야 합니다.
        self.mutex = threading.Lock()

        # 서비스 콜백 함수 이름을 handle_controller_trigger로 변경합니다.
        self.srv = self.create_service(ArmCommand, '/arm/execute_cmd', self.handle_controller_trigger)
        self.vision_client = self.create_client(Trigger, '/vision/start_detection')
        
        self.pack_count = 0
        self.mc.set_color(0, 255, 0)
        
        self.init_gripper_sequence()

    def init_gripper_sequence(self):
        try:
            self.get_logger().info("🔧 Initializing Gripper...")
            self.mc.set_gripper_mode(0)
            self.mc.init_electric_gripper()
            time.sleep(1.0)
            self.get_logger().info("✅ Gripper Initialized")
        except Exception as e:
            self.get_logger().error(f"⚠️ Gripper Init Warning: {e}")

    def wait_for_robot_stop(self, pos_tol=5.0, stable_time=0.2, timeout=10):
        """
        로봇이 목표 지점에 도달할 때까지 대기
        """
        # [수정 2] 출발 지연 대기 (매우 중요!)
        # 명령을 보내자마자 이 함수가 호출되면, 로봇이 아직 움직이기 시작도 안 해서
        # diff가 0이 되어 "도착했다"고 착각하고 함수가 바로 끝나버립니다.
        # 로봇이 반응하고 움직일 시간을 0.2초 정도 줍니다.
        time.sleep(0.2) 

        start = time.time()
        still_since = None
        last = self.mc.get_coords()
        
        while time.time() - start < timeout:
            time.sleep(0.1) 
            now = self.mc.get_coords()
            if not now or not last: continue
            
            diff = max(abs(n - l) for n, l in zip(now, last))
            
            if diff < pos_tol:
                if still_since is None: still_since = time.time()
                elif time.time() - still_since > stable_time:
                    return 
            else:
                still_since = None 
            last = now 
            
        self.get_logger().warn("⚠️ Motion Timeout") 

    def handle_controller_trigger(self, request, response):
        """
        [ROS Controller]로부터 /robot_arm/detect 서비스를 받았을 때 실행됩니다.
        이것이 곧 컨베이어 멈춤 신호입니다.
        """
        self.get_logger().info("📢 Cmd from Ros Controller: EXECUTE TASK (Detect & Pick)")
        
        # 1. Vision Node에 분석 요청 (Service 호출)
        vision_result_response = self.call_vision_service_sync()
        
        if not vision_result_response.success:
            # Vision Node가 분석에 실패하거나 물체를 못 찾았을 때
            response.success = False
            response.message = vision_result_response.message # Vision 노드의 실패 메시지 전달
            return response

        # 2. Vision Service 응답으로부터 품질(Good/Defect/No_Object)을 추출
        quality = vision_result_response.message # Vision 응답 메시지 = 품질

        # 3. Vision Node가 발행한 /vision/result 토픽에서 최신 좌표를 가져와야 함 (비동기)
        # Vision Node는 Service 응답 전에 /vision/result를 발행해야 합니다.
        
        if self.latest_vision_msg is None:
             self.get_logger().error("❌ Vision Service 호출 성공, but No /vision/result Topic Received!")
             response.success = False
             response.message = "VISION_RESULT_MISSING"
             return response
             
        # 4. 작업 수행 (Arm Command 전송)
        result_msg = self._execute_task_logic_from_vision(quality)
        
        if result_msg in ["NOT_ACTIVE", "PAUSED_FOR_AGV", "NO_OBJECT"]:
            response.success = False
        else:
            response.success = True
            
        response.message = result_msg # 최종 품질 메시지 반환
        return response

    # 새로운 헬퍼 함수: Arm Command 전송 (latest_vision_msg 사용)
    def _execute_task_logic_from_vision(self, quality):
        # 1. 시스템 활성화/AGV 대기 확인 (기존 로직 그대로 사용)
        if not self.is_system_active: return "NOT_ACTIVE"
        if self.is_waiting_agv: return "PAUSED_FOR_AGV"

        if quality == "GOOD":
            self.get_logger().info(f"🟢 Action: Pick Item (Good)")
            
            t_x = self.latest_vision_msg.center[0]
            t_y = self.latest_vision_msg.center[1]
            t_angle = self.latest_vision_msg.angle
            
            self.send_arm_command("pick_good", [t_x, t_y, t_angle])
            return "GOOD"
            
        elif quality == "DEFECT":
            self.get_logger().info(f"🔴 Action: Discard Item (Defect)")
            self.send_arm_command("discard_bad", [0.0, 0.0, 0.0])
            return "DEFECT"
        
        else: # quality == "NO_OBJECT"
            self.get_logger().info(f"⚪ No Object Detected by Vision.")
            return "NO_OBJECT"

    # [새로 추가] Vision Service 호출 함수 (동기적)
    def call_vision_service_sync(self):
        """Vision 노드에 분석 요청을 동기적으로 보냅니다."""
        if not self.vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("❌ Vision service not available!")
            return Trigger.Response(success=False, message="SERVICE_UNAVAILABLE")

        req = Trigger.Request()
        future = self.vision_client.call_async(req)
        # 서비스 응답을 받을 때까지 블로킹(대기)
        rclpy.spin_until_future_complete(self, future) 
        
        return future.result()
            
def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'mc'):
            node.mc.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()