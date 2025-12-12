import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from pymycobot.mycobot320 import MyCobot320 
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

        self.srv = self.create_service(ArmCommand, '/arm/execute_cmd', self.handle_command)
        
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

    def handle_command(self, request, response):
        # [수정 3] 뮤텍스(Lock) 걸기 - 동기화의 핵심
        # 이 블록(with) 안에 들어오면 '열쇠'를 잠급니다.
        # 앞선 명령이 끝나서 이 블록을 나가기 전까지, 뒷 명령은 여기서 멈춰 기다립니다.
        with self.mutex:
            command = request.command
            target = request.target_coord 
            
            self.get_logger().info(f"Command Received: {command}") # 로그 수정
            self.mc.set_color(0, 0, 255) 

            try:
                if command == "home":
                    self.get_logger().info("🏠 Moving to WAIT Position...")
                    self.mc.send_angles(POSE_WAIT, speed)
                    self.wait_for_robot_stop()
                    
                    self.init_gripper_sequence()
                    self.mc.set_gripper_value(100, 20, 1) 
                    self.wait_for_robot_stop()

                    self.mc.set_color(0, 255, 0)
                    response.success = True
                    response.message = "Robot Ready (WAIT)"
                    # return response (여기서 리턴하지 않고 맨 아래에서 통합 리턴)

                elif command == "pick_good":
                    x, y, angle = target[0], target[1], target[2]
                    self.get_logger().info(f"🚀 Processing Pick: {x}, {y}")

                    # 1. 이동
                    target_pose = [x, y, high+30, *PICK_ORIENTATION] 
                    self.mc.send_coords(target_pose, speed, 0)
                    self.wait_for_robot_stop()

                    # 2. 회전 보정
                    current_angles = self.mc.get_angles()
                    if current_angles:
                        target_j6 = angle - 90
                        while target_j6 > 90: target_j6 -= 90
                        while target_j6 < -90: target_j6 += 90
                        if target_j6 > 170: target_j6 = 170
                        if target_j6 < -170: target_j6 = -170
                        
                        current_angles[5] = target_j6
                        self.mc.send_angles(current_angles, speed)
                        self.wait_for_robot_stop()
                        
                   # 3. 하강 [수정됨]
                    cur_coords = self.mc.get_coords()
                    # ★ 중요: 여기도 리스트인지 확인!
                    if cur_coords and isinstance(cur_coords, list):
                        cur_coords[2] = high 
                        self.mc.send_coords(cur_coords, speed, 0)
                        self.wait_for_robot_stop()
                    else:
                        self.get_logger().warn("⚠️ Failed to get coords, Skipping descent.")
                        
                    # 4. 잡기
                    self.mc.set_gripper_value(40, 20, 1)
                    time.sleep(1.5) # 그리퍼 동작은 좌표 변화가 없으니 시간으로 대기

                    # 5. 상승
                    cur_coords = self.mc.get_coords()
                    if cur_coords:
                        cur_coords[2] = high+30
                        self.mc.send_coords(cur_coords, speed, 0)
                        self.wait_for_robot_stop()

                    # 6. 적재
                    final_place_pose = list(BOX_BASE_POSE) 
                    final_place_pose[0] = BOX_BASE_POSE[0] + (self.pack_count * BOX_OFFSET_X)
                    
                    up_pose = list(final_place_pose)
                    up_pose[2] += 100 
                    self.mc.send_coords(up_pose, speed, 0)
                    self.wait_for_robot_stop()
                    
                    self.mc.send_coords(final_place_pose, speed, 0)
                    self.wait_for_robot_stop()
                    
                    self.mc.set_gripper_value(100, 20, 1) 
                    time.sleep(1.0)
                    
                    self.mc.send_coords(up_pose, speed, 0)
                    self.wait_for_robot_stop()

                    self.pack_count += 1
                    if self.pack_count >= 3:
                        self.pack_count = 0 

                    # 홈 복귀
                    self.mc.send_angles(POSE_WAIT, speed)
                    self.wait_for_robot_stop()

                    response.success = True
                    response.message = f"Done (Count: {self.pack_count})"

                elif command == "discard_bad":
                    # [추가] 불량 처리 로직이 비어있으면 너무 빨리 끝나서 문제될 수 있음
                    # 필요하다면 여기에 동작 추가
                    response.success = True
                    response.message = "Discarded"
                
                self.mc.set_color(0, 255, 0)

            except Exception as e:
                self.mc.set_color(255, 0, 0)
                response.success = False
                response.message = str(e)
                self.get_logger().error(f"Execution Error: {e}")
            
            # [수정 4] Lock 범위 안에서 리턴
            # 여기까지 와야 with self.mutex 블록이 끝나고 열쇠가 반납됩니다.
            # 그래야 대기하던 다음 명령이 들어올 수 있습니다.
            return response

            
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