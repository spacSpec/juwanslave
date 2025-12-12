import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from pymycobot.mycobot320 import MyCobot320
import threading
import time
import math

# 0) 로봇암 라이브러리 & DB 함수 import)
from db_func import insert_robot_power # 방금 만든 함수 가져오기

# ===================== [설정] =====================
PORT = '/dev/ttyACM0'
BAUD = 115200
speed = 30
high = 315
POSE_WAIT = [-91.4, 2.81, 6.85, 75.05, -89.03, 0.43]
PICK_ORIENTATION = [-180, 0, 0]
BOX_BASE_POSE = [-182.5, -29.1, 192.4, -163.96, 1.95, 85.04]
BOX_OFFSET_X = -60.0
DB_MONITOR_INTERVAL = 10 # DB 기록 주기 (10초)

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

        # [제어용] 뮤텍스(Lock) 생성: 로봇 동작의 동기화를 위해
        self.mutex = threading.Lock()
        # [모니터링용] 플래그: 쓰레드 종료 시점 조절
        self._db_monitor_running = True

        self.srv = self.create_service(ArmCommand, '/arm/execute_cmd', self.handle_command)

        self.pack_count = 0
        self.mc.set_color(0, 255, 0)

        self.init_gripper_sequence()
        
        # ===================== [통합] DB 모니터링 쓰레드 시작 =====================
        # 별도의 쓰레드를 생성하여 DB 모니터링 함수를 실행합니다.
        self.db_thread = threading.Thread(target=self.db_monitor_loop)
        self.db_thread.start()
        self.get_logger().info("📊 DB Monitoring Thread Started.")
        # =====================================================================


    def init_gripper_sequence(self):
        try:
            self.get_logger().info("🔧 Initializing Gripper...")
            self.mc.set_gripper_mode(0)
            self.mc.init_electric_gripper()
            time.sleep(1.0)
            self.get_logger().info("✅ Gripper Initialized")
        except Exception as e:
            self.get_logger().error(f"⚠️ Gripper Init Warning: {e}")

    # ===================== [통합] DB 모니터링 함수 =====================
    def read_servo_status_and_insert(self):
        """
        로봇암의 상태를 읽고 DB에 삽입하는 함수 (원본 코드 1)
        """
        try:
            # MyCobot320 객체(self.mc)를 사용하여 상태 읽기
            # 원본 코드에서 사용된 robot 객체 대신 self.mc를 사용합니다.
            temps = self.mc.get_servo_temps()
            volts = self.mc.get_servo_voltages()
            currents = self.mc.get_servo_currents()

            if temps is None or currents is None:
                 self.get_logger().warn("⚠️ Failed to read servo status from robot.")
                 return

            self.get_logger().info(f"📊 온/전/전 : {temps} / {volts} / {currents}")

            # DB로 한 줄 INSERT
            insert_robot_power(currents, temps)
            self.get_logger().info("✅ Robot Power data inserted to DB.")

        except Exception as e:
            self.get_logger().error(f"[ERROR] 로봇암 DB INSERT 실패: {e}")

    def db_monitor_loop(self):
        """
        DB 모니터링 쓰레드에서 무한 루프를 돌며 주기적으로 상태를 기록하는 함수 (원본 코드 2)
        """
        # 쓰레드가 종료 플래그를 확인하여 노드가 종료될 때 같이 종료되도록 합니다.
        while self._db_monitor_running:
            self.read_servo_status_and_insert()
            time.sleep(DB_MONITOR_INTERVAL) # 10초마다 반복
    # =====================================================================


    def wait_for_robot_stop(self, pos_tol=5.0, stable_time=0.2, timeout=10):
        # ... (기존 ArmDriverNode 코드와 동일)
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
        # ... (기존 ArmDriverNode 코드와 동일)
        with self.mutex:
            command = request.command
            target = request.target_coord 
            
            self.get_logger().info(f"Command Received: {command}")
            self.mc.set_color(0, 0, 255) 

            try:
                if command == "home":
                    # ... (홈 명령 로직)
                    self.get_logger().info("🏠 Moving to WAIT Position...")
                    self.mc.send_angles(POSE_WAIT, speed)
                    self.wait_for_robot_stop()
                    
                    self.init_gripper_sequence()
                    self.mc.set_gripper_value(100, 20, 1) 
                    self.wait_for_robot_stop()

                    self.mc.set_color(0, 255, 0)
                    response.success = True
                    response.message = "Robot Ready (WAIT)"

                elif command == "pick_good":
                    # ... (Pick 명령 로직)
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
                        
                   # 3. 하강
                    cur_coords = self.mc.get_coords()
                    if cur_coords and isinstance(cur_coords, list):
                        cur_coords[2] = high 
                        self.mc.send_coords(cur_coords, speed, 0)
                        self.wait_for_robot_stop()
                    else:
                        self.get_logger().warn("⚠️ Failed to get coords, Skipping descent.")
                        
                    # 4. 잡기
                    self.mc.set_gripper_value(40, 20, 1)
                    time.sleep(1.5)

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
                    # ... (Discard 명령 로직)
                    response.success = True
                    response.message = "Discarded"
                
                self.mc.set_color(0, 255, 0)

            except Exception as e:
                self.mc.set_color(255, 0, 0)
                response.success = False
                response.message = str(e)
                self.get_logger().error(f"Execution Error: {e}")
            
            return response

    def destroy_node(self):
        # ===================== [통합] 쓰레드 안전 종료 =====================
        # 노드가 종료될 때 DB 모니터링 쓰레드도 안전하게 종료되도록 플래그를 설정하고 대기합니다.
        self._db_monitor_running = False
        if self.db_thread.is_alive():
            self.db_thread.join(timeout=DB_MONITOR_INTERVAL + 1)
            self.get_logger().info("✅ DB Monitoring Thread stopped.")
        # =====================================================================
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'mc'):
            # 노드 소멸자(destroy_node)에서 쓰레드 정리 후
            # 여기서 ROS 2의 최종 종료를 진행합니다.
            node.mc.stop()
        # ★ 중요: destroy_node가 먼저 호출되도록 수정
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()