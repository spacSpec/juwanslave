#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from std_msgs.msg import String  # [추가] JSON 메시지 전송용
from pymycobot.mycobot320 import MyCobot320 
import threading 
import time
import math
import json  # [추가]

# ===================== [설정] =====================
PORT = '/dev/ttyACM0'
BAUD = 115200
speed = 20 
high = 275
POSE_WAIT = [-108.19, -37.17, 80.15, 40.81, -89.2, -19.07]
PICK_ORIENTATION_RX = -170
PICK_ORIENTATION_RY = -1.88
BOX_BASE_POSE = [-255.5, 43.4, 120.7, 177.43, 3.33, -0.89]
BOX_OFFSET_Y = -60.0 
COLLISION_THRESHOLD = 2500 

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

        self.mutex = threading.Lock()
        self.srv = self.create_service(ArmCommand, '/arm/execute_cmd', self.handle_command)
        
        # [추가] 상태 모니터링 퍼블리셔
        self.status_pub = self.create_publisher(String, '/robot/status_json', 10)
        self.last_pub_time = 0.0  # 1초 주기 체크용
        
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

    # [추가] 로봇 건강 상태 계산 로직
    def calculate_health(self, volts, temps):
        p_lvl, m_lvl = "GOOD", "GOOD"
        msg = "Normal"
        
        # 전압 체크
        min_v = min(volts) if volts else 0
        if min_v < 19.0: p_lvl = "CRITICAL"; msg = "Voltage Collapse"
        elif min_v < 22.0: p_lvl = "WARNING"; msg = "Voltage Sag"

        # 온도 체크
        max_t = max(temps) if temps else 0
        if max_t > 75: m_lvl = "CRITICAL"; msg = "Overheat"
        elif max_t > 60: m_lvl = "WARNING"; msg = "High Temp"

        return p_lvl, m_lvl, msg

    # [추가] 상태 퍼블리시 함수 (핵심: 데이터 재활용 & 1Hz 제한)
    def publish_robot_status(self, current_data=None):
        # 1. 시간 체크 (1초가 안 지났으면 스킵 -> 통신 병목 방지)
        now = time.time()
        if now - self.last_pub_time < 1.0:
            return

        try:
            # 2. 데이터 수집 (없는 경우에만 읽음)
            # 전류값은 wait_until_arrival에서 줬으면 그거 쓰고, 안 줬으면 읽음
            currents = current_data if current_data else self.mc.get_servo_currents()
            
            # 전압, 온도는 자주 안 변하니까 1초에 한 번 여기서만 읽음
            volts = self.mc.get_servo_voltages()
            temps = self.mc.get_servo_temps()

            # 3. 건강 상태 계산
            p_lvl, m_lvl, h_msg = self.calculate_health(volts, temps)

            # 4. JSON 생성
            status_data = {
                "timestamp": now,
                "volts": volts,
                "temps": temps,
                "currents": currents,
                "power_status": p_lvl,
                "motor_status": m_lvl,
                "message": h_msg
            }

            # 5. 전송 (DB 노드가 받아서 저장함)
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_pub.publish(msg)
            
            self.last_pub_time = now # 시간 갱신

        except Exception as e:
            self.get_logger().warn(f"Status Pub Error: {e}")

    # 기존 함수 수정 (로그 기능은 삭제하고 위 함수로 대체 가능하지만, 일단 유지)
    def log_motor_status(self):
        # (기존 코드 유지)
        pass

    def wait_until_arrival(self, target, mode='coords', tol=5.0, timeout=15):
        start = time.time()
        last_values = None
        stop_count = 0
        
        time.sleep(0.2) 

        while time.time() - start < timeout:
            # 1. 충돌 감지 및 데이터 읽기
            try:
                currents = self.mc.get_servo_currents()
                
                # [★ 핵심 수정] 읽은 전류값을 바로 상태 전송 함수에 넘겨줌 (재활용)
                # 함수 내부에서 1초가 지났는지 체크하고 보낼지 말지 결정함
                self.publish_robot_status(current_data=currents)

                if currents and len(currents) == 6:
                    if max(currents) > COLLISION_THRESHOLD:
                        self.mc.stop()
                        raise RuntimeError(f"🚨 충돌 감지! (Current: {max(currents)}mA)")
            except RuntimeError as re:
                raise re
            except Exception:
                pass 

            # 2. 현재값 읽기
            try:
                if mode == 'coords':
                    current = self.mc.get_coords()
                else:
                    current = self.mc.get_angles()
            except Exception:
                continue

            if not current or len(current) != 6:
                continue

            # 3. 도착 확인
            diff = max([abs(c - t) for c, t in zip(current, target)])
            if diff < tol:
                return True 
            
            # 4. 정지 확인
            if last_values:
                move_diff = max([abs(c - l) for c, l in zip(current, last_values)])
                if move_diff < 1.0: 
                    stop_count += 1
                else:
                    stop_count = 0
            
            if stop_count > 5:
                self.get_logger().warn(f"⚠️ 목표 오차({diff:.1f})가 있지만 정지하여 진행함.")
                return True

            last_values = current
            time.sleep(0.1)
        
        err_msg = f"⏳ [TIMEOUT] 목표 도달 실패! (Mode: {mode}, Final Diff: {diff:.1f})"
        self.get_logger().fatal(err_msg)
        raise TimeoutError(err_msg)
    
    def handle_command(self, request, response):
        with self.mutex:
            # [추가] 동작 시작 전에도 상태 한번 체크해서 보냄
            self.publish_robot_status()

            # 출발 전 전압 검사 (기존 로직 + 로그 강화)
            try:
                volts = self.mc.get_servo_voltages()
                if volts and any(v < 20.0 for v in volts):
                    msg = f"❌ 전압 부족! (Current Volts: {volts})"
                    self.get_logger().fatal(msg)
                    response.success = False
                    response.message = msg
                    return response
            except Exception:
                pass 
            
            command = request.command
            target = request.target_coord 
            
            self.get_logger().info(f"Command Received: {command}")
            self.mc.set_color(0, 0, 255) 

            try:
                if command == "home":
                    self.get_logger().info("🏠 Moving to WAIT Position...")
                    self.mc.send_angles(POSE_WAIT, speed)
                    self.wait_until_arrival(POSE_WAIT, mode='angles', tol=3.0)
                    
                    self.init_gripper_sequence()
                    self.mc.set_gripper_value(100, 20, 1) 
                    time.sleep(1.0) 

                    self.mc.set_color(0, 255, 0)
                    response.success = True
                    response.message = "Robot Ready (WAIT)"

                elif command == "pick_good":
                    x, y, raw_angle = target[0], target[1], target[2]
                    
                    # (좌표 계산 로직 기존과 동일)
                    dist = math.sqrt(x**2 + y**2)
                    dynamic_rx = -175.0 
                    down_depth = 55  
                    corrected_angle = raw_angle
                    final_grip_yaw = (corrected_angle + 45) % 90 - 45

                    # [1단계] 상공 이동
                    target_pose = [x, y, high, dynamic_rx, PICK_ORIENTATION_RY, final_grip_yaw]
                    self.get_logger().info(f"🚀 접근: {target_pose}")
                    self.mc.send_coords(target_pose, speed, 0)
                    self.wait_until_arrival(target_pose, mode='coords', tol=5.0)

                    # [1.5단계] 회전 보정
                    self.mc.send_angle(6, final_grip_yaw, speed)
                    time.sleep(0.5) 
                    
                    # [2단계] 하강
                    descent_pose = list(target_pose)
                    descent_pose[2] = high - down_depth 
                    
                    self.get_logger().info(f"⬇️ 하강: {descent_pose}")
                    self.mc.send_coords(descent_pose, speed, 0)
                    self.wait_until_arrival(descent_pose, mode='coords', tol=5.0)

                    # [2.5단계] 착지 확인
                    self.mc.send_angle(6, final_grip_yaw, speed)
                    time.sleep(0.5) 
                    self.wait_until_arrival(descent_pose, mode='coords', tol=8.0) 

                    # 4. 잡기
                    self.get_logger().info("✊ 그리퍼 닫기")
                    self.mc.set_gripper_value(40, 20, 1) 
                    time.sleep(1.5)

                    # 5. 상승 & 이동
                    SAFE_Z_HEIGHT = high + 20
                    waypoint = list(descent_pose)
                    waypoint[0] *= 0.6
                    waypoint[1] *= 0.6
                    waypoint[2] = SAFE_Z_HEIGHT
                    
                    self.get_logger().info(f"↗️ 당겨서 올리기")
                    self.mc.send_coords(waypoint, speed, 0)
                    self.wait_until_arrival(waypoint, mode='coords', tol=10.0) 

                    # 6. 적재 위치
                    final_place_pose = list(BOX_BASE_POSE) 
                    final_place_pose[1] = BOX_BASE_POSE[1] + (self.pack_count * BOX_OFFSET_Y)
                    
                    hover_place_pose = list(final_place_pose)
                    hover_place_pose[2] = SAFE_Z_HEIGHT  
                    
                    self.get_logger().info("✈️ 적재 위치 상공으로 이동")
                    self.mc.send_coords(hover_place_pose, speed, 0)
                    self.wait_until_arrival(hover_place_pose, mode='coords', tol=5.0)

                    # 6-2. 하강
                    self.get_logger().info("⬇️ 박스 안으로 하강")
                    self.mc.send_coords(final_place_pose, speed, 0)
                    self.wait_until_arrival(final_place_pose, mode='coords', tol=5.0)
                    
                    # 7. 놓기
                    self.mc.set_gripper_value(100, 20, 1) 
                    time.sleep(1.0)
                    
                    # 8. 복귀 상승
                    self.get_logger().info("⬆️ 복귀를 위해 상승")
                    self.mc.send_coords(hover_place_pose, speed, 0)
                    self.wait_until_arrival(hover_place_pose, mode='coords', tol=5.0)

                    self.pack_count += 1
                    if self.pack_count >= 3:
                        self.pack_count = 0 

                    self.get_logger().info("🏠 홈으로 복귀")
                    self.mc.send_angles(POSE_WAIT, speed)
                    self.wait_until_arrival(POSE_WAIT, mode='angles', tol=3.0)

                    response.success = True
                    response.message = f"Done (Count: {self.pack_count})"

                elif command == "discard_bad":
                    response.success = True
                    response.message = "Discarded"
                
                self.mc.set_color(0, 255, 0)

            except Exception as e:
                self.mc.set_color(255, 0, 0) 
                response.success = False
                response.message = str(e)
                self.get_logger().fatal(f"🛑 작업 중단: {e}")
                self.log_motor_status()
                
                # 에러 발생 시에도 상태 한번 전송 (Red Light 상태 기록용)
                self.publish_robot_status()
            
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
            try:
                node.mc.stop()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
