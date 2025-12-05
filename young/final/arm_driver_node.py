import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmCommand
from pymycobot.mycobot import MyCobot
import threading
import time
import math

# ===================== [설정] =====================
PORT = '/dev/ttyACM0'
BAUD = 115200
speed = 40  # 속도를 조금 높임 (테스트 후 조절)
high = 315
# 1. 홈 위치 (관절 각도: 안전한 대기 자세, 그리퍼가 정면/위를 봐도 상관없음)
POSE_WAIT = [0, 0, 0, 0, 90, 0]

# 2. 집기 접근 자세 (좌표 RPY: 그리퍼가 바닥을 수직으로 보는 자세) 
# 산업 현장 용어: End-Effector Orientation Constraint (말단 자세 구속)
# 로봇이 어디에 있든(XY가 변하든) 그리퍼 바닥면은 항상 땅을 보게(-175도) 고정합니다.
# [Rx, Ry, Rz]
PICK_ORIENTATION = [-180, -10, 0] 

# 3. 박스 적재 시작 위치 (좌표: X, Y, Z, Rx, Ry, Rz)
BOX_BASE_POSE =  [237.1, -1.6, 146.3, -173.09, 2.86, -91.76]

# 적재 간격
BOX_OFFSET_X = 60.0 

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')
        try:
            self.mc = MyCobot(PORT, BAUD)
            time.sleep(0.5)
            self.mc.power_on()
            time.sleep(0.5)
            self.get_logger().info(f'✅ MyCobot Connected on {PORT}')
        except Exception as e:
            self.get_logger().error(f'❌ Connection Failed: {e}')
            return

        self.srv = self.create_service(ArmCommand, '/arm/execute_cmd', self.handle_command)
        self.pack_count = 0
        
        self.mc.set_color(0, 255, 0) # 초록 (대기)
        
        # 초기 위치 이동
        self.mc.send_angles(POSE_WAIT, speed)
        self.wait_for_robot_stop()

    def wait_for_robot_stop(self, pos_tol=5.0, stable_time=0.2, timeout=10):
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
        command = request.command
        target = request.target_coord 
        
        self.get_logger().info(f"Command: {command}, Target: {target}")
        self.mc.set_color(0, 0, 255) # 파란색 (작업 중)

        try:
            if command == "home":
                self.get_logger().info("🏠 Moving to WAIT Position...")
                self.mc.send_angles(POSE_WAIT, speed)
                self.wait_for_robot_stop()
                
                # 그리퍼 초기화
                self.mc.set_gripper_mode(0)
                self.mc.init_electric_gripper()
                time.sleep(1)
                self.mc.set_gripper_value(100, 30, 1) # 열기
                time.sleep(1.0)

                self.mc.set_color(0, 255, 0)
                response.success = True
                response.message = "Robot Ready (WAIT)"
                return response

            if command == "pick_good":
                x, y, angle = target[0], target[1], target[2]
                
                # [산업 현장 방식: Decoupled Motion]
                # 1단계: 위치(XY) 이동 - 자세(RPY)는 바닥보기로 고정 (물체 회전 무시하고 일단 접근)
                self.get_logger().info(f"🚀 Moving to XY: {x}, {y} with DOWN orientation")
                
                # Z높이는 안전하게 유지, RPY는 바닥을 보게 고정(PICK_ORIENTATION)
                # 이렇게 하면 로봇이 XY 어디로 가든 그리퍼는 항상 수직 아래를 봅니다.
                target_pose = [x, y, high+30, *PICK_ORIENTATION] 
                self.mc.send_coords(target_pose, speed, 0)
                self.wait_for_robot_stop()

                # 2단계: 회전(Yaw) 보정 - 위치는 고정하고 손목(J6)만 돌림
                # 이것이 '물체에 맞춰 RPY를 변경'하는 과정입니다.
                # 2단계: 회전(Yaw) 보정
                current_angles = self.mc.get_angles()
                if current_angles:
                    # [안전 장치] J6가 너무 많이 돌아가 있으면 0으로 풀고 다시 계산
                    # 하지만 위에서 PICK_ORIENTATION(Rz=0)으로 이동했으므로 J6는 0 근처일 것입니다.
                    
                    # [중요] 카메라 각도 부호 확인 필요 (테스트 해보고 반대로 돌면 -= 로 변경)
                    target_j6 = current_angles[5] + angle 

                    # [안전 장치] MyCobot J6 한계 보호 (-170 ~ 170)
                    if target_j6 > 170: target_j6 = 170
                    if target_j6 < -170: target_j6 = -170
                    
                    current_angles[5] = target_j6
                    self.mc.send_angles(current_angles, speed)
                    self.wait_for_robot_stop()
                    
                # 3. 하강 (Z=315 근처, 물체 잡는 높이)
                # 좌표 기반으로 Z축만 내립니다.
                cur_coords = self.mc.get_coords()
                if cur_coords:
                    cur_coords[2] = high # 물체 높이에 맞춰 수정 필요
                    self.mc.send_coords(cur_coords, speed, 0)
                    self.wait_for_robot_stop()

                # 4. 잡기 (Grip)
                self.mc.set_gripper_value(45, 30, 1) # 꽉 잡기
                time.sleep(1.5)

                # 5. 상승 (Z=290 복귀)
                cur_coords = self.mc.get_coords()
                if cur_coords:
                    cur_coords[2] = high+30
                    self.mc.send_coords(cur_coords, speed, 0)
                    self.wait_for_robot_stop()

                # 6. 박스 적재 (Pick & Place)
                # 적재 위치 계산 (X축으로 오프셋 적용)
                final_place_pose = list(BOX_BASE_POSE) # 복사해서 사용
                final_place_pose[0] = BOX_BASE_POSE[0] + (self.pack_count * BOX_OFFSET_X)
                
                # 6-1. 적재 위치 '위'로 이동 (충돌 방지, Z+50)
                up_pose = list(final_place_pose)
                up_pose[2] += 100 
                self.mc.send_coords(up_pose, speed, 0)
                self.wait_for_robot_stop()
                
                # 6-2. 내려놓기 (원래 높이)
                self.mc.send_coords(final_place_pose, speed, 0)
                self.wait_for_robot_stop()
                
                # 6-3. 놓기 (Open)
                self.mc.set_gripper_value(90, 30, 1)
                time.sleep(1.0)
                
                # 6-4. 복귀 상승 (다시 위로)
                self.mc.send_coords(up_pose, speed, 0)
                self.wait_for_robot_stop()

                # 카운트 관리
                self.pack_count += 1
                if self.pack_count >= 3:
                    self.pack_count = 0 

                # 홈 복귀 (다시 안전한 관절 각도로)
                self.mc.send_angles(POSE_WAIT, speed)
                self.wait_for_robot_stop()

                response.success = True
                response.message = f"Pick & Place Done (Count: {self.pack_count})"

            elif command == "discard_bad":
                self.mc.set_gripper_value(100, 30, 1)
                time.sleep(1.0)
                response.success = True
                response.message = "Discarded"
            
            self.mc.set_color(0, 255, 0)

        except Exception as e:
            self.mc.set_color(255, 0, 0)
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Execution Error: {e}")
            
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