import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import DetectionResult
from pymycobot.mycobot import MyCobot
import time
import threading
import sys

# ===================== [설정] =====================
PORT = '/dev/ttyACM2'
BAUD = 115200
SAFE_Z = 320  # 검증 시 충돌 방지를 위해 충분히 띄운 높이 (mm)
SPEED = 10

class CalibrationTestNode(Node):
    def __init__(self):
        super().__init__('calibration_test_node')
        
        # 1. Vision 데이터 구독
        self.create_subscription(DetectionResult, '/vision/result', self.vision_callback, 10)
        self.latest_msg = None

        # 2. 로봇 직접 연결 (검증용이라 서비스 안 거치고 직접 제어)
        try:
            self.mc = MyCobot(PORT, BAUD)
            time.sleep(0.5)
            self.mc.power_on()
            self.get_logger().info(f'✅ Connected to MyCobot for Testing')
            self.mc.set_gripper_value(100,20,1)
        except Exception as e:
            self.get_logger().error(f'❌ Connection Failed: {e}')
            sys.exit(1)

        # 3. 키보드 입력 쓰레드
        threading.Thread(target=self.input_loop, daemon=True).start()

    def vision_callback(self, msg):
        self.latest_msg = msg

    def input_loop(self):
        print("\n" + "="*50)
        print(" [AGV 천재 심영주님의 검증 툴]")
        print("  - 'a' + 엔터 : [각도] J6 회전 방향 테스트")
        print("  - 'c' + 엔터 : [좌표] X, Y 위치 정확도 테스트")
        print("  - 'h' + 엔터 : 홈 위치 복귀")
        print("  - 'q' + 엔터 : 종료")
        print("="*50 + "\n")

        while rclpy.ok():
            cmd = input("명령 입력 >> ")
            
            if cmd == 'q':
                rclpy.shutdown()
                sys.exit()
                
            elif cmd == 'h': # 홈으로
                self.mc.send_angles([0,0,0,0,90,0], SPEED)
                
            elif cmd == 'a': # Angle Test
                self.test_angle()
                
            elif cmd == 'c': # Coordinate Test
                self.test_coordinate()

    def test_angle(self):
        """[검증 1] J6 회전 방향 확인"""
        if not self.check_vision(): return
        
        angle = self.latest_msg.center[2]
        print(f"\n🔍 [Vision] 감지된 각도: {angle:.2f}도")
        
        # 1. 먼저 홈 자세(J6=0) 근처로 이동 (안전)
        #    그리퍼가 바닥을 보게 정렬 (X,Y,Z, Rx,Ry,Rz) -> Rz(J6)를 0으로
        print("   -> 로봇을 '기준 자세'로 정렬합니다 (J6=0)...")
        # 현재 위치에서 자세만 잡습니다. (안전하게 Z 높임)
        cur = self.mc.get_coords()
        if cur:
            # 안전 높이, 수직 자세, J6=0 (-180, 0, 0 등 상황에 맞게)
            # 여기서는 관절 각도로 안전하게 초기화 추천
            self.mc.send_angles([0, -10, -120, 40, 90, 0], SPEED) 
            time.sleep(3)
        
        # 2. 감지된 각도만큼 회전
        print(f"   -> J6 축을 {angle:.2f}도 만큼 회전합니다.")
        cur_angles = self.mc.get_angles()
        if cur_angles:
            target_j6 = cur_angles[5] + angle  # [핵심 검증 포인트]
            
            # 범위 제한
            if target_j6 > 170: target_j6 = 170
            if target_j6 < -170: target_j6 = -170
            
            cur_angles[5] = target_j6
            self.mc.send_angles(cur_angles, SPEED)
            print(f"   Done. 로봇 그리퍼와 물체가 '평행'한지 확인하세요.")
            print("   👉 평행하면 부호 일치! / 엇갈리면 부호 반대(change to -=)!")

    def test_coordinate(self):
        """[검증 2] X, Y 좌표 일치 확인 (Z축 찌르기)"""
        if not self.check_vision(): return
        
        tx = self.latest_msg.center[0]
        ty = self.latest_msg.center[1]
        print(f"\n🎯 [Vision] 목표 좌표: X={tx:.1f}, Y={ty:.1f}")
        
        # 1. 해당 좌표의 '공중'으로 이동
        print(f"   -> 로봇을 해당 좌표 위(Z={SAFE_Z})로 이동시킵니다.")
        
        # Rx=-180, Ry=0, Rz=0 (수직 아래 보기)
        self.mc.send_coords([tx, ty, SAFE_Z, -180, 0, 0], SPEED, 0)
        
        print("   Done. 로봇 끝(그리퍼 중심)이 물체 정중앙 위에 있나요?")
        print("   👉 정확하면 통과! / 어긋나면 Calibration Matrix 수정 필요!")

    def check_vision(self):
        if self.latest_msg is None or not self.latest_msg.is_detected:
            print("⚠️ Vision 데이터가 없거나 물체가 없습니다!")
            return False
        return True

def main():
    rclpy.init()
    node = CalibrationTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()