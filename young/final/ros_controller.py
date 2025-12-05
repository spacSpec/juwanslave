#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger 
import cv2
import time
import random
import os

class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller')
        
        # 1. 서비스 서버 개설
        self.srv = self.create_service(Trigger, 'judge_item', self.handle_judgement)
        
        # 2. 카메라 초기화
        # 0번이 안 되면 2번이나 4번으로 바꿔보세요 (노트북 캠 vs USB 캠)
        self.camera_index = 2
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error('❌ 카메라 연결 실패! (인덱스 확인 필요)')
        else:
            self.get_logger().info('✅ 카메라 연결 성공! (준비 완료)')

    def process_vision(self):
        """
        [비전 처리] 촬영 -> 저장 -> 화면 띄우기 -> 가상 분석
        """
        if self.cap.isOpened():
            # 1. 촬영 (Capture)
            ret, frame = self.cap.read()
            
            if ret:
                # 2. 이미지 저장 (Save)
                file_name = "inspection_result.jpg"
                cv2.imwrite(file_name, frame)
                self.get_logger().info(f"   📸 촬영 완료! '{file_name}' 저장됨")
                
                # 3. 화면 띄우기 (Show)
                # "Vision Inspection"이라는 창을 띄웁니다.
                cv2.imshow("Vision Inspection", frame)
                
                # 4. 분석 시뮬레이션 (2초 대기)
                # 이 시간 동안 창이 유지됩니다. (waitKey가 없으면 창이 안 뜹니다!)
                self.get_logger().info("   🧠 [AI] 불량 판독 중...")
                cv2.waitKey(2000) 
                
                # 5. 창 닫기
                cv2.destroyAllWindows()
            else:
                self.get_logger().error("   ⚠️ 프레임 읽기 실패")
        else:
            self.get_logger().warn("   ⚠️ 카메라 없음 (시뮬레이션 모드)")
            time.sleep(2.0)

        # --- [가상 판정 결과] ---
        # 실제로는 여기서 YOLO 결과를 써야 합니다.
        detected_count = random.randint(0, 5) 
        is_bad = random.choice([True, False])
        
        return detected_count, is_bad

    def handle_judgement(self, request, response):
        self.get_logger().info('\n📥 [PLC -> Controller] 도착 신호 수신! 검사 시작...')
        
        # 비전 처리 실행 (여기서 화면이 뜹니다!)
        count, is_defective = self.process_vision()
        
        self.get_logger().info(f"   📊 [결과] 개수: {count}개 / 불량: {is_defective}")

        # 판단 로직
        if count == 0:
            response.success = False
            response.message = "ERROR_EMPTY"
            self.get_logger().error("   🚨 [판정] 물건 없음 -> 알람")
        elif is_defective:
            response.success = False
            response.message = "PASS_RUN"
            self.get_logger().warn("   ⚠️ [판정] 불량품 -> 계속 가동 (PASS)")
        else:
            response.success = True
            response.message = "STOP_PICK"
            self.get_logger().info("   ✅ [판정] 양품 -> 정지 & 작업 (PICK)")
            
        return response

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RosController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()