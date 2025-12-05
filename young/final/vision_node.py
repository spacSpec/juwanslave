# ~/final_ws/src/final/final/vision_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import DetectionResult
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge     
import cv2
import numpy as np
import math
from ultralytics import YOLO
import torch

# ===================== [NEW] 캘리브레이션 행렬 =====================
# 작성자님이 구하신 '보물지도 변환 데이터'입니다.
# 이제 SCALE_X, OFFSET_X 같은 건 필요 없습니다!
CALIB_MATRIX = np.array([
    [0.06216, 0.54554, -63.02774],
    [0.71376, -0.37548, -382.46765],
    [-0.00051, 0.00114, 1.00000]
])

# ===================== ROI 설정 =====================
ROI_X = 100  
ROI_Y = 50   
ROI_W = 440  
ROI_H = 380  

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.result_pub = self.create_publisher(DetectionResult, '/vision/result', 10)
        self.image_pub = self.create_publisher(Image, '/vision/defect_img', 10)
        self.bridge = CvBridge()

        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'🚀 Accelerating Inference on Device: {self.device}')
        
        self.model_path = '/home/young/Downloads/best.pt' 
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f'❌ Model Load Error: {e}')
            return

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('❌ Camera 2 Open Failed. Trying 0...')
            self.cap = cv2.VideoCapture(0)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.03, self.process_frame)

    # ================= [핵심 수정] 행렬 변환 함수 =================
    def pixel_to_robot(self, u, v):
        """
        픽셀 좌표(u, v) -> 로봇 좌표(x, y) 변환 (Matrix 사용)
        """
        # OpenCV 함수에 넣기 위해 3차원 배열로 변환
        pixel_point = np.array([[[u, v]]], dtype=np.float32)
        
        # 행렬 연산 (투시 변환) 수행!
        robot_point = cv2.perspectiveTransform(pixel_point, CALIB_MATRIX)
        
        # 결과값: [x, y]
        rx = robot_point[0][0][0]
        ry = robot_point[0][0][1]
        
        # 로그로 변환값 확인 (디버깅용)
        # self.get_logger().info(f"Trans: ({u},{v}) -> ({rx:.1f}, {ry:.1f})")
        
        return [rx, ry]

    def get_angle_from_roi(self, full_frame, x1, y1, x2, y2):
        """깨진 큐브도 잡아내는 '고무줄(Convex Hull)' 각도 계산"""
        h, w = full_frame.shape[:2]
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w, int(x2)), min(h, int(y2))

        roi = full_frame[y1:y2, x1:x2]
        if roi.size == 0: return 0.0

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return 0.0
        
        cnt = max(cnts, key=cv2.contourArea)
        hull = cv2.convexHull(cnt)
        rect = cv2.minAreaRect(hull) 
        angle = rect[2]

        if angle < -45: 
            angle = 90 + angle
            
        return angle

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        # 1. ROI 자르기
        roi_img = frame[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]

        # 2. YOLO 추론
        results = self.model.predict(roi_img, imgsz=640, conf=0.55, device=self.device, verbose=False)
        
        detection_found = False
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            box = max(results[0].boxes, key=lambda b: float(b.conf[0]))
            
            if self.device == '0':
                local_x1, local_y1, local_x2, local_y2 = box.xyxy[0].cpu().numpy()
            else:
                local_x1, local_y1, local_x2, local_y2 = box.xyxy[0].numpy()
                
            conf = float(box.conf[0])
            
            # [중요] Global 좌표 복원 (ROI 오프셋 더하기)
            # 이 좌표가 캘리브레이션 행렬에 들어갈 진짜 픽셀 좌표입니다.
            global_x1 = local_x1 + ROI_X
            global_y1 = local_y1 + ROI_Y
            global_x2 = local_x2 + ROI_X
            global_y2 = local_y2 + ROI_Y

            cx = int((global_x1 + global_x2) / 2)
            cy = int((global_y1 + global_y2) / 2)

            angle = self.get_angle_from_roi(frame, global_x1, global_y1, global_x2, global_y2)
            
            # [수정됨] 행렬을 이용한 좌표 변환 호출!
            robot_coord = self.pixel_to_robot(cx, cy)
            
            # 로깅: 변환된 좌표 확인
            self.get_logger().info(f"📍 Detect: {quality_str} at Pixel({cx},{cy}) -> Robot({robot_coord[0]:.1f}, {robot_coord[1]:.1f})")

            is_defect = conf < 0.94
            quality_str = "BAD" if is_defect else "GOOD"
            
            msg = DetectionResult()
            msg.is_detected = True
            msg.quality = quality_str
            # 변환된 좌표(mm)를 그대로 보냅니다.
            msg.center = [float(robot_coord[0]), float(robot_coord[1]), float(angle)]
            self.result_pub.publish(msg)
            detection_found = True
            
            if is_defect:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_pub.publish(img_msg)
                except Exception as e: pass
            
            # 시각화
            color = (0, 0, 255) if is_defect else (0, 255, 0)
            cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (255, 0, 0), 2)
            cv2.rectangle(frame, (int(global_x1), int(global_y1)), (int(global_x2), int(global_y2)), color, 2)
            # 변환된 좌표를 화면에도 띄워줍니다 (확인용)
            label = f"{quality_str} X:{robot_coord[0]:.0f} Y:{robot_coord[1]:.0f}"
            cv2.putText(frame, label, (int(global_x1), int(global_y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if not detection_found:
            msg = DetectionResult()
            msg.is_detected = False
            self.result_pub.publish(msg)

        cv2.imshow("Vision Eye", frame)
        cv2.waitKey(1)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()