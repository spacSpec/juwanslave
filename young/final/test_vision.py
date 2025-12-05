# ~/final_ws/src/final/final/vision_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import DetectionResult
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge     
import cv2
import numpy as np
import math
import random 

# ===================== [NEW] 렌즈 캘리브레이션 (1280x960 기준) =====================
CALIB_MATRIX_K = np.array([
    [1.25038936e+03, 0.00000000e+00, 5.74939770e+02],
    [0.00000000e+00, 1.26131675e+03, 4.72721799e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])
DIST_COEFF_D = np.array([
    [0.04689649, 0.54787717, 0.00547649, -0.0037721, -1.14700805]
])
# ==============================================================================

# ===================== [EXISTING] 좌표 변환 행렬 (Perspective) =================
PERSPECTIVE_MATRIX = np.array([
    [0.06216, 0.54554, -63.02774],
    [0.71376, -0.37548, -382.46765],
    [-0.00051, 0.00114, 1.00000]
])
# ==============================================================================

# ===================== ROI 설정 =====================
ROI_X = 350 
ROI_Y = 130   
ROI_W = 440  
ROI_H = 350  

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.result_pub = self.create_publisher(DetectionResult, '/vision/result', 10)
        self.image_pub = self.create_publisher(Image, '/vision/defect_img', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info('🚀 [SYSTEM] 1280x960 Res & Lens Calibration Applied!')

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('❌ Camera 2 Open Failed. Trying 0...')
            self.cap = cv2.VideoCapture(0)
            
        # ================= [수정됨] 해상도 1280 x 960 설정 =================
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        # =================================================================

        self.timer = self.create_timer(0.03, self.process_frame)

    def pixel_to_robot(self, u, v):
        """좌표 변환: 픽셀 -> 로봇 (Undistorted 이미지 기준)"""
        pixel_point = np.array([[[u, v]]], dtype=np.float32)
        robot_point = cv2.perspectiveTransform(pixel_point, PERSPECTIVE_MATRIX)
        return [robot_point[0][0][0], robot_point[0][0][1]]

    def get_angle_from_roi(self, full_frame, x1, y1, x2, y2):
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
        if angle < -45: angle = 90 + angle
        return angle

    def process_frame(self):
        ret, raw_frame = self.cap.read()
        if not ret: return

        # 1. 렌즈 왜곡 보정 (Undistort)
        # 1280x960 해상도에 맞는 K 행렬을 사용하여 펴줍니다.
        frame = cv2.undistort(raw_frame, CALIB_MATRIX_K, DIST_COEFF_D, None, CALIB_MATRIX_K)

        # 2. ROI 자르기
        roi_img = frame[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]

        # 3. 물체 인식 로직
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detection_found = False
        
        if cnts:
            largest_cnt = max(cnts, key=cv2.contourArea)
            
            if cv2.contourArea(largest_cnt) > 2000:
                lx, ly, lw, lh = cv2.boundingRect(largest_cnt)
                
                global_x1 = lx + ROI_X
                global_y1 = ly + ROI_Y
                global_x2 = lx + lw + ROI_X
                global_y2 = ly + lh + ROI_Y

                cx = int((global_x1 + global_x2) / 2)
                cy = int((global_y1 + global_y2) / 2)

                angle = self.get_angle_from_roi(frame, global_x1, global_y1, global_x2, global_y2)
                
                # 좌표 변환 (보정된 픽셀 좌표 -> 로봇 좌표)
                robot_coord = self.pixel_to_robot(cx, cy)

                is_defect = False
                quality_str = "BAD" if is_defect else "GOOD"

                msg = DetectionResult()
                msg.is_detected = True
                msg.quality = quality_str
                msg.center = [float(robot_coord[0]), float(robot_coord[1]), float(angle)]
                self.result_pub.publish(msg)
                detection_found = True
                
                if is_defect:
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        self.image_pub.publish(img_msg)
                    except Exception as e: pass
                
                color = (0, 0, 255) if is_defect else (0, 255, 0)
                
                cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (255, 0, 0), 2)
                cv2.rectangle(frame, (global_x1, global_y1), (global_x2, global_y2), color, 2)
                
                info_text = f"{quality_str} X:{robot_coord[0]:.0f} Y:{robot_coord[1]:.0f}"
                cv2.putText(frame, info_text, (global_x1, global_y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

        if not detection_found:
            msg = DetectionResult()
            msg.is_detected = False
            self.result_pub.publish(msg)

        cv2.imshow("Vision Eye (1280x960 Undistorted)", frame)
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