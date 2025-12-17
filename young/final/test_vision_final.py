#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger 
from my_robot_interfaces.srv import DetectItem 
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
import torch.nn.functional as F
from torchvision.models import wide_resnet50_2, Wide_ResNet50_2_Weights
import time 
import threading 
import math
from PIL import Image as PILImage
from torchvision import transforms
from ultralytics import YOLO

# ==============================================================================
# [설정] 경로 및 파라미터 
# ==============================================================================
YOLO_WEIGHTS_PATH = '/home/young/runs/obb/train3/weights/best.pt' 
WEIGHTS_PATH = '/home/young/final_ws/src/final/final/padim_weights/cube'
DEFECT_IMAGE_SAVE_DIR = '/home/young/final_ws/src/final/defect_images' 

FIXED_SIZE = 120 
NUM_RANDOM_CHANNELS = 300 
TOP_N_PATCHES = 10 
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
ANOMALY_THRESHOLD = 91.0 
TARGET_DEVICE_ID = '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0'

# [복구됨] 고해상도(1280x960) 기준 ROI 좌표
ROI_X = 470
ROI_Y = 130
ROI_W = 300
ROI_H = 350

# [복구됨] 박스 상태 확인용 ROI (1280x960 기준)
BOX_ROI_X = 850
BOX_ROI_Y = 580
BOX_ROI_W = 400
BOX_ROI_H = 350

BOX_FULL_THRESHOLD = 3  

CUBE_REAL_SIZE_MM = 50.0 
CENTER_ROBOT_X = -42.0   
CENTER_ROBOT_Y = 230.0   
FRAME_TIMEOUT_SEC = 1.0 

# [천재 심영주 HSV 설정값] (툴 검증 완료)
HSV_LOWER_GREEN = np.array([51, 69, 69])
HSV_UPPER_GREEN = np.array([91, 255, 255])

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.TOP_N_PATCHES = TOP_N_PATCHES
        self.ANOMALY_THRESHOLD = ANOMALY_THRESHOLD
        self.DEVICE = DEVICE
        
        self.last_detected_box = None
        self.last_detected_quality = ""
        self.last_detected_score = 0.0
        self.last_detect_time = 0.0
        
        self.is_camera_open = False
        self.latest_frame = None      
        self.last_frame_time = 0.0    
        self.frame_lock = threading.Lock() 
        self.ai_lock = threading.Lock()    
        self.running = True           

        self.image_pub = self.create_publisher(RosImage, '/vision/defect_img', 10)
        
        # 1. 픽킹 좌표 감지 서비스
        self.detect_srv = self.create_service(DetectItem, '/vision/detect_item', self.handle_detection_request)
        
        # 2. 박스 상태 확인 서비스
        self.box_check_srv = self.create_service(Trigger, '/vision/check_box_full', self.handle_box_check_request)
        
        if not os.path.exists(DEFECT_IMAGE_SAVE_DIR):
            os.makedirs(DEFECT_IMAGE_SAVE_DIR)
        
        self.bridge = CvBridge()
        self.get_logger().info(f"🚀 Vision Node Started. Using Device: {self.DEVICE}")

        self.transform = transforms.Compose([
            transforms.Resize((224, 224), PILImage.LANCZOS),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        # 모델 로드
        self.load_yolo_model()
        self.load_padim_model()
        self.setup_camera()
        
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.display_timer = self.create_timer(0.033, self._display_callback)

    def load_yolo_model(self):
        try:
            self.get_logger().info(f"🚀 Loading YOLO OBB: {YOLO_WEIGHTS_PATH}")
            self.yolo_model = YOLO(YOLO_WEIGHTS_PATH, task='obb')
        except Exception as e:
            self.get_logger().error(f"❌ YOLO Fail: {e}")

    def find_camera_index(self, device_path):
        if not os.path.exists(device_path): return None
        try:
            real_path = os.path.realpath(device_path)
            if 'video' in real_path and real_path.startswith('/dev/video'):
                return int(real_path.split('video')[-1])
            return None
        except: return None
            
    def setup_camera(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        camera_index = self.find_camera_index(TARGET_DEVICE_ID)
        if camera_index is None: camera_index = 4
        
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            # [설정] 고해상도(1280x960) & FPS 15 (대역폭 안정화)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            self.cap.set(cv2.CAP_PROP_FPS, 15)
            
            time.sleep(2) 
            self.is_camera_open = True
            
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f"✅ Camera {camera_index} OK. Res: {int(w)}x{int(h)}")
        else:
            self.is_camera_open = False
    
    def _capture_loop(self):
        fail_count = 0
        while self.running and rclpy.ok():
            if not self.is_camera_open or not self.cap.isOpened():
                self.setup_camera(); time.sleep(1.0); continue
            ret, frame = self.cap.read()
            if ret:
                fail_count = 0 
                with self.frame_lock:
                    self.latest_frame = frame
                    self.last_frame_time = time.time()
            else:
                fail_count += 1
                if fail_count > 30:
                    self.is_camera_open = False; self.cap.release(); fail_count = 0; time.sleep(1.0)
            time.sleep(0.005)

    def _display_callback(self):
        if not self.running: return
        frame_to_show = None
        with self.frame_lock:
            if self.latest_frame is not None:
                frame_to_show = self.latest_frame.copy()
        
        if frame_to_show is not None:
            # 픽킹 ROI (Blue)
            cv2.rectangle(frame_to_show, (ROI_X, ROI_Y), (ROI_X+ROI_W, ROI_Y+ROI_H), (255, 0, 0), 2)
            cv2.putText(frame_to_show, "Pick ROI", (ROI_X, ROI_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # 박스 ROI (Yellow)
            cv2.rectangle(frame_to_show, (BOX_ROI_X, BOX_ROI_Y), (BOX_ROI_X+BOX_ROI_W, BOX_ROI_Y+BOX_ROI_H), (0, 255, 255), 2)
            cv2.putText(frame_to_show, "Box ROI", (BOX_ROI_X, BOX_ROI_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            if self.last_detected_box is not None and (time.time() - self.last_detect_time < 5.0):
                color = (0, 0, 255) if self.last_detected_quality == "DEFECT" else (0, 255, 0)
                cv2.drawContours(frame_to_show, [self.last_detected_box], 0, color, 3)
                
                # 시각화 라벨에 각도 추가
                label = f"{self.last_detected_quality} ({self.last_detected_score:.1f})"
                cv2.putText(frame_to_show, label, (self.last_detected_box[1][0], self.last_detected_box[1][1] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            cv2.imshow("Vision View", frame_to_show)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False; rclpy.shutdown()

    def load_padim_model(self):
        self.get_logger().info("🧠 Loading PaDiM...")
        try:
            mean_vec_np = np.load(os.path.join(WEIGHTS_PATH, 'mean_vector.npy'))
            inv_cov_np = np.load(os.path.join(WEIGHTS_PATH, 'inv_cov_matrix.npy'))
            self.random_channels = np.load(os.path.join(WEIGHTS_PATH, 'random_channels.npy'))
            self.mean_vector = torch.from_numpy(mean_vec_np).to(self.DEVICE).float()
            self.inv_cov_matrix = torch.from_numpy(inv_cov_np).to(self.DEVICE).float()
            
            self.model = wide_resnet50_2(weights=Wide_ResNet50_2_Weights.IMAGENET1K_V2)
            self.feature_maps = {}
            def hook_fn(module, input, output, name):
                if name == 'layer3': output = F.interpolate(output, size=(28, 28), mode='bilinear', align_corners=False)
                elif name == 'layer1': output = F.avg_pool2d(output, kernel_size=2)
                self.feature_maps[name] = output
            self.model.layer1.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer1'))
            self.model.layer2.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer2'))
            self.model.layer3.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer3'))
            self.model.fc = torch.nn.Identity(); self.model.eval(); self.model.to(self.DEVICE)
        except Exception as e: self.model = None

    def extract_features(self, img_bgr):
        roi_img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(roi_img_rgb)
        input_tensor = self.transform(pil_img).unsqueeze(0).to(self.DEVICE)
        with torch.no_grad(): _ = self.model(input_tensor)
        combined = torch.cat([self.feature_maps['layer1'], self.feature_maps['layer2'], self.feature_maps['layer3']], dim=1)
        return combined.permute(0, 2, 3, 1).flatten(1, 2).squeeze(0)

    def detect_anomaly(self, img_bgr):
        if self.model is None: return "ERROR", 0.0
        features_reduced = self.extract_features(img_bgr)[:, self.random_channels]
        delta = features_reduced - self.mean_vector
        dist = torch.sqrt(torch.abs(torch.sum(torch.matmul(delta, self.inv_cov_matrix) * delta, dim=1)))
        score = torch.mean(torch.topk(dist, k=min(self.TOP_N_PATCHES, len(dist)))[0]).item()
        quality = "DEFECT" if score > self.ANOMALY_THRESHOLD else "GOOD"
        return quality, score

    def crop_rotated_rect(self, image, center, rotation_rad, target_size):
        angle_deg = math.degrees(rotation_rad)
        
        # 정사각형 90도 주기 정규화
        angle_deg = angle_deg % 90
        if angle_deg > 45:
            angle_deg -= 90
        
        rotation_angle = -angle_deg 
        M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
        
        rotated = cv2.warpAffine(
            image, M, 
            (image.shape[1], image.shape[0]), 
            borderMode=cv2.BORDER_REPLICATE
        )
        
        crop_half = target_size // 2
        cx, cy = int(center[0]), int(center[1])
        
        start_y = max(0, cy - crop_half)
        end_y = start_y + target_size
        start_x = max(0, cx - crop_half)
        end_x = start_x + target_size
        
        cropped = rotated[start_y:end_y, start_x:end_x]
        
        if cropped.shape[0] != target_size or cropped.shape[1] != target_size:
            return None
            
        return cropped

    def pixel_to_robot(self, px, py, mm_per_pixel):
        dx_px = px - (ROI_W / 2)
        dy_px = py - (ROI_H / 2)
        dx_mm = dx_px * mm_per_pixel
        dy_mm = dy_px * mm_per_pixel
        robot_x = (dy_mm * -1.1) + CENTER_ROBOT_X
        robot_y = (dx_mm * -1.2) + CENTER_ROBOT_Y
        return robot_x, robot_y

    def save_defect_image_to_file(self, image_bgr):
        try:
            filename = f"defect_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(os.path.join(DEFECT_IMAGE_SAVE_DIR, filename), image_bgr)
        except: pass

    # ==========================================================================
    # 박스 상태 확인 핸들러 (서비스 콜백)
    # ==========================================================================
    def handle_box_check_request(self, request, response):
        self.get_logger().info("📦 Vision: Checking Box (Hybrid Mode: YOLO + HSV)...")
        
        raw = None
        with self.frame_lock:
            if self.latest_frame is not None:
                raw = self.latest_frame.copy()
        
        if raw is None:
            response.success = False; response.message = "NO_FRAME"; return response

        try:
            # ROI 추출 (고해상도 기준)
            roi_img = raw[BOX_ROI_Y : BOX_ROI_Y+BOX_ROI_H, BOX_ROI_X : BOX_ROI_X+BOX_ROI_W]
            if roi_img.size == 0:
                response.success = False; response.message = "ROI_ERROR"; return response

            # A. YOLO Check
            yolo_count = 0
            try:
                results = self.yolo_model(roi_img, imgsz=640, conf=0.25, verbose=False)
                for r in results:
                    if r.obb is not None:
                        yolo_count += len(r.obb)
            except Exception as e:
                self.get_logger().warn(f"YOLO Check Fail: {e}")

            # B. HSV Check (수정된 HSV 값 적용)
            hsv_count = 0
            hsv_area_total = 0
            try:
                hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, HSV_LOWER_GREEN, HSV_UPPER_GREEN)
                
                kernel = np.ones((5,5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    # 고해상도이므로 노이즈 임계값도 원래대로(1000) 유지
                    if area > 1000: 
                        hsv_count += 1
                        hsv_area_total += area
            except Exception as e:
                self.get_logger().warn(f"HSV Check Fail: {e}")

            cond_yolo = (yolo_count >= BOX_FULL_THRESHOLD)
            cond_hsv_count = (hsv_count >= BOX_FULL_THRESHOLD)
            # 면적 임계값도 고해상도 기준(12000)으로 유지
            cond_hsv_area = (hsv_area_total > 20000) 

            is_full = cond_yolo or cond_hsv_count or cond_hsv_area
            log_msg = f"📦 Result - YOLO:{yolo_count}, HSV_Cnt:{hsv_count}, Area:{int(hsv_area_total)}"
            self.get_logger().info(log_msg)

            if is_full:
                response.success = True; response.message = f"FULL ({log_msg})"
            else:
                response.success = False; response.message = f"NOT_FULL ({log_msg})"

        except Exception as e:
            self.get_logger().error(f"❌ Box Check Logic Error: {e}")
            response.success = False; response.message = "ERROR"
            
        return response

    # ==========================================================================
    # [핵심] 픽킹 서비스 핸들러 (YOLO 위치 + OpenCV 각도)
    # ==========================================================================
    def handle_detection_request(self, request, response):
        self.get_logger().info("▶️ Service Call Received")
        if not self.ai_lock.acquire(blocking=False):
            response.success = False; response.message = "BUSY"; return response

        try:
            raw = None
            with self.frame_lock:
                if self.latest_frame is not None and (time.time() - self.last_frame_time) < FRAME_TIMEOUT_SEC:
                    raw = self.latest_frame.copy()
            
            if raw is None:
                response.success = False; response.message = "FRAME_STALE"; return response

            # 1. YOLO로 "위치" 찾기
            results = self.yolo_model(raw, imgsz=640, conf=0.5, verbose=False)
            
            quality = "NO_OBJECT"; robot_x = 0.0; robot_y = 0.0; angle_res = 0.0; score = 0.0
            found_obb = False
            best_obb = None

            for r in results:
                if r.obb is None: continue
                obb_data = r.obb.xywhr.cpu().numpy()
                
                for obb in obb_data:
                    cx, cy, w, h, rot = obb
                    # ROI 필터링
                    if (ROI_X <= cx <= ROI_X + ROI_W) and (ROI_Y <= cy <= ROI_Y + ROI_H):
                        best_obb = obb
                        found_obb = True
                        break
                if found_obb: break

            if found_obb:
                cx, cy, w, h, _ = best_obb # YOLO 각도 무시
                center = (cx, cy)
                
                # ------------------------------------------------------------------
                # [천재 솔루션] OpenCV HSV + minAreaRect로 "진짜 각도" 계산
                # ------------------------------------------------------------------
                try:
                    # 1. YOLO 중심 기준 Crop
                    crop_size = 200 # 고해상도니까 조금 더 크게 잡음
                    x1 = max(0, int(cx - crop_size//2))
                    y1 = max(0, int(cy - crop_size//2))
                    x2 = min(raw.shape[1], int(cx + crop_size//2))
                    y2 = min(raw.shape[0], int(cy + crop_size//2))
                    
                    crop_img = raw[y1:y2, x1:x2]
                    
                    # 2. HSV 마스킹 (검증된 값 사용)
                    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(hsv, HSV_LOWER_GREEN, HSV_UPPER_GREEN)
                    
                    # 3. 윤곽선 및 최소 외접 사각형
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    real_angle = 10.0
                    if contours:
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c) 
                        
                        # minAreaRect의 각도 -> -45 ~ 45도로 정규화
                        raw_cv_angle = rect[2]
                        if raw_cv_angle > 45:
                            real_angle = raw_cv_angle - 90
                        elif raw_cv_angle < -45:
                            real_angle = raw_cv_angle + 90
                        else:
                            real_angle = raw_cv_angle
                            
                        # 하드웨어 오차 보정값 (Calibration Offset)
                        OFFSET_ANGLE = -20.0 

    # 계산된 각도에 보정값을 더해줍니다.
                        angle_res = real_angle + OFFSET_ANGLE
                        
                        self.get_logger().info(f"📐 OpenCV Angle: {angle_res:.1f} deg")
                    else:
                        angle_res = math.degrees(best_obb[4]) # 실패시 백업

                except Exception as e:
                    self.get_logger().warn(f"Angle Calc Fail: {e}")
                    angle_res = math.degrees(best_obb[4])

                # ------------------------------------------------------------------
                
                cube_px = max(w, h)
                mm_per_pixel = CUBE_REAL_SIZE_MM / cube_px if cube_px > 0 else 1.0
                
                roi_rel_x = cx - ROI_X
                roi_rel_y = cy - ROI_Y
                robot_x, robot_y = self.pixel_to_robot(roi_rel_x, roi_rel_y, mm_per_pixel)

                rotation_rad = math.radians(angle_res)
                ai_input = self.crop_rotated_rect(raw, center, rotation_rad, FIXED_SIZE)
                
                if ai_input is not None and ai_input.size > 0:
                    quality, score = self.detect_anomaly(ai_input)
                else:
                    quality = "ERROR"

                if r.obb.xyxyxyxy is not None:
                     rect_viz = ((cx, cy), (w, h), angle_res)
                     viz_box = cv2.boxPoints(rect_viz).astype(int)
                     self.last_detected_box = viz_box
                
                self.last_detected_quality = quality
                self.last_detected_score = score
                self.last_detect_time = time.time()

                color = (0, 0, 255) if quality == "DEFECT" else (0, 255, 0)
                if self.last_detected_box is not None:
                    cv2.drawContours(raw, [self.last_detected_box], 0, color, 3)
                    label_str = f"{quality} {angle_res:.1f}deg"
                    cv2.putText(raw, label_str, 
                               (self.last_detected_box[1][0], self.last_detected_box[1][1]-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                if quality == "DEFECT": self.save_defect_image_to_file(raw)
                
                self.get_logger().info(f"🔎 {quality} (Score:{score:.1f}) | Angle: {angle_res:.1f} | Pos: {robot_x:.1f}, {robot_y:.1f}")

            else:
                self.get_logger().info("👀 No OBB Detection in ROI")

            response.success = True; response.quality = quality
            if quality in ["GOOD", "DEFECT", "ERROR"]:
                response.message = f"{quality} (Score: {score:.1f})"
                response.center = [float(robot_x), float(robot_y)]
                response.angle = float(angle_res)
            else:
                response.message = "NO_OBJECT"; response.center = [0.0, 0.0]; response.angle = 0.0

        except Exception as e:
            self.get_logger().error(f"❌ Logic Error: {e}")
            response.success = False; response.quality = "ERROR"
        finally:
            self.ai_lock.release()
            try: self.image_pub.publish(self.bridge.cv2_to_imgmsg(raw, "bgr8"))
            except: pass
            
        return response

    def __del__(self):
        self.running = False 
        if hasattr(self, 'capture_thread'): self.capture_thread.join(timeout=1.0) 
        if hasattr(self, 'cap'): self.cap.release()
        cv2.destroyAllWindows() 

def main(args=None):
    rclpy.init(args=args); node = VisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
