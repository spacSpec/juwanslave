# !/usr/bin/env python3    NNNNNNNNNNNEEEEEEEewWWWWWWWWWWWWWWWWWWWWWW!!!!
# sudo chmod 777 /dev/ttyACM0 
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
# ★ OBB 모델 경로로 수정 (학습시킨 best.pt 경로 확인 필수!)
YOLO_WEIGHTS_PATH = '/home/young/runs/obb/train3/weights/best.pt' 

# PaDiM 가중치 경로
WEIGHTS_PATH = '/home/young/final_ws/src/final/final/padim_weights/cube'
DEFECT_IMAGE_SAVE_DIR = '/home/young/final_ws/src/final/defect_images' 

# ★ 데이터셋 만들 때 썼던 그 크기와 똑같이 설정! (120)
FIXED_SIZE = 120 

NUM_RANDOM_CHANNELS = 300 
TOP_N_PATCHES = 10 
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
ANOMALY_THRESHOLD = 90.0 
TARGET_DEVICE_ID = '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0'

# ROI 및 로봇 파라미터
ROI_X = 470; ROI_Y = 130; ROI_W = 300; ROI_H = 350
CUBE_REAL_SIZE_MM = 50.0 
CENTER_ROBOT_X = -50.0   
CENTER_ROBOT_Y = 270.0   
FRAME_TIMEOUT_SEC = 1.0 

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
        self.detect_srv = self.create_service(DetectItem, '/vision/detect_item', self.handle_detection_request)
        
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
            # ★ 핵심: task='obb' 설정
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
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            self.cap.set(cv2.CAP_PROP_FPS, 5) 
            time.sleep(2) 
            self.is_camera_open = True
            self.get_logger().info(f"✅ Camera {camera_index} OK.")
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
            cv2.rectangle(frame_to_show, (ROI_X, ROI_Y), (ROI_X+ROI_W, ROI_Y+ROI_H), (255, 0, 0), 2)
            cv2.putText(frame_to_show, "ROI", (ROI_X, ROI_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            if self.last_detected_box is not None and (time.time() - self.last_detect_time < 5.0):
                color = (0, 0, 255) if self.last_detected_quality == "DEFECT" else (0, 255, 0)
                # OBB 박스 그리기
                cv2.drawContours(frame_to_show, [self.last_detected_box], 0, color, 3)
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

    # ==========================================================================
    # [★핵심 수정★] OBB 기반 회전 보정 크롭 함수 (학습 때 쓴 그 함수!)
    # ==========================================================================
    def crop_rotated_rect(self, image, center, rotation_rad, target_size):
        """
        OBB가 준 각도(rotation_rad)를 역보정하여 큐브를 0도 자세로 만듭니다.
        """
        # 1. 라디안 -> 도 변환
        angle_deg = math.degrees(rotation_rad)
        
        # 2. 역회전 각도 계산 (-angle)
        rotation_angle = -angle_deg 

        # 3. 회전 매트릭스 생성
        M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
        
        # 4. 이미지 전체 회전 (배경 복사)
        rotated = cv2.warpAffine(
            image, M, 
            (image.shape[1], image.shape[0]), 
            borderMode=cv2.BORDER_REPLICATE
        )
        
        # 5. 중심 기준으로 고정 크기(FIXED_SIZE)로 잘라내기
        crop_half = target_size // 2
        cx, cy = int(center[0]), int(center[1])
        
        start_y = max(0, cy - crop_half)
        end_y = start_y + target_size
        start_x = max(0, cx - crop_half)
        end_x = start_x + target_size
        
        cropped = rotated[start_y:end_y, start_x:end_x]
        
        # 크기 안맞으면(경계선 등) 실패 처리
        if cropped.shape[0] != target_size or cropped.shape[1] != target_size:
            return None
            
        return cropped

    def pixel_to_robot(self, px, py, mm_per_pixel):
        # ROI 중심 기준 좌표로 변환 (기존 로직 유지)
        dx_px = px - (ROI_W / 2)
        dy_px = py - (ROI_H / 2)
        
        dx_mm = dx_px * mm_per_pixel
        dy_mm = dy_px * mm_per_pixel
        
        # 로봇 좌표계 변환
        robot_x = (dy_mm * -1.0) + CENTER_ROBOT_X
        robot_y = (dx_mm * -1.0) + CENTER_ROBOT_Y
        return robot_x, robot_y

    def save_defect_image_to_file(self, image_bgr):
        try:
            filename = f"defect_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(os.path.join(DEFECT_IMAGE_SAVE_DIR, filename), image_bgr)
        except: pass

    # ==========================================================================
    # [서비스 핸들러] OBB 추론 -> OBB 좌표 사용 -> PaDiM
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

            # ------------------------------------------------------------------
            # 1. [YOLO OBB] 추론 실행
            # ------------------------------------------------------------------
            results = self.yolo_model(raw, imgsz=640, conf=0.5, verbose=False)
            
            quality = "NO_OBJECT"; robot_x = 0.0; robot_y = 0.0; angle_res = 0.0; score = 0.0
            found_obb = False
            best_obb = None

            for r in results:
                if r.obb is None: continue
                # xywhr: x_center, y_center, width, height, rotation(rad)
                obb_data = r.obb.xywhr.cpu().numpy()
                
                # ROI 안에 중심이 들어오는 객체 찾기
                for obb in obb_data:
                    cx, cy, w, h, rot = obb
                    if (ROI_X <= cx <= ROI_X + ROI_W) and (ROI_Y <= cy <= ROI_Y + ROI_H):
                        best_obb = obb
                        found_obb = True
                        break
                if found_obb: break

            if found_obb:
                cx, cy, w, h, rotation_rad = best_obb
                center = (cx, cy)
                
                # ------------------------------------------------------------------
                # 2. [좌표 변환] mm/pixel 및 로봇 좌표 계산
                # ------------------------------------------------------------------
                # OBB가 준 w, h 중 큰 값을 기준으로 스케일 계산
                cube_px = max(w, h)
                mm_per_pixel = CUBE_REAL_SIZE_MM / cube_px if cube_px > 0 else 1.0
                
                # ROI 기준 상대 좌표 계산
                # OBB 좌표(cx, cy)는 전체 이미지 기준이므로 ROI_X, ROI_Y를 빼야 함
                roi_rel_x = cx - ROI_X
                roi_rel_y = cy - ROI_Y
                robot_x, robot_y = self.pixel_to_robot(roi_rel_x, roi_rel_y, mm_per_pixel)

                # 각도 변환 (라디안 -> 도)
                angle_res = math.degrees(rotation_rad)

                # ------------------------------------------------------------------
                # 3. [Preprocessing] OBB 각도로 역회전 크롭 (학습과 동일 조건)
                # ------------------------------------------------------------------
                ai_input = self.crop_rotated_rect(raw, center, rotation_rad, FIXED_SIZE)
                
                # 4. [AI] PaDiM 판별
                if ai_input is not None and ai_input.size > 0:
                    quality, score = self.detect_anomaly(ai_input)
                else:
                    quality = "ERROR" # 크롭 실패

                # 시각화용 박스 그리기
                # OBB의 4개 꼭짓점 좌표 (xyxyxyxy format for visualization)
                if r.obb.xyxyxyxy is not None:
                     # 해당 객체의 인덱스를 찾아야 하는데, 위에서 best_obb를 찾았으니
                     # 여기서는 간단히 OBB 중심과 각도로 박스 포인트를 다시 계산해서 그립니다.
                     rect = ((cx, cy), (w, h), angle_res)
                     viz_box = cv2.boxPoints(rect).astype(int)
                     self.last_detected_box = viz_box
                
                self.last_detected_quality = quality
                self.last_detected_score = score
                self.last_detect_time = time.time()

                # 화면 표시
                color = (0, 0, 255) if quality == "DEFECT" else (0, 255, 0)
                if self.last_detected_box is not None:
                    cv2.drawContours(raw, [self.last_detected_box], 0, color, 3)
                    cv2.putText(raw, f"{quality} ({score:.1f})", 
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
