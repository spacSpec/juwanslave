#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger 
from my_robot_interfaces.msg import DetectionResult 
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
import torch.nn.functional as F
from torchvision.models import wide_resnet50_2, Wide_ResNet50_2_Weights
import time 
from PIL import Image as PILImage
from torchvision import transforms

# ==============================================================================
# [설정] 경로 및 파라미터 
# ==============================================================================
WEIGHTS_PATH = '/home/young/final_ws/src/final/final/padim_weights/cube'
DEFECT_IMAGE_SAVE_DIR = '/home/young/final_ws/src/final/defect_images' 

NUM_RANDOM_CHANNELS = 300 
TOP_N_PATCHES = 10 
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
ANOMALY_THRESHOLD = 90.0 
# [✅ 안정화된 ID 경로 사용]
TARGET_DEVICE_ID = '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0'

ROI_X = 350; ROI_Y = 130; ROI_W = 440; ROI_H = 350
LOWER_GREEN = np.array([35, 40, 40])
UPPER_GREEN = np.array([85, 255, 255])
PERSPECTIVE_MATRIX = np.array([
    [-0.07023947, -0.34752808,  67.7946950],
    [-0.57473692,  0.08202037, 298.1217640],
    [-0.00094684,  0.00030250,   1.0000000]
], dtype=np.float32)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.TOP_N_PATCHES = TOP_N_PATCHES
        self.ANOMALY_THRESHOLD = ANOMALY_THRESHOLD
        self.DEVICE = DEVICE
        self.is_camera_open = False

        # 1. ROS 통신 설정
        self.result_pub = self.create_publisher(DetectionResult, '/vision/result', 10) 
        self.image_pub = self.create_publisher(RosImage, '/vision/defect_img', 10) 
        self.detect_srv = self.create_service(
            Trigger,  
            '/vision/start_detection', 
            self.handle_detection_request
        )

        if not os.path.exists(DEFECT_IMAGE_SAVE_DIR):
            os.makedirs(DEFECT_IMAGE_SAVE_DIR)
        
        self.bridge = CvBridge()
        self.get_logger().info(f"🚀 Vision Node Started. Using Device: {self.DEVICE}")

        # 2. AI 모델 로드
        self.transform = transforms.Compose([
            transforms.Resize((224, 224), PILImage.LANCZOS),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        self.load_padim_model()
        
        # 3. 카메라 설정
        self.setup_camera()
        
        self.get_logger().info("✅ Vision Node Ready. Waiting for Task Manager request...")

    # ========================== Helper Functions ==========================
    
    def find_camera_index(self, device_path):
        """[✅ 안정성 강화 로직] 고유 ID 경로에서 실제 인덱스를 안정적으로 추출"""
        # 1. 경로가 존재하는지 확인
        if not os.path.exists(device_path): 
            self.get_logger().error(f"❌ Device ID path does not exist: {device_path}")
            return None
        
        # 2. 심볼릭 링크를 따라 실제 경로 (예: /dev/video2)를 찾음
        try:
            real_path = os.path.realpath(device_path)
            
            # 3. 경로가 /dev/videoN 형식인지 확인 후 인덱스 추출
            if 'video' in real_path and real_path.startswith('/dev/video'):
                index_str = real_path.split('video')[-1]
                return int(index_str)
            else:
                self.get_logger().error(f"❌ Real path is not a video device: {real_path}")
                return None
        except Exception as e:
            self.get_logger().error(f"❌ Error during index resolution: {e}")
            return None
            
    def setup_camera(self):
        """[✅ 복구된 로직] 인덱스 고정을 제거하고 고유 ID를 사용"""
        # 고유 ID를 통해 카메라 인덱스를 찾습니다.
        camera_index = self.find_camera_index(TARGET_DEVICE_ID)
        
        # 인덱스를 찾지 못하면 0번을 시도합니다.
        if camera_index is None:
            camera_index = 0
            self.get_logger().warn(f"⚠️ Failed to resolve ID. Switching to default Camera {camera_index}...")

        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
            self.is_camera_open = True
            self.get_logger().info(f"✅ Camera Index {camera_index} Opened Successfully.")
        else:
            self.get_logger().error(f"🔴 Failed to open Camera {camera_index}. Check permissions and device availability."); 
            self.is_camera_open = False
    
    def load_padim_model(self):
        self.get_logger().info(f"🧠 Loading PaDiM weights from: {WEIGHTS_PATH}")
        try:
            if not os.path.exists(os.path.join(WEIGHTS_PATH, 'mean_vector.npy')): raise FileNotFoundError("Weight files missing.")
            mean_vec_np = np.load(os.path.join(WEIGHTS_PATH, 'mean_vector.npy')); inv_cov_np = np.load(os.path.join(WEIGHTS_PATH, 'inv_cov_matrix.npy')); self.random_channels = np.load(os.path.join(WEIGHTS_PATH, 'random_channels.npy'))
            self.mean_vector = torch.from_numpy(mean_vec_np).to(self.DEVICE).float(); self.inv_cov_matrix = torch.from_numpy(inv_cov_np).to(self.DEVICE).float()
            self.model = wide_resnet50_2(weights=Wide_ResNet50_2_Weights.IMAGENET1K_V2); self.feature_maps = {}
            def hook_fn(module, input, output, name):
                if name == 'layer3': output = F.interpolate(output, size=(28, 28), mode='bilinear', align_corners=False)
                elif name == 'layer1': output = F.avg_pool2d(output, kernel_size=2)
                self.feature_maps[name] = output
            self.model.layer1.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer1'))
            self.model.layer2.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer2'))
            self.model.layer3.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer3'))
            self.model.fc = torch.nn.Identity(); self.model.eval(); self.model.to(self.DEVICE)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load PaDiM: {e}"); self.model = None

    def extract_features(self, img_bgr):
        roi_img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB); pil_img = PILImage.fromarray(roi_img_rgb); input_tensor = self.transform(pil_img).unsqueeze(0).to(self.DEVICE)
        with torch.no_grad(): _ = self.model(input_tensor)
        combined = torch.cat([self.feature_maps['layer1'], self.feature_maps['layer2'], self.feature_maps['layer3']], dim=1)
        patch_features = combined.permute(0, 2, 3, 1).flatten(1, 2).squeeze(0); return patch_features 

    def detect_anomaly(self, img_bgr):
        if self.model is None: return "ERROR", 0.0
        features_full = self.extract_features(img_bgr); features_reduced = features_full[:, self.random_channels]
        delta = features_reduced - self.mean_vector; temp = torch.matmul(delta, self.inv_cov_matrix); dist_sq = torch.sum(temp * delta, dim=1)
        dist = torch.sqrt(torch.abs(dist_sq)); top_n_values, _ = torch.topk(dist, k=min(self.TOP_N_PATCHES, len(dist)))
        score = torch.mean(top_n_values).item(); quality = "DEFECT" if score > self.ANOMALY_THRESHOLD else "GOOD"; return quality, score

    def align_and_crop(self, src_img, contour, padding=-2):
        rect = cv2.minAreaRect(contour); (center, (w, h), angle) = rect
        if w < h: angle = angle + 90; w, h = h, w
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_img = cv2.warpAffine(src_img, M, (src_img.shape[1], src_img.shape[0]))
        crop_w = int(w) + padding; crop_h = int(h) + padding
        if crop_w <= 0 or crop_h <= 0: return None
        cropped = cv2.getRectSubPix(rotated_img, (crop_w, crop_h), center); return cropped

    def pixel_to_robot(self, px, py):
        pt = np.array([[[px, py]]], dtype=np.float32); dst = cv2.perspectiveTransform(pt, PERSPECTIVE_MATRIX)
        return float(dst[0][0][0]), float(dst[0][0][1])

    def save_defect_image_to_file(self, image_bgr):
        """불량 판정 시 이미지를 로컬 파일 시스템에 저장합니다."""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"defect_{timestamp}.jpg"
            filepath = os.path.join(DEFECT_IMAGE_SAVE_DIR, filename)
            cv2.imwrite(filepath, image_bgr)
            self.get_logger().info(f"✅ Image Save Success: Saved to {filepath}")
        except Exception as e:
            self.get_logger().error(f"❌ Image Save Failed: {e}")
            
    def handle_detection_request(self, request, response):
        """[동기] Task Manager 요청 시 분석을 수행하고 결과를 즉시 응답합니다."""
        self.get_logger().info("▶️ Service Call: Starting AI analysis...")

        if not self.is_camera_open:
            response.success = False; response.message = "CAMERA_UNAVAILABLE"; return response

        ret, raw = self.cap.read()
        if not ret: response.success = False; response.message = "FRAME_READ_FAIL"; return response
        
        # 1. ROI 추출 및 색상 마스크 생성
        roi = raw[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        msg = DetectionResult()
        quality = "NO_OBJECT" 

        if cnts:
            target_cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(target_cnt) > 2000:
                raw_rect = cv2.minAreaRect(target_cnt); (cx, cy), _, angle = raw_rect
                ai_input_img = self.align_and_crop(roi, target_cnt, padding=-2)

                # 2. AI 분석
                if ai_input_img is not None and ai_input_img.size > 0:
                    quality, score = self.detect_anomaly(ai_input_img)
                else: quality, score = "ERROR", 0.0

                # 3. 로봇 좌표 계산 및 시각화
                global_cx = cx + ROI_X; global_cy = cy + ROI_Y
                robot_x, robot_y = self.pixel_to_robot(global_cx, global_cy)

                msg.is_detected = True; msg.quality = quality; msg.center = [robot_x, robot_y]; msg.angle = float(angle)
                
                box = cv2.boxPoints(raw_rect); box = np.array(box, dtype=int)
                box[:, 0] += ROI_X; box[:, 1] += ROI_Y
                color = (0, 0, 255) if quality == "DEFECT" else (0, 255, 0)
                cv2.drawContours(raw, [box], 0, color, 3)
                label = f"{quality} ({score:.1f})"; cv2.putText(raw, label, (box[1][0], box[1][1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            else: quality = "NO_OBJECT"
        
        cv2.rectangle(raw, (ROI_X, ROI_Y), (ROI_X+ROI_W, ROI_Y+ROI_H), (255, 255, 0), 2)
        
        # =======================================================
        # [✅ 핵심] 디버깅 시각화 창
        # =======================================================
        try:
            cv2.imshow("1. Raw Image (Result)", raw)
            cv2.imshow("2. ROI Image", roi) 
            cv2.imshow("3. HSV Mask (White = Detected)", mask) 
            cv2.waitKey(1) 
        except Exception as e:
            # GUI 환경(X server)이 없는 곳에서 실행되면 이 에러가 발생할 수 있습니다.
            self.get_logger().error(f"❌ OpenCV Display Error (Check X server / GUI): {e}")
        # =======================================================
        
        # 4. 토픽 발행 
        self.result_pub.publish(msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(raw, "bgr8"))

        # 5. 불량품 판정 시 로컬 파일에 저장
        if quality == "DEFECT":
            self.get_logger().warn("🔴 DEFECT DETECTED. Saving image to local file...")
            self.save_defect_image_to_file(raw) 
            
        # 6. Task Manager에 동기적으로 응답
        response.success = True if quality in ["GOOD", "DEFECT"] else False
        response.message = quality 

        self.get_logger().info(f"✅ AI Analysis complete. Responding: {quality}")
        return response
        
    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened(): self.cap.release()
        cv2.destroyAllWindows() 

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()