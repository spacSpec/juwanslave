#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import DetectionResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
import torch.nn.functional as F
from torchvision.models import wide_resnet50_2, Wide_ResNet50_2_Weights
from PIL import Image as PILImage
from torchvision import transforms

# ==============================================================================
# [설정 1] 경로 및 PaDiM 파라미터
# ==============================================================================
WEIGHTS_PATH = '/home/young/final_ws/src/final/final/padim_weights/cube'
NUM_RANDOM_CHANNELS = 1500
TOP_N_PATCHES = 10 
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
ANOMALY_THRESHOLD = 170.0 

# ==============================================================================
# [설정 2] ROI 및 좌표 변환 행렬
# ==============================================================================
# ⚠️ 좌표 변환 행렬 (왜곡 보정된 점을 기준으로 맵핑한 값이어야 정확함)
PERSPECTIVE_MATRIX = np.array([
    [-0.06894,  0.13738,  21.38340],
    [ 0.22404, -1.59558, 322.14769],
    [ 0.00160, -0.00696,   1.00000]
], dtype=np.float32)

ROI_X = 420; ROI_Y = 130; ROI_W = 300; ROI_H = 350

# ==============================================================================
# [설정 3] 카메라 캘리브레이션 값 (점 좌표 보정용)
# ==============================================================================
# ⚠️ 실제 캘리브레이션 값으로 꼭 채워주세요! (아래는 예시)
CAMERA_MATRIX = np.array([
    [1000.0,    0.0, 640.0],
    [   0.0, 1000.0, 360.0],
    [   0.0,    0.0,   1.0]
])
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) 

# 색상 범위 (초록색만)
LOWER_GREEN = np.array([35, 20, 20])
UPPER_GREEN = np.array([90, 255, 255])

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.result_pub = self.create_publisher(DetectionResult, '/vision/result', 10)
        self.image_pub = self.create_publisher(Image, '/vision/defect_img', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"🚀 Vision Node Started. Using Device: {DEVICE}")

        self.load_padim_model()

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().warn("⚠️ Camera 2 failed. Switching to Camera 0...")
            self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

        self.timer = self.create_timer(0.03, self.process_frame)

    def load_padim_model(self):
        self.get_logger().info(f"🧠 Loading PaDiM weights from: {WEIGHTS_PATH}")
        try:
            if not os.path.exists(os.path.join(WEIGHTS_PATH, 'mean_vector.npy')):
                raise FileNotFoundError(f"Weight files not found at {WEIGHTS_PATH}")

            mean_vec_np = np.load(os.path.join(WEIGHTS_PATH, 'mean_vector.npy'))
            inv_cov_np = np.load(os.path.join(WEIGHTS_PATH, 'inv_cov_matrix.npy'))
            self.random_channels = np.load(os.path.join(WEIGHTS_PATH, 'random_channels.npy'))

            self.mean_vector = torch.from_numpy(mean_vec_np).to(DEVICE).float()
            self.inv_cov_matrix = torch.from_numpy(inv_cov_np).to(DEVICE).float()

            self.model = wide_resnet50_2(weights=Wide_ResNet50_2_Weights.IMAGENET1K_V2)
            self.feature_maps = {}

            def hook_fn(module, input, output, name):
                if name == 'layer3':
                    output = F.interpolate(output, size=(28, 28), mode='bilinear', align_corners=False)
                elif name == 'layer1':
                    output = F.avg_pool2d(output, kernel_size=2)
                self.feature_maps[name] = output

            self.model.layer1.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer1'))
            self.model.layer2.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer2'))
            self.model.layer3.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer3'))

            self.model.fc = torch.nn.Identity()
            self.model.eval()
            self.model.to(DEVICE)
            
            self.get_logger().info("✅ PaDiM Model Loaded & Optimized on GPU!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load PaDiM: {e}")
            self.model = None

    def extract_features(self, roi_img_rgb):
        pil_img = PILImage.fromarray(roi_img_rgb)
        transform = transforms.Compose([
            transforms.Resize(256, PILImage.LANCZOS),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        input_tensor = transform(pil_img).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            _ = self.model(input_tensor)

        combined = torch.cat([self.feature_maps['layer1'], 
                              self.feature_maps['layer2'], 
                              self.feature_maps['layer3']], dim=1)
        
        patch_features = combined.permute(0, 2, 3, 1).flatten(1, 2).squeeze(0)
        patch_features = F.normalize(patch_features, p=2, dim=1)

        return patch_features

    def detect_anomaly(self, roi_img_bgr):
        if self.model is None:
            return "ERROR", 0.0

        roi_rgb = cv2.cvtColor(roi_img_bgr, cv2.COLOR_BGR2RGB)
        features = self.extract_features(roi_rgb)
        features_reduced = features[:, self.random_channels] 

        delta = features_reduced - self.mean_vector
        temp = torch.matmul(delta, self.inv_cov_matrix)
        dist_sq = torch.sum(temp * delta, dim=1)
        dist = torch.sqrt(torch.abs(dist_sq))

        top_n_values, _ = torch.topk(dist, k=min(TOP_N_PATCHES, len(dist)))
        score = torch.mean(top_n_values).item()

        quality = "DEFECT" if score > ANOMALY_THRESHOLD else "GOOD"
        return quality, score

    def undistort_point(self, u, v):
        """
        픽셀 좌표 (u, v) 하나를 입력받아 렌즈 왜곡을 보정한 좌표 (u', v')를 반환
        """
        # 입력 형식에 맞게 변환 (1, 1, 2)
        src_pt = np.array([[[u, v]]], dtype=np.float32)
        
        # P=CAMERA_MATRIX를 넣어주지 않으면 정규화된 좌표가 나옴.
        # 우리는 다시 픽셀 좌표가 필요하므로 P에 카메라 매트릭스를 넣어줌.
        dst_pt = cv2.undistortPoints(src_pt, CAMERA_MATRIX, DIST_COEFFS, P=CAMERA_MATRIX)
        
        return dst_pt[0][0][0], dst_pt[0][0][1]

    def pixel_to_robot(self, px, py):
        """
        1. 이미지상의 점(px, py)을 받음
        2. 왜곡 보정 수행 (undistort_point)
        3. 로봇 좌표계로 변환 (perspectiveTransform)
        """
        # 1. 왜곡 보정 (렌즈 펴기)
        undistorted_x, undistorted_y = self.undistort_point(px, py)

        # 2. 로봇 좌표 변환
        pt = np.array([[[undistorted_x, undistorted_y]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, PERSPECTIVE_MATRIX)
        
        return float(dst[0][0][0]), float(dst[0][0][1])

    def apply_clahe(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        return final

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        # ----------------------------------------------------------------------
        # 1. 원본에서 바로 ROI 자르기 (속도 빠름)
        # ----------------------------------------------------------------------
        roi = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]
        roi = self.apply_clahe(roi)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2. 초록색 마스크
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        
        mask = cv2.dilate(mask, None, iterations=3) 
        mask = cv2.erode(mask, None, iterations=3) 
        mask = cv2.dilate(mask, None, iterations=1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        msg = DetectionResult()
        
        if not cnts:
            msg.is_detected = False
            self.result_pub.publish(msg)
        
        else:
            # 3. 가장 큰 물체 추출
            largest_contour = max(cnts, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) < 1000:
                msg.is_detected = False
                self.result_pub.publish(msg)
            
            else:
                c = largest_contour
                rect = cv2.minAreaRect(c)
                (cx, cy), (w, h), angle = rect
                if w < h: angle += 90

                bx, by, bw, bh = cv2.boundingRect(c)
                margin = 10
                
                x1 = max(0, bx - margin)
                y1 = max(0, by - margin)
                x2 = min(ROI_W, bx + bw + margin)
                y2 = min(ROI_H, by + bh + margin)
                
                cube_img = roi[y1:y2, x1:x2]

                if cube_img.size > 0 and cube_img.shape[0] > 20 and cube_img.shape[1] > 20:
                    quality, score = self.detect_anomaly(cube_img)
                else:
                    quality, score = "UNKNOWN", 0.0

                # --------------------------------------------------------------
                # ★ [핵심] ROI 좌표 -> 전체 좌표 -> 왜곡 보정 -> 로봇 좌표
                # --------------------------------------------------------------
                # (1) ROI 내부 중심점(cx, cy)을 전체 이미지 기준 좌표로 변환
                global_cx = cx + ROI_X
                global_cy = cy + ROI_Y

                # (2) pixel_to_robot 함수 내부에서 'undistort_point'를 수행함
                robot_x, robot_y = self.pixel_to_robot(global_cx, global_cy)

                # 메시지 전송
                msg.is_detected = True
                msg.quality = quality
                msg.center = [robot_x, robot_y+100]
                msg.angle = float(angle)
                self.result_pub.publish(msg)

                # --- 시각화 ---
                # 화면에 그릴 때는 왜곡 보정 전인 원본 이미지(frame)에 그냥 그립니다.
                box = cv2.boxPoints(rect)
                box = np.array(box, dtype=int) 
                box[:, 0] += ROI_X
                box[:, 1] += ROI_Y
                
                color = (0, 0, 255) if quality == "DEFECT" else (0, 255, 0)
                
                cv2.drawContours(frame, [box], 0, color, 3)
                
                label = f"{quality} ({score:.1f})"
                cv2.putText(frame, label, (box[1][0], box[1][1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # 로봇 좌표 표시
                coord_txt = f"X:{robot_x:.0f} Y:{robot_y:.0f}"
                cv2.putText(frame, coord_txt, (box[1][0], box[1][1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                if cube_img.size > 0: cv2.imshow("AI Input (Crop)", cube_img)

        # ROI 박스 그리기
        cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X+ROI_W, ROI_Y+ROI_H), (255, 255, 0), 2)
        cv2.imshow("Vision Eye + PaDiM", frame)
        cv2.waitKey(1)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()