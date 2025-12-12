import numpy as np
import cv2
import time
# 🚨 MyCobot 제어를 위한 라이브러리 임포트 (pymycobot 기준)
# pip install pymycobot
from pymycobot.mycobot320 import MyCobot320

# --- 1. 캘리브레이션 및 ROI 상수 정의 ---
CALIB_MATRIX_K = np.array([
    [1.25038936e+03, 0.00000000e+00, 5.74939770e+02],
    [0.00000000e+00, 1.26131675e+03, 4.72721799e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])
DIST_COEFF_D = np.array([
    [0.04689649, 0.54787717, 0.00547649, -0.0037721, -1.14700805]
])

# 픽존(ROI) 영역 정의 (픽셀 좌표) 및 카메라 인덱스
ROI_PIXEL_X_MIN = 250
ROI_PIXEL_Y_MIN = 250
ROI_PIXEL_X_MAX = 750
ROI_PIXEL_Y_MAX = 650
CAMERA_INDEX = 0 

# --- 2. MyCobot 제어 클래스 (실제 API 호출) ---

class MyCobotController:
    """MyCobot 320 제어 및 좌표 획득을 담당하는 클래스"""
    
    def __init__(self, port, baudrate=115200):
        try:
            # 🚨 사용자 환경에 맞는 시리얼 포트 이름으로 변경하세요 (예: '/dev/ttyACM0' 또는 'COM3')
            self.mc = MyCobot320(port, baudrate)
            self.mc.set_color(0, 0, 255) # 연결 확인용 (파란색)
            print(f"✅ MyCobot 연결 성공: {port}")
        except Exception as e:
            print(f"🚨 MyCobot 연결 실패: {e}")
            self.mc = None

    def release_for_manual(self):
        """그리퍼/토크 해제 (수동 제어 가능 상태)"""
        if self.mc:
            # 모든 서보모터 토크 해제
            self.mc.release_all_servos() 
            self.mc.set_color(255, 0, 0) # 토크 해제 시 빨간색 표시
            print("🤖 [MyCobot] 모든 서보모터 토크 해제. 로봇을 수동으로 움직일 수 있습니다.")
            return True
        return False

    def get_current_coords(self):
        """현재 로봇의 TCP 좌표 (X, Y, Z)를 가져옴"""
        if self.mc:
            # get_coords()는 [X, Y, Z, Rx, Ry, Rz] 리스트를 반환합니다.
            coords = self.mc.get_coords()
            if coords:
                # 픽앤플레이스는 X, Y 좌표만 사용합니다.
                x, y, z = coords[0], coords[1], coords[2]
                self.mc.set_color(0, 255, 0) # 좌표 획득 시 초록색 표시
                return (x, y)
        print("🚨 로봇 좌표를 가져올 수 없습니다. 연결 상태를 확인하거나 로봇이 '제어 모드'인지 확인하세요.")
        return None

# --- 3. 핵심 함수: 초록색 큐브 감지 (이전 코드와 동일) ---

def detect_green_cube(frame):
    # (코드 생략: 초록색 큐브 감지 및 ROI 필터링 로직)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            M = cv2.moments(c)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                if (ROI_PIXEL_X_MIN <= center_x <= ROI_PIXEL_X_MAX) and \
                   (ROI_PIXEL_Y_MIN <= center_y <= ROI_PIXEL_Y_MAX):
                    return (center_x, center_y)
    return None

# --- 4. 호모그래피 계산 함수 (이전 코드와 동일) ---

def calculate_homography_matrix_ransac(pixel_src_distorted, robot_dst_points):
    """제공된 K, D와 픽셀-로봇 쌍을 사용하여 H 행렬을 계산합니다."""
    if len(pixel_src_distorted) < 4 or len(pixel_src_distorted) != len(robot_dst_points):
        return None
        
    src_distorted_np = np.array(pixel_src_distorted, dtype=np.float32).reshape(-1, 1, 2)
    src_undistorted_np = cv2.undistortPoints(
        src_distorted_np, CALIB_MATRIX_K, DIST_COEFF_D, P=CALIB_MATRIX_K
    )
    src_undistorted = src_undistorted_np.reshape(-1, 2)
    dst_robot = np.float32(robot_dst_points)

    H, mask = cv2.findHomography(
        src_undistorted, dst_robot, method=cv2.RANSAC, reprojectionThreshold=1.0
    )
    
    inlier_ratio = np.sum(mask) / len(mask)
    print(f"\n--- 최종 캘리브레이션 결과 ---")
    print(f"RANSAC Inlier 비율: {inlier_ratio*100:.2f}% (최소 80% 이상 권장)")
    
    return H

# --- 5. 수동 캘리브레이션 루프 ---

def manual_calibration_loop(robot_controller):
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"🚨 오류: 카메라 인덱스 {CAMERA_INDEX}를 열 수 없습니다.")
        return

    pixel_src_distorted = []
    robot_dst_points = []
    current_pixel = None
    
    # 로봇 연결 확인
    if not robot_controller.mc:
        print("🚨 로봇이 연결되지 않아 캘리브레이션을 진행할 수 없습니다.")
        cap.release()
        cv2.destroyAllWindows()
        return

    print("\n==================================================================")
    print("      🟢 MyCobot 캘리브레이션 시작")
    print("==================================================================")
    print(" - [Enter] 키: 초록색 큐브 픽셀 좌표 저장 및 MyCobot 'Release' 명령")
    print(" - [Space] 키: 로봇 수동 이동 후 현재 로봇 좌표 'Get' 및 저장")
    print(" - [Q] 또는 [ESC] 키: 캘리브레이션 종료 및 H 행렬 계산")
    print("==================================================================")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            display_frame = frame.copy()
            cv2.rectangle(display_frame, (ROI_PIXEL_X_MIN, ROI_PIXEL_Y_MIN), 
                          (ROI_PIXEL_X_MAX, ROI_PIXEL_Y_MAX), (0, 255, 0), 2)
            
            detected_coords = detect_green_cube(frame)
            current_pixel = detected_coords
            
            if current_pixel:
                cv2.circle(display_frame, current_pixel, 10, (0, 0, 255), -1) 
                cv2.putText(display_frame, f"Pixel: {current_pixel}", (current_pixel[0] + 15, current_pixel[1] - 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.putText(display_frame, f"Pairs Collected: {len(pixel_src_distorted)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("MyCobot Calibration View - Press ENTER/SPACE/Q", display_frame)

            key = cv2.waitKey(1)
            
            if key == 13:  # Enter 키: 픽셀 좌표 저장 및 로봇 'Release'
                if current_pixel:
                    pixel_src_distorted.append(current_pixel)
                    print(f"\n--- [Enter] 1. 픽셀 좌표 저장: {current_pixel} (쌍 개수: {len(pixel_src_distorted)})")
                    robot_controller.release_for_manual()
                else:
                    print("🚨 물체가 감지되지 않았거나 ROI 밖에 있습니다. 다시 시도하세요.")
                    
            elif key == 32:  # Spacebar 키: 로봇 좌표 저장
                if len(pixel_src_distorted) > len(robot_dst_points):
                    robot_coords = robot_controller.get_current_coords()
                    
                    if robot_coords:
                        robot_dst_points.append(robot_coords[:2]) # X, Y만 저장
                        print(f"--- [Space] 2. 로봇 좌표 저장: {robot_coords[:2]} (쌍 개수: {len(robot_dst_points)})")
                        
                        if len(pixel_src_distorted) != len(robot_dst_points):
                            print("🚨 경고: 픽셀/로봇 좌표 쌍이 불균형합니다. 픽셀 좌표를 제거하여 균형을 맞춥니다.")
                            pixel_src_distorted.pop()
                            print(f"   -> 현재 쌍 개수: {len(pixel_src_distorted)}")
                        
                else:
                    print("🚨 픽셀 좌표(Enter)를 먼저 저장해야 로봇 좌표(Space)를 저장할 수 있습니다.")
            
            elif key == 113 or key == 27:  # 'q' 또는 ESC 키: 종료
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
    
    # --- 6. 최종 캘리브레이션 계산 ---
    
    if len(pixel_src_distorted) >= 4 and len(pixel_src_distorted) == len(robot_dst_points):
        H_matrix = calculate_homography_matrix_ransac(pixel_src_distorted, robot_dst_points)
        
        if H_matrix is not None:
            print("\n✅ 성공적으로 호모그래피 행렬 H가 계산되었습니다.")
            print("H 행렬:")
            print(H_matrix)
            
    else:
        print(f"\n❌ 실패: 캘리브레이션에 필요한 최소 점(4쌍) 미달 또는 데이터 불일치.")
        print(f"   -> 픽셀 쌍: {len(pixel_src_distorted)}, 로봇 쌍: {len(robot_dst_points)}")


# --- 메인 실행 ---
if __name__ == "__main__":
    # 🚨 시리얼 포트 이름을 사용자 환경에 맞게 변경하세요. (예: '/dev/ttyACM0', 'COM5', '/dev/ttyUSB0')
    ROBOT_PORT = "/dev/ttyACM0" 
    
    mycobot_controller = MyCobotController(ROBOT_PORT)
    manual_calibration_loop(mycobot_controller)