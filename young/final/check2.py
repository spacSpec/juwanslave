# 파일명: calibrate_affine_integrated_final.py
# Affine 변환, Bias Correction, Z-Height 고정 및 왜곡 보정 통합 버전

import cv2
import numpy as np
import time
import sys
import tty
import termios
from pymycobot.mycobot320 import MyCobot320 

# ==============================================================================
# [설정 0] 렌즈 캘리브레이션 계수 (사용자님의 실제 계수로 교체해야 함!)
# ==============================================================================
CALIB_MATRIX_K = np.array([
    [1.25038936e+03, 0.00000000e+00, 5.74939770e+02], 
    [0.00000000e+00, 1.26131675e+03, 4.72721799e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

DIST_COEFF_D = np.array([
    [0.04689649, 0.54787717, 0.00547649, -0.0037721, -1.14700805]
], dtype=np.float32)


# ==============================================================================
# [설정 1] 로봇 및 카메라 연결 정보
# ==============================================================================
PORT = '/dev/ttyACM0' 
BAUD = 115200
CAM_INDEX = 2 
CAM_WIDTH = 1280
CAM_HEIGHT = 960

# ROI 설정
ROI_X = 350
ROI_Y = 130
ROI_W = 440
ROI_H = 350

LOWER_GREEN = np.array([35, 30, 30])
UPPER_GREEN = np.array([90, 255, 255])

# ==============================================================================
# [설정 2] 캘리브레이션 목표 좌표 (Z_pick 평면 기준)
# ⚠️ 이 값들을 실제 로봇 좌표로 반드시 변경해야 합니다.
# ==============================================================================
TARGET_ROBOT_POSITIONS = [
    [ 100.0, 150.0],  # P1: 좌상단 영역 
    [ 100.0, -150.0], # P2: 우상단 영역 
    [ 300.0, 150.0]   # P3: 좌하단 영역 
]
# 목표 좌표 정의 수정: [566] -> [X, Y]
TARGET_BIAS_CENTER = [566,315] # P_Center 목표 좌표 예시

mc = None 

# ==============================================================================
# [유틸리티 함수]
# ==============================================================================
def getch():
    """리눅스 터미널용 키 입력 함수"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_robot_coordinate_from_api():
    """
    MyCobot320에서 직접 X, Y 좌표를 읽어옵니다. (안정성 확보 로직 포함)
    """
    if mc is None:
        print("❌ 로봇 연결 객체(mc)가 초기화되지 않았습니다.")
        return None, None
        
    try:
        # 1. 좌표를 정확히 읽도록 잠시 힘을 줍니다.
        mc.power_on() 
        time.sleep(0.1) 
        
        coords = mc.get_coords()
        
        if coords and len(coords) >= 3:
            robot_x = coords[0]
            robot_y = coords[1]
            
            # 2. 좌표를 읽은 직후, 다시 수동 이동이 가능하도록 힘을 풉니다.
            mc.release_all_servos() 
            time.sleep(0.1) 

            print(f"\n--- 🤖 현재 로봇 좌표 획득: X={robot_x:.2f}, Y={robot_y:.2f}, Z={coords[2]:.2f} ---")
            return robot_x, robot_y
        else:
            print("\n❌ 로봇 좌표 읽기 실패 (데이터 불완전). 통신 상태를 확인하세요.")
            return None, None
            
    except Exception as e:
        print(f"\n❌ 로봇 통신 오류 발생: {e}")
        return None, None

def find_green_cube_center(image):
    """큐브의 중심 픽셀 좌표 (전체 화면 기준)를 찾습니다."""
    # 왜곡 보정된 이미지를 받아 ROI에서 처리
    roi = image[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 2000:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx_local = int(M["m10"] / M["m00"])
                cy_local = int(M["m01"] / M["m00"])
                # 전체 화면 기준 좌표로 변환
                cx_global = cx_local + ROI_X
                cy_global = cy_local + ROI_Y
                return (cx_global, cy_global), c 
                
    return None, None

# ==============================================================================
# [메인 루프] 캘리브레이션 실행
# ==============================================================================
def run_calibration_loop():
    global mc 
    
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 카메라 연결에 실패했습니다. 종료합니다.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    
    print("\n--- 📐 Affine 캘리브레이션 도구 실행 (Z_pick 고정 필수) ---")
    
    calibration_data = []
    current_target_index = 0
    target_pos_list = TARGET_ROBOT_POSITIONS + [TARGET_BIAS_CENTER]
    
    while cap.isOpened() and current_target_index < len(target_pos_list):
        ret, raw_frame = cap.read()
        if not ret: continue
        
        # ⚠️ [수정된 왜곡 보정 로직] raw_frame을 펴서 frame에 저장
        frame = cv2.undistort(raw_frame, CALIB_MATRIX_K, DIST_COEFF_D, None, CALIB_MATRIX_K)
        
        center_coords, contour = find_green_cube_center(frame) # 왜곡 보정된 frame 사용
        target_pos = target_pos_list[current_target_index]
        
        # 시각화
        cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (255, 0, 0), 2)

        label = "P_Center" if current_target_index >= 3 else f"P{current_target_index+1}"
        status_text = f"[{label}] Target X={target_pos[0]:.1f}, Y={target_pos[1]:.1f}"
        hint_text = "ENTER(Pixel 기록) | SPACE(Robot 기록 & 다음) | ESC(종료)"

        if center_coords is not None:
            gx, gy = center_coords
            if contour is not None:
                contour_shift = np.array([ROI_X, ROI_Y])
                cv2.drawContours(frame, [contour + contour_shift], 0, (0, 255, 0), 3)
            cv2.circle(frame, (gx, gy), 5, (0, 0, 255), -1)
        else:
            status_text += " | ❌ 큐브를 배치하세요."
        
        cv2.putText(frame, status_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(frame, hint_text, (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        cv2.imshow("Affine Calibration Tool", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if center_coords is not None:
            # 1. 큐브 픽셀 좌표 측정 (Enter 키)
            if key == 13: # Enter key
                if len(calibration_data) <= current_target_index:
                    calibration_data.append({'pixel': center_coords, 'robot': None, 'target_robot': target_pos})
                else:
                    calibration_data[current_target_index]['pixel'] = center_coords
                print(f"\n✅ {label} 픽셀 기록: {center_coords}")

            # 2. 로봇 좌표 측정 (Space bar) - 로봇 통신 호출
            elif key == 32: # Space key
                if len(calibration_data) > current_target_index and calibration_data[current_target_index]['pixel'] is not None:
                    
                    robot_coords = get_robot_coordinate_from_api() 
                    
                    if robot_coords and robot_coords[0] is not None:
                        calibration_data[current_target_index]['robot'] = robot_coords
                        print(f"✅ {label} 로봇 좌표 기록: {robot_coords}")
                        
                        # 두 데이터가 모두 기록되면 다음 점으로 이동
                        current_target_index += 1
                        print("="*50 + "\n➡️ 다음 측정 지점으로 이동합니다.")
                    
                else:
                    print("❌ 픽셀 좌표를 먼저 측정(Enter)해야 로봇 좌표를 기록할 수 있습니다.")

        # 3. 종료 (ESC 키)
        if key == 27: # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()
    
    # 최종 행렬 계산
    calculate_and_print_matrix(calibration_data)

# ==============================================================================
# [함수] 행렬 계산 및 결과 출력
# ==============================================================================
def calculate_and_print_matrix(data):
    affine_data = [d for d in data[:3] if d.get('pixel') and d.get('robot')]
    
    if len(affine_data) < 3:
        print("\n❌ Affine 행렬 계산에 필요한 최소 3쌍의 데이터가 부족합니다. 종료합니다.")
        return

    pixel_points = np.array([d['pixel'] for d in affine_data], dtype=np.float32)
    robot_points = np.array([d['robot'] for d in affine_data], dtype=np.float32)

    affine_matrix = cv2.getAffineTransform(pixel_points, robot_points)

    print("\n\n" + "="*50)
    print("✨ 최종 Affine 캘리브레이션 결과 (2x3 행렬) ✨")
    print(" vision_node.py의 AFFINE_MATRIX에 이 값을 붙여넣으세요.")
    print("="*50)
    
    print("AFFINE_MATRIX = np.array([")
    for row in affine_matrix:
        print(f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}],")
    print("], dtype=np.float32)\n")
    
    # Bias Correction 값 계산 (P_Center 데이터 사용)
    if len(data) >= 4 and data[3].get('pixel') and data[3].get('robot'):
        center_data = data[3]
        pc_pixel = np.array([center_data['pixel']], dtype=np.float32)
        
        # Affine 행렬을 이용해 중앙점 변환
        pc_calculated = cv2.transform(pc_pixel[None,:,:], affine_matrix)
        
        target_x, target_y = center_data['target_robot']
        calc_x, calc_y = pc_calculated[0][0]
        
        bias_x = target_x - calc_x
        bias_y = target_y - calc_y
        
        print("="*50)
        print("💡 Bias Correction 오프셋 (중앙점 보정값) 💡")
        print(" vision_node.py의 ROBOT_BIAS_X/Y에 이 값을 붙여넣으세요.")
        print(f"  Target Robot Center (X, Y): ({target_x:.3f}, {target_y:.3f})")
        print(f"  Calculated Center (X, Y):  ({calc_x:.3f}, {calc_y:.3f})")
        print(f"  ROBOT_BIAS_X: {bias_x:.6f}")
        print(f"  ROBOT_BIAS_Y: {bias_y:.6f}")
        print("="*50)

# ==============================================================================
# [메인 실행] 로봇 연결 및 캘리브레이션 시작
# ==============================================================================
def main():
    global mc 
    try:
        print(f"🔌 로봇 연결 시도 중... ({PORT})")
        mc = MyCobot320(PORT, BAUD)
        mc.power_on()
        time.sleep(1)
        mc.release_all_servos()
        print("✅ 로봇 연결 성공! 카메라를 준비합니다.")
        
    except Exception as e:
        print(f"❌ 로봇 연결 실패: {e}")
        sys.exit()

    try:
        run_calibration_loop()
    except KeyboardInterrupt:
        print("\n프로그램 종료.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if mc:
            mc.power_on()
            print("로봇에 다시 힘을 주어 자세를 유지합니다.")
            time.sleep(0.5)

if __name__ == "__main__":
    main()