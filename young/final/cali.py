# 파일명: calibrate_affine.py

import cv2
import numpy as np
import math

# ==============================================================================
# [설정 1] 카메라 및 ROI 설정 (vision_node.py와 동일하게 설정)
# ==============================================================================
CAM_INDEX = 2 # vision_node.py의 self.cap = cv2.VideoCapture(2)와 동일하게 설정
CAM_WIDTH = 1280
CAM_HEIGHT = 960

# ROI 설정 (vision_node.py와 동일)
ROI_X = 350
ROI_Y = 130
ROI_W = 440
ROI_H = 350

# HSV 색상 범위 (vision_node.py와 동일)
LOWER_GREEN = np.array([35, 30, 30])
UPPER_GREEN = np.array([90, 255, 255])

# ==============================================================================
# [설정 2] 캘리브레이션 데이터 저장 변수
# ==============================================================================
# 3쌍의 (픽셀, 로봇) 좌표를 저장할 리스트
calibration_data = []

# 캘리브레이션 데이터 측정용 로봇 좌표 (수동으로 변경해야 함)
# 사용자가 직접 로봇 컨트롤러에서 확인한 실제 mm 값을 여기에 입력합니다.
# P1(좌상), P2(우상), P3(좌하)의 실제 로봇 좌표(Z_pick 평면 기준)
TARGET_ROBOT_POSITIONS = [
    # [X, Y] (mm)
    [ 100.0, 150.0],  # P1: 좌상단 영역 (예시값)
    [ 100.0, -150.0], # P2: 우상단 영역 (예시값)
    [ 300.0, 150.0]   # P3: 좌하단 영역 (예시값)
]
# Bias Correction용 중앙점 (추가 측정)
TARGET_BIAS_CENTER = [200.0, 0.0] # P_Center (예시값)

# ==============================================================================
# [함수] 초록색 큐브 중심 좌표 찾기
# ==============================================================================
def find_green_cube_center(image):
    # 1. ROI 자르기
    roi = image[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]
    
    # 2. 이미지 처리
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 3. 윤곽선 검출
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 2000: # 최소 면적 필터
            M = cv2.moments(c)
            if M["m00"] != 0:
                # ROI 내부 픽셀 좌표
                cx_local = int(M["m10"] / M["m00"])
                cy_local = int(M["m01"] / M["m00"])
                
                # 전체 화면 기준 좌표로 복원
                cx_global = cx_local + ROI_X
                cy_global = cy_local + ROI_Y
                
                return (cx_global, cy_global), c # 중심 좌표와 윤곽선 반환
                
    return None, None

# ==============================================================================
# [메인 루프] 캘리브레이션 실행
# ==============================================================================
def run_calibration():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    
    print("--- Affine 캘리브레이션 도구 실행 ---")
    print(f"현재 목표 측정 횟수: 3회 (Affine 행렬용) + 1회 (Bias용)")
    print("---------------------------------")
    
    current_target_index = 0
    
    while cap.isOpened() and current_target_index < len(TARGET_ROBOT_POSITIONS) + 1:
        ret, frame = cap.read()
        if not ret: continue
        
        # 큐브 중심 찾기
        center_coords, contour = find_green_cube_center(frame)
        
        # 시각화: ROI 영역 표시
        cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (255, 0, 0), 2)
        
        if center_coords is not None:
            gx, gy = center_coords
            
            # 큐브 윤곽선 및 중심점 표시
            cv2.drawContours(frame, [contour + np.array([ROI_X, ROI_Y])], 0, (0, 255, 0), 3)
            cv2.circle(frame, (gx, gy), 5, (0, 0, 255), -1)
            
            # 현재 상태 표시
            if current_target_index < len(TARGET_ROBOT_POSITIONS):
                target_pos = TARGET_ROBOT_POSITIONS[current_target_index]
                status_text = f"P{current_target_index+1}: Target X={target_pos[0]:.1f}, Y={target_pos[1]:.1f}"
                hint_text = "엔터(Pixel) | 스페이스(Robot)"
            else:
                target_pos = TARGET_BIAS_CENTER
                status_text = f"P_Center: Target X={target_pos[0]:.1f}, Y={target_pos[1]:.1f}"
                hint_text = "엔터(Pixel) | 스페이스(Robot) | ESC(완료)"

            cv2.putText(frame, status_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame, hint_text, (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        else:
            cv2.putText(frame, "Cube NOT FOUND!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow("Affine Calibration Tool", frame)
        
        # 키 입력 대기
        key = cv2.waitKey(1) & 0xFF
        
        if center_coords is not None:
            # 1. 큐브 픽셀 좌표 측정 (엔터 키)
            if key == 13: # Enter key
                if len(calibration_data) <= current_target_index:
                    calibration_data.append({'pixel': center_coords, 'robot': None, 'target_robot': target_pos})
                    print(f"✅ P{current_target_index+1} 픽셀 기록: {center_coords}")
                else:
                    calibration_data[current_target_index]['pixel'] = center_coords
                    print(f"🔄 P{current_target_index+1} 픽셀 업데이트: {center_coords}")

            # 2. 로봇 좌표 측정 (스페이스 바)
            elif key == 32: # Space bar
                if len(calibration_data) > current_target_index and calibration_data[current_target_index]['pixel'] is not None:
                    # 사용자 입력: 실제 로봇 좌표를 수동으로 입력받음
                    print("\n" + "="*50)
                    print(f"P{current_target_index+1} ({'Center' if current_target_index >= 3 else ''})의 현재 로봇 좌표를 입력하세요 (Target: {target_pos})")
                    try:
                        robot_x = float(input("Robot X (mm): "))
                        robot_y = float(input("Robot Y (mm): "))
                        
                        calibration_data[current_target_index]['robot'] = (robot_x, robot_y)
                        print(f"✅ P{current_target_index+1} 로봇 좌표 기록: ({robot_x}, {robot_y})")
                        
                        # 두 데이터가 모두 기록되면 다음 점으로 이동
                        if calibration_data[current_target_index]['robot'] is not None:
                            current_target_index += 1
                            print("➡️ 다음 측정 지점으로 이동합니다.")
                            print("="*50 + "\n")
                    except ValueError:
                        print("❌ 유효한 숫자를 입력하세요. 다시 시도합니다.")
                else:
                    print("❌ 픽셀 좌표를 먼저 측정(Enter)해야 로봇 좌표를 기록할 수 있습니다.")

        # 3. 완료 및 종료 (ESC 키)
        if key == 27 and current_target_index >= 3: # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()
    
    # 캘리브레이션 결과 계산
    calculate_and_print_matrix(calibration_data)

# ==============================================================================
# [함수] 행렬 계산 및 결과 출력
# ==============================================================================
def calculate_and_print_matrix(data):
    if len(data) < 3:
        print("\n❌ Affine 행렬 계산에 필요한 최소 3쌍의 데이터가 부족합니다.")
        return

    # Affine 행렬 생성에 필요한 3쌍의 데이터
    pixel_points = np.array([d['pixel'] for d in data[:3]], dtype=np.float32)
    robot_points = np.array([d['robot'] for d in data[:3]], dtype=np.float32)

    # Affine 행렬 계산
    affine_matrix = cv2.getAffineTransform(pixel_points, robot_points)

    print("\n\n" + "="*50)
    print("✨ 최종 Affine 캘리브레이션 결과 (2x3 행렬) ✨")
    print("="*50)
    
    # vision_node.py에 붙여넣기 쉬운 형태로 출력
    print("AFFINE_MATRIX = np.array([")
    for row in affine_matrix:
        print(f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}],")
    print("], dtype=np.float32)\n")
    
    # Bias Correction 값 계산 (P_Center 데이터 사용)
    if len(data) >= 4:
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
        print(f"  Target Robot Center (X, Y): ({target_x:.3f}, {target_y:.3f})")
        print(f"  Calculated Center (X, Y):  ({calc_x:.3f}, {calc_y:.3f})")
        print(f"  ROBOT_BIAS_X: {bias_x:.6f}")
        print(f"  ROBOT_BIAS_Y: {bias_y:.6f}")
        print("="*50)
        print("이 Bias 값을 vision_node.py에 추가하여 최종 오차를 보정하세요.")

if __name__ == "__main__":
    run_calibration()