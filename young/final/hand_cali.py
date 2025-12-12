import cv2
import numpy as np
import time
from pymycobot import MyCobot320
from scipy.spatial.transform import Rotation as R

# ==========================================
# 1. 사용자 설정 (이 부분만 수정하세요)
# ==========================================
# 체커보드의 "내부 코너" 개수 (가로, 세로)
CHECKERBOARD = (8, 7) 

# 체커보드 한 칸의 실제 한 변 길이 (단위: 미터)
# 예: 2.5cm라면 0.025
SQUARE_SIZE = 0.015

# 카메라 인덱스
CAMERA_INDEX = 4

# 로봇 설정 (포트 확인 필요)
PORT = '/dev/ttyUSB0' # 윈도우라면 'COM3' 등
BAUD = 115200

# [중요] 미리 구해둔 카메라 내부 파라미터 (Camera Matrix & Distortion Coeffs)
# 캘리브레이션 결과값을 여기에 복사해서 넣으세요.
# 예시 값입니다. 본인 값으로 교체 필수!
mtx = np.array([
    [1.25038936e+03, 0.00000000e+00, 5.74939770e+02],
    [0.00000000e+00, 1.26131675e+03, 4.72721799e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

dist = np.array([
    [0.04689649, 0.54787717, 0.00547649, -0.0037721, -1.14700805]
], dtype=np.float64)


# ==========================================
# 2. 초기화 및 함수 정의
# ==========================================

# 체커보드 3D 좌표 생성 (로봇 끝단 기준, Z=0 평면)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

# 데이터 저장용 리스트
R_gripper2base = [] # 로봇: 그리퍼 -> 베이스 회전
t_gripper2base = [] # 로봇: 그리퍼 -> 베이스 이동
R_target2cam = []   # 비전: 보드 -> 카메라 회전
t_target2cam = []   # 비전: 보드 -> 카메라 이동

# 로봇 연결
try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    print(f"로봇 연결 성공: {PORT}")
except Exception as e:
    print(f"로봇 연결 실패: {e}")
    exit()

# 카메라 연결
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

def get_robot_pose_matrix(mc):
    """
    MyCobot의 현재 좌표(x,y,z,rx,ry,rz)를 읽어서
    3x3 회전행렬과 3x1 이동벡터로 변환
    """
    # 좌표 읽기 (MyCobot은 보통 mm 단위, 각도는 degree)
    coords = mc.get_coords() 
    if not coords:
        return None, None
    
    # 단위 변환: mm -> meter (ROS/비전 표준인 미터 단위로 통일 추천)
    x, y, z = coords[0]/1000.0, coords[1]/1000.0, coords[2]/1000.0
    rx, ry, rz = coords[3], coords[4], coords[5]

    # 오일러 각(Degree) -> 회전 행렬 변환
    r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
    rot_matrix = r.as_matrix()
    trans_vector = np.array([x, y, z]).reshape(3, 1)
    
    return rot_matrix, trans_vector

# ==========================================
# 3. 메인 루프 (데이터 수집)
# ==========================================
print("\n=== 사용 설명 ===")
print("1. 로봇 팔 끝에 체커보드를 붙이세요.")
print("2. 로봇을 움직여서 보드가 카메라에 잘 보이게 하세요 (다양한 각도/위치).")
print("3. 키보드 'SPACE'를 누르면 현재 위치를 캡처합니다.")
print("4. 최소 15장 이상 찍은 뒤, 'q'를 눌러 계산을 시작하세요.")
print("===============\n")

valid_captures = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 체커보드 찾기
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    display_frame = frame.copy()
    
    if ret_corners:
        # 코너 정확도 향상
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        # 화면에 그리기
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners2, ret_corners)
    
    # 상태 표시
    cv2.putText(display_frame, f"Captured: {valid_captures}", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('Hand-Eye Calibration', display_frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    # [SPACE] 키: 캡처
    if key == 32: 
        if ret_corners:
            # 1. 로봇 포즈 가져오기
            r_base, t_base = get_robot_pose_matrix(mc)
            
            if r_base is not None:
                # 2. 카메라-보드 관계 계산 (SolvePnP)
                success, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist)
                
                if success:
                    # rvec(회전벡터)를 3x3 행렬로 변환
                    r_cam, _ = cv2.Rodrigues(rvec)
                    
                    # 데이터 저장
                    R_gripper2base.append(r_base)
                    t_gripper2base.append(t_base)
                    R_target2cam.append(r_cam)
                    t_target2cam.append(tvec) # tvec은 이미 미터 단위 (SQUARE_SIZE가 미터이므로)
                    
                    valid_captures += 1
                    print(f"[{valid_captures}] 캡처 성공! 로봇위치: {t_base.T}")
                else:
                    print("SolvePnP 실패")
            else:
                print("로봇 좌표를 읽을 수 없습니다.")
        else:
            print("체커보드가 보이지 않습니다.")

    # [q] 키: 종료 및 계산
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# ==========================================
# 4. 캘리브레이션 계산
# ==========================================
if valid_captures < 5:
    print("데이터가 너무 적습니다. (최소 5개 이상 필요)")
else:
    print("\n캘리브레이션 계산 중...")
    
    # OpenCV Hand-Eye Calibration (Eye-to-Hand 방식)
    # 로봇 끝(Gripper)에 보드가 달려있고, 카메라는 고정된 경우
    # 결과: 카메라 좌표계 -> 로봇 베이스 좌표계로 가는 변환 행렬
    
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    print("\n========= [최종 결과] =========")
    print("이 행렬을 복사해서 프로젝트에 사용하세요.\n")
    
    # 보기 좋게 4x4 행렬로 합치기
    H_cam2base = np.eye(4)
    H_cam2base[:3, :3] = R_cam2base
    H_cam2base[:3, 3] = t_cam2base.T
    
    print("Transform Matrix (Camera -> Robot Base):")
    # 깔끔하게 출력
    np.set_printoptions(suppress=True, precision=6)
    print(H_cam2base)
    
    print("\n===============================")
    print(f"X 이동: {t_cam2base[0][0]:.4f} m")
    print(f"Y 이동: {t_cam2base[1][0]:.4f} m")
    print(f"Z 이동: {t_cam2base[2][0]:.4f} m (이게 카메라 높이와 비슷해야 함)")