import cv2
import numpy as np

class RobotCameraCalibrator:
    def __init__(self, camera_matrix, dist_coeffs):
        self.camera_matrix = np.array(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.camera_position = None

    def calibrate(self, pixel_points, robot_points):
        """
        N개의 점(권장 6개 이상)을 이용해 카메라 위치 계산
        """
        pixel_points = np.array(pixel_points, dtype=np.float64)
        robot_points = np.array(robot_points, dtype=np.float64)

        # [핵심 변경] SOLVEPNP_EPNP 사용
        # 이 모드는 점이 4개여도, 6개여도, 100개여도 에러 없이 가장 강건(Robust)하게 계산합니다.
        success, rvec, tvec = cv2.solvePnP(
            robot_points, 
            pixel_points, 
            self.camera_matrix, 
            self.dist_coeffs, 
            flags=cv2.SOLVEPNP_EPNP
        )
        
        if not success:
            raise Exception("캘리브레이션 실패! 데이터 점검 필요")

        self.rvec = rvec
        self.tvec = tvec
        self.rotation_matrix, _ = cv2.Rodrigues(rvec)
        self.camera_position = -np.dot(self.rotation_matrix.T, tvec)
        
        print("=== 캘리브레이션 완료 ===")
        print(f"카메라 위치 (Robot Frame): \n{self.camera_position.ravel()}")
        print(f"데이터 점 개수: {len(pixel_points)}개")
        print("-" * 30)

    def pixel_to_robot(self, u, v, target_z):
        if self.rvec is None:
            raise Exception("먼저 calibrate()를 실행해주세요.")

        # 1. 픽셀 -> 언디스토션 -> 정규 좌표
        uv_point = np.array([[[u, v]]], dtype=np.float64)
        undistorted_point = cv2.undistortPoints(uv_point, self.camera_matrix, self.dist_coeffs)
        x_norm = undistorted_point[0][0][0]
        y_norm = undistorted_point[0][0][1]
        
        # 2. 광선 벡터(Ray) 생성 및 월드 좌표 변환
        ray_camera = np.array([x_norm, y_norm, 1.0]).reshape(3, 1)
        ray_world = np.dot(self.rotation_matrix.T, ray_camera)
        cam_pos = self.camera_position

        # 3. Ray Casting (평면 교차점 계산)
        if abs(ray_world[2]) < 1e-6:
             return 0, 0, 0 # 에러 방지용 리턴

        s = (target_z - cam_pos[2]) / ray_world[2]
        target_x = cam_pos[0] + s * ray_world[0]
        target_y = cam_pos[1] + s * ray_world[1]
        
        return float(target_x), float(target_y), float(target_z)

# ==========================================
# [사용자 데이터 입력]
# ==========================================

W, H = 1280, 960
# 파라미터는 님께서 구하신 정확한 값 사용
camera_mtx = np.array([
    [1.25038936e+03, 0.00000000e+00, 5.74939770e+02],
    [0.00000000e+00, 1.26131675e+03, 4.72721799e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])
dist_coef = np.array([
    [0.04689649, 0.54787717, 0.00547649, -0.0037721, -1.14700805]
])

# -------------------------------------------------------------
# [중요] 점 6개 입력 (기존 4개 + 추가 2개)
# -------------------------------------------------------------
# 추가 추천 위치:
# 점 5: 중앙 좌측 (ex: x좌표 중간, y좌표 위쪽)
# 점 6: 중앙 우측 (ex: x좌표 중간, y좌표 아래쪽)

pixel_data = [
    [435, 162], # 1. 좌상
    [564, 120], # 2. 우상
    [692, 370], # 3. 우하
    [483, 408], # 4. 좌하
    [460, 285], # <--- 5. l 점 (직접 입력하세요)
    [670, 250]  # <--- 6. r (직접 입력하세요)
]

robot_data = [
    [62.4, 284.4, 299.8],  # 1. 좌상
    [53.0, 229.9, 294.5],  # 2. 우상
    [-43.8, 232.1, 298.2], # 3. 우하
    [-42.6, 287.7, 303.3], # 4. 좌하
    [6.8, 292.8, 303.7],   # <--- 5. 추가 점 로봇 좌표 (직접 입력하세요)
    [6.8, 197.4, 301.1]    # <--- 6. 추가 점 로봇 좌표 (직접 입력하세요)
]

# -------------------------------------------------------------
# 실행
# -------------------------------------------------------------
calibrator = RobotCameraCalibrator(camera_mtx, dist_coef)
calibrator.calibrate(pixel_data, robot_data)

# -------------------------------------------------------------
# [테스트 시 주의사항]
# -------------------------------------------------------------
# 현재 로봇 좌표의 Z값이 약 300mm입니다. (컨베이어 높이)
# 물건을 잡으려면 '컨베이어 높이 + 물건 두께'를 target_z로 줘야 합니다.
# 예: 큐브 두께가 30mm라면 -> target_z = 300 + 30 = 330mm 이어야 함.
# 단순히 60을 넣으면 로봇이 바닥(Z=0) 근처인 60mm로 내려가려다 쾅 박을 수 있습니다!!

conveyor_z_avg = 300.0 # 대략적인 컨베이어 높이
object_thickness = 50.0 # 물체 두께
target_picking_z = conveyor_z_avg + object_thickness 

target_u, target_v = 640, 480 # 화면 중앙

final_x, final_y, final_z = calibrator.pixel_to_robot(target_u, target_v, target_picking_z)

print(f"\n[결과] 픽셀({target_u}, {target_v}) -> 로봇 좌표")
print(f"X: {final_x:.2f}, Y: {final_y:.2f}, Z: {final_z:.2f}")