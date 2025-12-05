# ~/final_ws/src/final/final/calibration.py
import cv2
import numpy as np
import os
import shutil

# ==========================================
# [설정] 체커보드 교차점 개수 (가로-1, 세로-1)
# 심영주 님이 쓰시는 보드에 맞춰 수정하세요! (8, 7)이 맞는지 꼭 확인!
CHECKERBOARD = (8, 7) 
CAMERA_INDEX = 4
SAVE_DIR = "calib_imgs" # 이미지가 저장될 폴더명
# ==========================================

def main():
    # 1. 저장 폴더 만들기 (기존꺼 있으면 삭제 후 재생성 - 깔끔하게)
    if os.path.exists(SAVE_DIR):
        shutil.rmtree(SAVE_DIR)
    os.makedirs(SAVE_DIR)
    print(f"📂 이미지는 '{SAVE_DIR}' 폴더에 자동 저장됩니다.")

    # 3D 점 준비
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints = [] # 3D points
    imgpoints = [] # 2D points

       # 2. 카메라 설정
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"⚠️ {CAMERA_INDEX}번 실패 -> 0번 시도")
        cap = cv2.VideoCapture(2)

    # ===================================
    # 📌 원본 화각 유지(디지털 줌/크롭 방지)
    # ===================================
    # 압축 포맷 강제 (YUYV는 종종 자동 크롭됨)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

    # 센서가 제공하는 최대 원본 해상도
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    # ===================================
    # 📌 디지털 줌(Zoom/Pan/Tilt) 기본값 강제
    #     ※ 지원 안 하는 웹캠이면 무시됨(문제 없음)
    # ===================================
    try:
        cap.set(cv2.CAP_PROP_ZOOM, 1)      # 디지털 줌 1.0
        cap.set(cv2.CAP_PROP_PAN, 0)       # 좌/우 이동 원위치
        cap.set(cv2.CAP_PROP_TILT, 0)      # 상/하 이동 원위치
    except:
        pass

    print("\n카메라 설정 적용됨:")
    print(" - MJPG 포맷")
    print(" - 1280x720 원본 센서")
    print(" - 디지털 줌/크롭 방지 적용")


    count = 0
    last_frame = None

    while True:
        ret, frame = cap.read()
        if not ret: break
        last_frame = frame.copy() # 마지막 프레임 보관용
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_corn, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        # 화면에 그리기 (저장은 원본으로 함)
        vis_frame = frame.copy()
        if ret_corn:
            cv2.drawChessboardCorners(vis_frame, CHECKERBOARD, corners, ret_corn)
            cv2.putText(vis_frame, "READY (Press 'c')", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Calibration Shot', vis_frame)
        key = cv2.waitKey(1)

        if key == ord('c'):
            if ret_corn:
                objpoints.append(objp)
                imgpoints.append(corners)
                count += 1
                
                # [NEW] 이미지 파일로 저장!
                filename = f"{SAVE_DIR}/img_{count:02d}.jpg"
                cv2.imwrite(filename, frame) 
                print(f"📸 [{count}장] 저장 완료 -> {filename}")
            else:
                print("❌ 인식 실패! 각도를 조절하세요.")
        
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if count < 10:
        print("⚠️ 이미지가 너무 적습니다. (최소 10장 권장)")
        return

    # 3. 계산 시작
    print("\n🧮 행렬 계산 중... (잠시만 기다리세요)")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # ================= [NEW] 정확도(에러율) 계산 =================
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    total_error = mean_error / len(objpoints)
    
    print("\n" + "="*50)
    print(f"📊 캘리브레이션 정확도 (Re-projection Error): {total_error:.4f}")
    print("   - 0.1 ~ 0.3 : 신의 경지 (완벽)")
    print("   - 0.3 ~ 0.7 : 아주 훌륭함 (Good)")
    print("   - 1.0 이상  : 재촬영 추천 (Bad)")
    print("="*50)

    print("\n✅ 결과 행렬 (vision_node.py에 복사하세요):")
    print("-" * 30)
    print("CALIB_MATRIX_K = np.array(" + np.array2string(mtx, separator=', ') + ")")
    print("DIST_COEFF_D   = np.array(" + np.array2string(dist, separator=', ') + ")")
    print("-" * 30)

    # 4. [NEW] 왜곡 보정 테스트 (눈으로 확인)
    print("\n👀 왜곡 보정(Undistort) 테스트 화면을 띄웁니다. (아무 키나 누르면 종료)")
    
    # 마지막 찍은 사진으로 테스트
    if last_frame is not None:
        h, w = last_frame.shape[:2]
        # 최적의 새 카메라 행렬 계산 (검은 테두리 제거)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        
        # 왜곡 펴기
        dst = cv2.undistort(last_frame, mtx, dist, None, newcameramtx)
        
        # 비교를 위해 나란히 배치
        compare = np.hstack((last_frame, dst))
        cv2.imshow('Before (Left) vs After (Right)', compare)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()