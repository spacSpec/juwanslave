#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import os

# ===== 설정값 =====
# 체스보드 내부 코너 개수 (가로, 세로)
CHESSBOARD_SIZE = (9, 6)  # (cols, rows)

# 한 칸 한 변의 실제 길이 [m]
SQUARE_SIZE = 0.024  # 예: 2.4cm

# 사용할 카메라 인덱스
CAMERA_INDEX = 0

# 결과 저장 파일명 (OpenCV FileStorage YAML, 당신이 쓰던 이름)
YAML_PATH = "/home/vboxuser/myagv_ros/yuyan.yaml"

# 캘리브레이션 이미지 저장 경로
IMAGE_SAVE_DIR = "/home/vboxuser/myagv_ros/calib_images"


def main():
    # 3D 체스보드 코너 좌표 (z=0 평면 상의 좌표)
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0],
                           0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE  # 실제 스케일 반영

    objpoints = []  # 3D 포인트들 (월드 좌표)
    imgpoints = []  # 2D 포인트들 (이미지 좌표)

    print("[CALIB] Open camera index: %d" % CAMERA_INDEX)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        return

    print("[CALIB] Start capturing.")
    print("  - 체스보드를 화면에 비치고, 코너가 잘 잡혔을 때 [Enter] 를 눌러 샘플 + 사진 저장")
    print("  - 충분히 다양한 각도/거리에서 10~20장 정도 찍는 것을 권장")
    print("  - [ESC] 를 누르면 캘리브레이션 실행 후 종료")

    # 이미지 저장 폴더 생성
    os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)

    last_gray_size = None

    while True:
        ret, frame = cap.read(0)
        if not ret:
            print("[WARN] Failed to read from camera")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        last_gray_size = gray.shape[::-1]  # (width, height)

        # 체스보드 코너 찾기
        ret_cb, corners = cv2.findChessboardCorners(
            gray, CHESSBOARD_SIZE,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                  cv2.CALIB_CB_NORMALIZE_IMAGE +
                  cv2.CALIB_CB_FAST_CHECK
        )

        display = frame.copy()

        if ret_cb:
            # 코너 위치를 더 정밀하게 다듬기
            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )

            # 체스보드 코너 시각화
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE,
                                      corners_sub, ret_cb)

            cv2.putText(
                display, "Press ENTER to capture",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
        else:
            cv2.putText(
                display, "Chessboard NOT detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )

        cv2.putText(
            display, "Samples: %d" % len(objpoints),
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
        )

        cv2.imshow("Camera Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("[CALIB] ESC pressed. Start calibration.")
            break

        # 엔터(Enter) 키: 일부 환경에서는 13 또는 10으로 들어올 수 있어서 둘 다 처리
        elif key == 13 or key == 10:
            if ret_cb:
                # 1) 캘리브레이션 샘플 추가
                objpoints.append(objp.copy())
                imgpoints.append(corners_sub.copy())

                # 2) 현재 프레임 이미지 파일로 저장
                img_idx = len(objpoints)
                img_filename = os.path.join(
                    IMAGE_SAVE_DIR, "calib_%02d.png" % img_idx
                )
                cv2.imwrite(img_filename, frame)

                print("[CALIB] Capture sample #%d -> %s" % (img_idx, img_filename))
            else:
                print("[CALIB] Chessboard not detected. Try again.")

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 3:
        print("[ERROR] Not enough samples for calibration (need >= 3, recommended 10+).")
        return

    if last_gray_size is None:
        print("[ERROR] No valid image size.")
        return

    print("[CALIB] Running calibrateCamera with %d samples..." % len(objpoints))
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, last_gray_size, None, None
    )

    print("[CALIB] RMS re-projection error: %.6f" % ret)
    print("[CALIB] camera_matrix:\n", camera_matrix)
    print("[CALIB] dist_coeffs:\n", dist_coeffs)

    # YAML로 저장 (OpenCV FileStorage 형식)
    os.makedirs(os.path.dirname(YAML_PATH), exist_ok=True)
    fs = cv2.FileStorage(YAML_PATH, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeff", dist_coeffs)
    fs.release()

    print("[CALIB] Saved calibration to: %s" % YAML_PATH)


if __name__ == "__main__":
    main()
