import cv2
import numpy as np
import os
import glob
from tqdm import tqdm  # 진행률 표시 바 (없으면 pip install tqdm)

# =========================================================
# [사용자 설정 구역] 여기만 수정하시면 됩니다!
# =========================================================

# 1. 원본 이미지가 있는 최상위 폴더 (지금 갖고 있는, 위치 제각각인 사진들)
RAW_DATA_ROOT = "/home/young/final_ws/defect" 

# 2. 전처리된 이미지를 저장할 폴더 (PaDiM 학습에 쓸 폴더)
# 이 폴더가 없으면 코드가 알아서 만듭니다.
PROCESSED_DATA_ROOT = "/home/young/final_ws/datasets/cube" 

# =========================================================

def align_and_crop(img, padding=-2):
    """
    이미지(img)를 받아 초록색 객체를 찾아 수평을 맞추고 타이트하게 잘라낸 이미지를 반환
    """
    # 1. HSV 변환 및 마스크 생성
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 노이즈 제거
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 2. 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None  # 객체 못 찾음

    # 가장 큰 덩어리 선택
    c = max(contours, key=cv2.contourArea)

    # 3. 회전된 사각형(Rotated Rect) 정보 계산
    rect = cv2.minAreaRect(c)
    (center, (w, h), angle) = rect

    # 4. 각도 보정 (가로/세로 비율에 따라 눕히기)
    if w < h:
        angle = angle + 90
        w, h = h, w

    # 5. 회전 변환
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_img = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))

    # 6. 크롭 (패딩 적용하여 배경 제거)
    crop_w = int(w) + padding
    crop_h = int(h) + padding

    if crop_w <= 0 or crop_h <= 0:
        return None

    cropped_img = cv2.getRectSubPix(rotated_img, (crop_w, crop_h), center)
    return cropped_img

def process_batch():
    # 원본 폴더 구조를 그대로 유지하면서 처리합니다.
    # 예: raw/train/good/1.jpg  -->  processed/train/good/1.jpg
    
    print(f"📂 원본 경로: {RAW_DATA_ROOT}")
    print(f"📂 저장 경로: {PROCESSED_DATA_ROOT}")
    print("-" * 50)

    # os.walk로 모든 하위 폴더 탐색
    for root, dirs, files in os.walk(RAW_DATA_ROOT):
        for file in files:
            # 이미지 파일만 골라내기
            if file.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                
                # 1. 파일 경로 설정
                src_path = os.path.join(root, file)
                
                # 2. 저장할 경로 계산 (폴더 구조 유지)
                # 원본 경로에서 root 부분을 떼어내고, 새 root를 붙임
                relative_path = os.path.relpath(root, RAW_DATA_ROOT)
                save_dir = os.path.join(PROCESSED_DATA_ROOT, relative_path)
                
                # 저장 폴더가 없으면 생성
                os.makedirs(save_dir, exist_ok=True)
                save_path = os.path.join(save_dir, file)

                # 3. 이미지 로드 및 처리
                img = cv2.imread(src_path)
                if img is None:
                    continue

                processed_img = align_and_crop(img, padding=-2)

                # 4. 결과 저장
                if processed_img is not None:
                    cv2.imwrite(save_path, processed_img)
                    # print(f"성공: {save_path}") # 너무 많이 뜨면 주석 처리
                else:
                    print(f"❌ 실패 (객체 못 찾음): {src_path}")

    print("-" * 50)
    print("✅ 모든 이미지 전처리 완료!")

if __name__ == "__main__":
    process_batch()