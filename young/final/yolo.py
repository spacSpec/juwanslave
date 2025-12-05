import cv2
import os

# ==========================================
# [설정] 사용자 정의 ROI 및 저장 경로
# ==========================================
SAVE_DIR = "dataset_roi_crop"  # 저장될 폴더명
WIDTH = 1280                   # 카메라 해상도 (가로)
HEIGHT = 960                   # 카메라 해상도 (세로)

# 사용자님이 정하신 ROI 좌표 및 크기
ROI_X = 350
ROI_Y = 130
ROI_W = 440
ROI_H = 350

# 폴더가 없으면 생성
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)
    print(f"폴더 생성 완료: {SAVE_DIR}")

# 웹캠 연결
cap = cv2.VideoCapture(2) # 0번이 안되면 1번으로 변경
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# 저장된 파일 카운트 확인
count = len(os.listdir(SAVE_DIR))

print("=============================================")
print(f"  [ROI 크롭] 데이터 수집 시작")
print(f"  ROI 영역: x={ROI_X}, y={ROI_Y}, w={ROI_W}, h={ROI_H}")
print("  [SPACE] 키: 크롭된 이미지 저장")
print("  [   q   ] 키: 종료")
print("=============================================")

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 오류!")
        break

    # 1. ROI 영역 잘라내기 (이게 실제로 저장되고 학습될 이미지)
    # 파이썬 배열 슬라이싱: [세로(y), 가로(x)] 순서 주의!
    cropped_frame = frame[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]

    # 2. 사용자 확인용 전체 화면 (녹색 박스 그리기)
    display_frame = frame.copy()
    cv2.rectangle(display_frame, 
                  (ROI_X, ROI_Y), 
                  (ROI_X + ROI_W, ROI_Y + ROI_H), 
                  (0, 255, 0), 2)  # 녹색 박스, 두께 2

    # 화면 띄우기
    cv2.imshow("1. Full View (Green Box is ROI)", display_frame)
    cv2.imshow("2. Saved View (Cropped)", cropped_frame)

    key = cv2.waitKey(1)

    # 스페이스바 누르면 'cropped_frame'을 저장
    if key == 32:
        filename = f"{SAVE_DIR}/crop_{count}.jpg"
        
        # 중요: 전체 화면(frame)이 아니라 잘린 화면(cropped_frame)을 저장함
        cv2.imwrite(filename, cropped_frame)
        
        print(f"[저장 완료] {filename} (크기: {ROI_W}x{ROI_H})")
        count += 1
    
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()