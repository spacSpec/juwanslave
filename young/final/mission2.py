# -*- coding: utf-8 -*-
"""
MyCobot 320 M5 + OpenCV 색상 감지 + 각도 추정 + 색상별 적재
[v4.0 FINAL] 로봇 이동 완전 안정화 버전
------------------------------------------------------------
- 모든 send_coords/send_angles 후 wait_for_robot_stop 강제 적용
- 상대 좌표 모드 제거 (1 → 0)
- 조인트 회전 안정화
------------------------------------------------------------
"""

import time, argparse, threading, cv2, numpy as np

try:
    from pymycobot.mycobot320 import MyCobot320 as CobotClass
except Exception:
    from pymycobot.mycobot import MyCobot as CobotClass



# ===================== 포즈 설정 =====================
POSE_HOME   = [-228.8, -78.7, 332.0, -176.36, -3.68, -178.24]
POSE_CLEAR1  = [-254.4, -17.4, 350.6, -178.78, 15.16, 1.6]
POSE_CLEAR2 =  [-90.1, 181.2, 347.1, -175.45, 39.27, -97.12]     
 
# 색상별 배치 포즈
POSE_SET_GREEN =  [-212.6, 187.6, 293.4, -166.97, -1.34, 37.04]
POSE_PLACE_GREEN =  [-275.2, 163.5, 177.6, -175.24, -6.24, 29.08]

POSE_SET_BLUE  =   [-84.9, 207.0, 384.5, -151.28, -1.27, 2.44]
POSE_PLACE_BLUE =  [-77.5, 319.8, 174.9, 178.88, -2.45, 3.99]

POSE_SET_RED   = [66.0, 182.2, 386.3, -162.77, -5.47, -44.77]
POSE_PLACE_RED = [163.0, 273.6, 165.9, -179.08, -1.18, -34.29]

DEFAULT_SPEED = 15
MOVE_SPEED = 15

# ===================== 쌓기 설정 =====================
STACK_HEIGHT = 30
stack_count_green = 0
stack_count_blue = 0
stack_count_red = 0


# ===================== 보정 파라미터 =====================
SCALE_X = 0.35
SCALE_Y = 0.36
OFFSET_X = -5.0
OFFSET_Y = -75.0


# ===================== 카메라 내부 보정 =====================
K = np.array([
    [539.1372906745268, 0.0, 329.02126025840977],
    [0.0, 542.3421738705956, 242.1099554052592],
    [0.0, 0.0, 1.0]
])
D = np.array([[0.20528603028454656, -0.766640680691422,
               -0.0009661402178902956, 0.0011189160210831846,
               0.9763000357883636]])



# ===================== 픽셀 → 로봇 변환 =====================
def pixel_to_robot(cx, cy, frame_w, frame_h):
    dx = (cx - frame_w / 2) * SCALE_X
    dy = (cy - frame_h / 2) * SCALE_Y
    robot_x = POSE_HOME[0] + OFFSET_X + dx
    robot_y = POSE_HOME[1] + OFFSET_Y - dy
    robot_z = POSE_HOME[2]
    print(f"[DEBUG] pixel→robot: ({cx:.1f},{cy:.1f}) → ({robot_x:.1f},{robot_y:.1f})")
    return [robot_x, robot_y, robot_z, POSE_HOME[3], POSE_HOME[4], POSE_HOME[5]]



# ===================== 로봇 동작 안정 대기 함수 =====================
# ===================== 로봇 동작 안정 대기 함수 (수정됨) =====================
def wait_for_robot_stop(mc, pos_tol=0.3, ang_tol=0.3, stable_time=0.30, timeout=20):
    """
    로봇이 일정 시간 동안 움직임이 거의 없어야 '정지 완료'로 판정
    """
    print("⏳ 로봇 이동 완료 대기 중...")

    start = time.time()
    still_since = None
    last = mc.get_coords()

    while True:
        time.sleep(0.1)
        now = mc.get_coords()

        # [수정된 부분] 데이터가 없거나, 리스트가 아니면(숫자 -1 등) 무시하고 다시 시도
        if not isinstance(now, list) or not isinstance(last, list):
            # 만약 now는 정상인데 last가 이상하면, last를 갱신하고 다음 턴으로
            if isinstance(now, list):
                last = now
            # 반대로 now가 이상하면 그냥 무시
            continue

        # 이제 둘 다 리스트인게 확실하므로 계산 수행
        try:
            pos_diff = max(abs(now[i] - last[i]) for i in range(3))
            ang_diff = max(abs(now[i] - last[i]) for i in range(3, 6))
        except Exception:
            continue

        if pos_diff < pos_tol and ang_diff < ang_tol:
            if still_since is None:
                still_since = time.time()
            elif time.time() - still_since >= stable_time:
                print("✅ 로봇 정지 완료")
                return
        else:
            still_since = None

        last = now

        if time.time() - start > timeout:
            print("⚠️ wait_for_robot_stop 타임아웃 (강제 진행)")
            return


# ===================== 색상 감지 (OpenCV) =====================
def detect_color_and_angle(frame):



    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    COLOR_RANGES = {
        "red1": ([0, 100, 50], [10, 255, 255]),
        "red2": ([170, 100, 50], [180, 255, 255]),
        "green": ([35, 40, 40], [70, 255, 255]),
        "blue": ([100, 120, 100], [130, 255, 255])
    }

    masks = {}
    kernel_open = np.ones((5,5), np.uint8)
    kernel_close = np.ones((10,10), np.uint8)

    sat_mask = cv2.inRange(hsv[:,:,1], 80, 255)
    val_mask = cv2.inRange(hsv[:,:,2], 50, 255)

    combined_mask = cv2.bitwise_and(sat_mask, val_mask)

    red_mask = None
    for key, (low, up) in COLOR_RANGES.items():
        m = cv2.inRange(hsv, np.array(low), np.array(up))
        m = cv2.bitwise_and(m, combined_mask)
        if key == "red1":
            red_mask = m
        elif key == "red2":
            red_mask = cv2.bitwise_or(red_mask, m)
            m = red_mask
            masks["red"] = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel_open)
            masks["red"] = cv2.morphologyEx(masks["red"], cv2.MORPH_CLOSE, kernel_close)
        else:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel_open)
            masks[key] = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel_close)

    best_cnt, detected_color, max_area = None, None, 0
    area_threshold = 1500

    for cname, mask in masks.items():
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            a = cv2.contourArea(c)
            if a > area_threshold and a > max_area:
                max_area = a
                best_cnt = c
                detected_color = cname

    if best_cnt is None:
        return frame, None, 0, None, None

    rect = cv2.minAreaRect(best_cnt)
    (cx, cy), (rw, rh), angle = rect

    if rw < rh:
        angle += 0
    angle = abs(angle)

    box = cv2.boxPoints(rect)
    box = np.intp(box)

    color_vis = (0,0,255)
    if detected_color == "green": color_vis = (0,255,0)
    elif detected_color == "blue": color_vis = (255,0,0)

    cv2.drawContours(frame, [box], 0, color_vis, 2)
    cv2.circle(frame, (int(cx),int(cy)), 5, (0,255,255), -1)

    return frame, detected_color, angle, int(cx), int(cy)



# ===================== 카메라 스레드 =====================
def camera_thread(stop_event, frame_container):
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("❌ 카메라 열기 실패")
        return

    cap.set(3, 640); cap.set(4, 480)
    w, h = 640, 480
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w,h), 1, (w,h))
    mx, my = cv2.initUndistortRectifyMap(K, D, None, new_K, (w,h), 5)

    while not stop_event.is_set():
        ret, frame = cap.read()
        if ret:
            frame_container["frame"] = cv2.remap(frame, mx, my, cv2.INTER_LINEAR)
        time.sleep(0.03)
    cap.release()



# ===================== 메인 루틴 =====================
def main():
    global stack_count_green, stack_count_blue, stack_count_red

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    mc = CobotClass(args.port, args.baud)
    mc.power_on()
    time.sleep(1)

    # 초기화
    mc.send_angles([0,0,0,0,0,0], 20)
    wait_for_robot_stop(mc)

    # mc.send_coords(POSE_CLEAR1, DEFAULT_SPEED, 0)
    # wait_for_robot_stop(mc)

    mc.send_coords(POSE_HOME, DEFAULT_SPEED, 0)
    wait_for_robot_stop(mc)

    mc.set_gripper_mode(0)
    mc.set_electric_gripper(0)
    mc.set_gripper_value(100,20,1)
    print("🏠 초기화 완료")



    # ==================== 9회 반복 ====================
    for i in range(9):
        print(f"\n--- 🔁 사이클 {i+1}/9 ---")

        frame_container, stop_event = {"frame": None}, threading.Event()
        cam_thread = threading.Thread(target=camera_thread, args=(stop_event, frame_container), daemon=True)
        cam_thread.start()

        detect_start = None
        detected_color = None

        # ------------------ 색상 감지 루프 ------------------
        while not stop_event.is_set():
            frame = frame_container.get("frame")
            if frame is None:
                continue

            vis, color, angle, cx, cy = detect_color_and_angle(frame)

            if color:
                if detect_start is None:
                    detect_start = time.time()
                elif time.time() - detect_start > 4:
                    print(f"🎯 색상 확정: {color.upper()}")
                    detected_color = color
                    detected_angle = angle
                    detected_coords = pixel_to_robot(cx, cy, frame.shape[1], frame.shape[0])
                    stop_event.set()
                    break
            else:
                detect_start = None

            cv2.imshow("Camera", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

        cam_thread.join()
        cv2.destroyAllWindows()

        if detected_color is None:
            print("❌ 감지 실패 → 다음 사이클")
            continue



        # ==================== 픽업 단계 ====================
        x, y, z, r, p, yaw = detected_coords

        # 1) XY 이동
        mc.send_coords([x, y, 345, r, p, yaw], 25, 0)
        wait_for_robot_stop(mc)

        # 2) J6 회전 보정
        ang = mc.get_angles()
        if ang:
            ang[5] += detected_angle
            mc.send_angles(ang, 25)
            wait_for_robot_stop(mc)

        # 3) 하강
        cur = mc.get_coords()
        cur[2] = 300
        mc.send_coords(cur, 20, 0)
        wait_for_robot_stop(mc)

        # 4) 집기
        mc.set_gripper_value(40, 30, 1)
        time.sleep(1)

        # 5) 상승
        cur = mc.get_coords()
        cur[2] = 345
        mc.send_coords(cur, 25, 0)
        wait_for_robot_stop(mc)

        # 6) CLEAR 포즈 이동
        mc.send_coords(POSE_CLEAR1, DEFAULT_SPEED, 0)
        wait_for_robot_stop(mc)



        # ==================== 색상별 분류 & 쌓기 ====================
        if detected_color == "green":
            base_set = POSE_SET_GREEN.copy()       # SET 포즈는 고정
            base_place = POSE_PLACE_GREEN.copy()
            z_offset = (STACK_HEIGHT+2) * stack_count_green
            stack_count_green += 1

        elif detected_color == "blue":
            base_set = POSE_SET_BLUE.copy()
            base_place = POSE_PLACE_BLUE.copy()
            z_offset = STACK_HEIGHT * stack_count_blue
            stack_count_blue += 1

        else:  # red
            base_set = POSE_SET_RED.copy()
            base_place = POSE_PLACE_RED.copy()
            z_offset = (STACK_HEIGHT+3) * stack_count_red
            stack_count_red += 1

        # ❗ SET 포즈는 변경 금지
        # base_set[2] += z_offset   # ← 제거!

        # ✔ PLACE 포즈에만 적용
        base_place[2] += z_offset



        # SET 포즈
        mc.send_coords(base_set, MOVE_SPEED, 0)
        wait_for_robot_stop(mc)

        # PLACE 포즈
        mc.send_coords(base_place, DEFAULT_SPEED, 0)
        wait_for_robot_stop(mc)

        # 내려놓기
        mc.set_gripper_value(80, 20, 1)
        time.sleep(1)

        # SET 포즈로 복귀
        mc.send_coords(base_set, MOVE_SPEED, 0)
        wait_for_robot_stop(mc)

        # 홈 복귀
        mc.send_coords(POSE_CLEAR1, MOVE_SPEED, 0)
        wait_for_robot_stop(mc)

        mc.send_coords(POSE_HOME, MOVE_SPEED, 0)
        wait_for_robot_stop(mc)

        print(f"✅ 사이클 {i+1}/9 완료")



    print("🎉 모든 작업 완료!")




if __name__ == "__main__":
    main()