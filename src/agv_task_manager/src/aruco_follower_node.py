#!/usr/bin/env python3
# coding=utf-8

import cv2
import cv2.aruco as aruco
import numpy as np
import math
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8   # ✅ cmd_val용

# =========================
# 설정 값 (필요 시 수정)
# =========================

CAMERA_YAML_PATH = "/home/vboxuser/myagv_ros/src/agv_task_manager/config/yuyan.yaml"

# ArUco 마커 한 변 길이 [m]
# ⚠ 실제 마커 전체 한 변 길이를 m 단위로 넣어야 함 (예: 3cm => 0.03)
# 3.2cm
MARKER_LENGTH_M = 0.032

# 추적할 마커 ID
TARGET_IDS = [5]

# 거리/각도 제어 파라미터
DIST_OFFSET_CM = 0.0        # 카메라 위치 보정 (원래 스크립트에서 -5cm 사용)
DIST_TARGET_CM = 5.0        # 이 거리 이내면 도착으로 판단 [cm]
ANGLE_DEADBAND_DEG = 5.0     # 이 각도 이내면 회전 대신 직진 허용 각도범위
FORWARD_SPEED = 0.08         # 전진 속도 [m/s]
MAX_ANGULAR_SPEED = 0.6      # 회전 속도 제한 [rad/s]
K_ANGLE = 0.02               # 각도 비례 제어 계수 (deg -> rad/s)

# 거리 이동 평균용 윈도우 크기
DISTANCE_WINDOW = 5

# 화면 표시 여부 (AGV(headless)에서는 False, PC에서 돌릴 때 True)
SHOW_WINDOW = True


def get_aruco_dict():
    """OpenCV 버전에 따라 적절한 딕셔너리 생성."""
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(aruco.DICT_6X6_1000)
    elif hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    else:
        raise RuntimeError(
            "현재 설치된 OpenCV(cv2.aruco)에는 Dictionary_get / getPredefinedDictionary가 없습니다.\n"
            "opencv-contrib-python 패키지를 확인하세요."
        )


def get_detector_params():
    """버전에 따라 DetectorParameters 생성."""
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    elif hasattr(aruco, "DetectorParameters"):
        try:
            return aruco.DetectorParameters()
        except TypeError:
            return None
    else:
        return None


def load_camera_params(yaml_path):
    """카메라 파라미터(yaml) 로드."""
    cv_file = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not cv_file.isOpened():
        raise RuntimeError("Failed to open camera yaml: %s" % yaml_path)

    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()
    cv_file.release()

    if camera_matrix is None or dist_matrix is None:
        raise RuntimeError("camera_matrix/dist_coeff not found in yaml: %s" % yaml_path)

    return camera_matrix, dist_matrix


def detect_best_marker(frame, camera_matrix, dist_matrix, target_ids):
    """
    가장 가까운(또는 첫 번째) 타겟 마커 하나 선택해서 정보 반환.
    여기서는 estimatePoseSingleMarkers가 없어서 solvePnP로 직접 pose 추정.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = get_aruco_dict()
    params = get_detector_params()

    # 1) 마커 검출
    corners = ids = rejected = None

    if hasattr(aruco, "ArucoDetector"):
        # 새 API
        try:
            if params is not None:
                detector = aruco.ArucoDetector(aruco_dict, params)
            else:
                detector = aruco.ArucoDetector(aruco_dict)
            corners, ids, rejected = detector.detectMarkers(gray)
        except Exception as e:
            rospy.logerr_throttle(5.0, "[ARUCO] ArucoDetector.detectMarkers error: %s", str(e))
            return None

    elif hasattr(aruco, "detectMarkers"):
        # 옛 API
        try:
            if params is None:
                corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)
            else:
                corners, ids, rejected = aruco.detectMarkers(
                    gray, aruco_dict, parameters=params
                )
        except Exception as e:
            rospy.logerr_throttle(5.0, "[ARUCO] aruco.detectMarkers error: %s", str(e))
            return None

    else:
        rospy.logerr_throttle(
            5.0,
            "[ARUCO] cv2.aruco 에 detectMarkers / ArucoDetector 가 모두 없습니다. "
            "현재 OpenCV 빌드에서는 ArUco를 사용할 수 없습니다."
        )
        return None

    if ids is None or len(ids) == 0:
        return None

    # 2) solvePnP로 pose 추정
    objp = np.array([
        [0.0,              0.0,               0.0],
        [MARKER_LENGTH_M,  0.0,               0.0],
        [MARKER_LENGTH_M,  MARKER_LENGTH_M,   0.0],
        [0.0,              MARKER_LENGTH_M,   0.0]
    ], dtype=np.float32)

    h, w = frame.shape[:2]

    best_info = None
    best_dist = None
    rvecs_draw = []
    tvecs_draw = []

    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id not in target_ids:
            continue

        img_pts = corners[i][0].astype(np.float32)  # (4,2)

        ok, rvec, tvec = cv2.solvePnP(
            objp, img_pts, camera_matrix, dist_matrix, flags=cv2.SOLVEPNP_IPPE_SQUARE
        ) if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.solvePnP(
            objp, img_pts, camera_matrix, dist_matrix
        )

        if not ok:
            continue

        rvecs_draw.append(rvec)
        tvecs_draw.append(tvec)

        tx = float(tvec[0])
        tz = float(tvec[2])

        # 카메라 Z축 거리 -> cm + 오프셋
        distance_cm = tz * 100.0 + DIST_OFFSET_CM

        # 좌우 각도 (deg)
        angle_rad = math.atan2(tx, tz)
        angle_deg = math.degrees(angle_rad)

        # 화면상 중심 비율 [0,1]
        cx = float(np.mean(img_pts[:, 0]))
        perc_x = cx / float(w)

        if (best_dist is None) or (distance_cm < best_dist):
            best_dist = distance_cm
            best_info = {
                "id": marker_id,
                "distance_cm": distance_cm,
                "angle_deg": angle_deg,
                "perc_x": perc_x,
            }

    # 시각화 (옵션)
    if SHOW_WINDOW:
        if hasattr(aruco, "drawDetectedMarkers") and corners is not None and ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        if hasattr(aruco, "drawAxis") and len(rvecs_draw) == len(tvecs_draw) and len(rvecs_draw) > 0:
            for rvec, tvec in zip(rvecs_draw, tvecs_draw):
                aruco.drawAxis(frame, camera_matrix, dist_matrix, rvec, tvec, 0.03)
        elif hasattr(cv2, "drawFrameAxes") and len(rvecs_draw) == len(tvecs_draw) and len(rvecs_draw) > 0:
            for rvec, tvec in zip(rvecs_draw, tvecs_draw):
                cv2.drawFrameAxes(frame, camera_matrix, dist_matrix, rvec, tvec, 0.03, 2)

    return best_info


def make_cmd_from_marker(info, distance_buffer, arrived):
    """
    마커 정보(거리/각도)를 받아서 /cmd_vel Twist와 상태코드(mode)를 생성.
    mode:
      0 = 마커 없음(정지)
      1 = 회전 중
      2 = 전진 중
      3 = 목표 도착
    """
    twist = Twist()
    mode = 0  # 기본: 아무 것도 안 함

    # 마커가 안 보이면 정지
    if info is None:
        return twist, arrived, mode

    distance_cm = info["distance_cm"]
    angle_deg = info["angle_deg"]

    distance_buffer.append(distance_cm)
    filt_distance = float(np.mean(distance_buffer))

    # 이미 도착 상태라면 정지 + mode=3 유지
    if arrived:
        mode = 3
        return twist, arrived, mode

    # 1) 거리 기준: 목표 거리 이내면 도착
    if filt_distance <= DIST_TARGET_CM:
        rospy.loginfo("[ARUCO] Target reached: ~%.1f cm" % filt_distance)
        arrived = True
        mode = 3
        return twist, arrived, mode

    # 2) 각도 기준: 먼저 각도 맞추기
    if abs(angle_deg) > ANGLE_DEADBAND_DEG:
        angular_z = -K_ANGLE * angle_deg

        if angular_z > MAX_ANGULAR_SPEED:
            angular_z = MAX_ANGULAR_SPEED
        elif angular_z < -MAX_ANGULAR_SPEED:
            angular_z = -MAX_ANGULAR_SPEED

        twist.angular.z = angular_z
        twist.linear.x = 0.0

        rospy.loginfo("[ARUCO] Rotating: angle=%.1f deg, cmd_az=%.2f",
                      angle_deg, angular_z)
        mode = 1
    else:
        # 각도가 충분히 맞으면 전진
        twist.linear.x = FORWARD_SPEED
        twist.angular.z = 0.0
        rospy.loginfo("[ARUCO] Forward: dist=%.1f cm" % filt_distance)
        mode = 2

    return twist, arrived, mode


class ArucoFollowerCompressed(object):
    """ /camera/image/compressed 를 구독해서 /cmd_vel /cmd_val 을 내보내는 노드 """

    def __init__(self):
        rospy.init_node("aruco_follower_compressed", anonymous=False)

        rospy.loginfo("[ARUCO] Loading camera parameters from: %s", CAMERA_YAML_PATH)
        self.camera_matrix, self.dist_matrix = load_camera_params(CAMERA_YAML_PATH)
        rospy.loginfo("[ARUCO] Camera params loaded.")

        # ✅ 속도 명령
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # ✅ 상태 코드(cmd_val)
        self.state_pub = rospy.Publisher("/cmd_val", Int8, queue_size=10)

        self.distance_buffer = deque(maxlen=DISTANCE_WINDOW)
        self.arrived = False

        self.sub = rospy.Subscriber(
            "/camera/image/compressed",
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("[ARUCO] Node started. Tracking IDs: %s", TARGET_IDS)
        rospy.loginfo("[ARUCO] Using /camera/image/compressed as input.")

    def image_callback(self, msg: CompressedImage):
        # JPEG -> OpenCV BGR
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(2.0, "[ARUCO] cv2.imdecode() returned None")
            return

        # 1) 마커 검출
        info = detect_best_marker(frame, self.camera_matrix, self.dist_matrix, TARGET_IDS)

        # 2) 상태 텍스트 구성
        if info is not None:
            status_text = "ID={} D={:.1f}cm A={:.1f}deg".format(
                info["id"], info["distance_cm"], info["angle_deg"]
            )
        else:
            status_text = "No marker"

        # 3) 제어 명령 + 상태 코드 생성
        twist, self.arrived, mode = make_cmd_from_marker(
            info, self.distance_buffer, self.arrived
        )

        # 4) cmd_vel / cmd_val 발행
        self.cmd_pub.publish(twist)

        state_msg = Int8()
        state_msg.data = mode
        self.state_pub.publish(state_msg)

        # 5) 화면 표시 (옵션)
        if SHOW_WINDOW:
            cv2.putText(
                frame, status_text, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 0), 2, cv2.LINE_AA
            )
            cv2.imshow("aruco_follower_view", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rospy.loginfo("ESC pressed in viewer. Shutting down node.")
                rospy.signal_shutdown("ESC pressed in viewer")
                return

    def spin(self):
        rospy.spin()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = ArucoFollowerCompressed()
        node.spin()
    except rospy.ROSInterruptException:
        pass
