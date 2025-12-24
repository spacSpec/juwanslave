#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, threading, tf
from std_msgs.msg import Bool, Int32, Int8
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID 
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Imu

from db_agv import insert_agv_battery

class AgvTaskManager:
    def __init__(self):
        rospy.init_node('agv_task_manager', anonymous=False)
        rospy.loginfo("🚀 AGV Task Manager: Full Task & Battery Logic Integrated")

        # --- TF 리스너 초기화 (연산 대기용) ---
        self.tf_listener = tf.TransformListener()

        # --- 상태 변수 ---
        self.stage = "IDLE"
        self.is_paused = False
        self.forklift_done_flag = False
        self.current_yaw = 0.0
        self.battery_level = 100 

        self.battery_level = 100 
        self.last_logged_batt = None  # 추가: DB 중복 기록 방지용

        # --- 목적지 좌표 ---
        self.pos_door_in = (7.98, -2.71, 15.86)     
        self.pos_door_out_forward = (9.89, -2.16, 16.03) 
        self.pos_door_out_return = (9.89, -2.16, -163.03) 
        self.pos_final = (9.28, -1.21, 98.04)       
        self.pos_home = (6.37, -1.18, 109.66)
        self.pos_batt_final = (10.30, -0.22, 16.06)       
        self.pos_after_unload = (9.87, -1.55, 108.16) 

        # --- Subscriber (최우선 등록) ---
        rospy.Subscriber('/battery_percent', Int8, self.cb_battery)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.cb_result)
        rospy.Subscriber('/imu/data_filtered', Imu, self.cb_imu)
        
        # --- Publisher ---
        self.pub_forklift1 = rospy.Publisher("/forklift1_cmd", Bool, queue_size=1)
        self.pub_forklift2 = rospy.Publisher("/forklift2_cmd", Bool, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # --- Service & 나머지 Subs ---
        rospy.Service("/agv/request_dispatch", SetBool, self.handle_srv)
        rospy.Subscriber('/agv/fence_open', Bool, self.cb_fence_open)
        rospy.Subscriber('/agv/door_open', Bool, self.cb_door_open)
        rospy.Subscriber('/forklift_done', Bool, self.cb_forklift_done)
        rospy.Subscriber('/qr_task_done', Bool, self.cb_qr_done)

        self.qr_task_srv = None
        threading.Thread(target=self.connect_qr_service, daemon=True).start()

        rospy.sleep(1.0)
        rospy.loginfo("🏗️ 초기화: 지게차 상승 시작")
        self.pub_forklift1.publish(Bool(data=True))
        rospy.sleep(2.0)
        rospy.loginfo("✅ 시스템 준비 완료")

        # __init__ 함수 마지막 부분에 추가
        threading.Thread(target=self.monitor_system, daemon=True).start()

    def connect_qr_service(self):
        try:
            rospy.wait_for_service('/qr_task', timeout=1.0)
            self.qr_task_srv = rospy.ServiceProxy('/qr_task', SetBool)
        except: pass

    # ===============================
    # 🔋 배터리 로직 (중복 명령 방지)
    # ===============================
    def cb_battery(self, msg):
        self.battery_level = msg.data

        if self.last_logged_batt is None or self.battery_level != self.last_logged_batt:
            # 별도 스레드로 실행하여 로봇 동작에 방해를 주지 않음
            threading.Thread(target=insert_agv_battery, args=(self.battery_level,)).start()
            self.last_logged_batt = self.battery_level
        
        # IDLE 상태에서만 배터리 부족 시 충전소 이동 시작
        if self.stage == "IDLE" and self.battery_level <= 10:
            rospy.logwarn(f"🚨 배터리 부족({self.battery_level}%): 충전소 이동 시작")
            self.stage = "BATT_GO_DOOR_INNER"
            self.send_goal(*self.pos_door_in)
        
        # 충전 중(IDLE_CHARGING)에 배터리가 차면 복귀
        elif self.stage == "IDLE_CHARGING" and self.battery_level > 10:
            rospy.loginfo(f"🔋 충전 완료({self.battery_level}%): 복귀 시작")
            self.stage = "BATT_RET_GO_OUT" 
            self.send_goal(*self.pos_door_out_return)

    # [수정] TF 연산 대기 함수 (공용)
    def wait_for_transform(self, target="base_footprint", source="imu_link", timeout=0.1):
        """ TF 연산이 준비될 때까지 기다립니다. 실패 시 AGV 정지 유도. """
        try:
            self.tf_listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(timeout))
            return True
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(2, f"⏳ TF 연산 대기 중... ({source} -> {target})")
            return False

    # ===============================
    # 🎯 메인 사이클 제어 (도착 결과)
    # ===============================
    def cb_result(self, msg):
        if self.is_paused or msg.status.status != 3: return
        rospy.loginfo(f"🎯 도착 완료: {self.stage}")

        # 1. 문 안쪽 도착 (일반/충전 공통)
        if self.stage in ["GO_DOOR_INNER", "BATT_GO_DOOR_INNER"]:
            self.stage = "QR_DOOR_INNER" if self.stage == "GO_DOOR_INNER" else "BATT_QR_DOOR_INNER"
            self._call_qr_task_async(target_id=4)

        # 2. 문 밖으로 전진 완료
        elif self.stage == "GO_DOOR_OUT":
            self.stage = "GO_FINAL"; self.send_goal(*self.pos_final)
        
        # 3. 배터리 충전 전용 문 밖 전진 완료
        elif self.stage == "BATT_GO_DOOR_OUT":
            self.stage = "BATT_GO_FINAL"; self.send_goal(*self.pos_batt_final)

        # 4. 일반 하역지 도착
        elif self.stage == "GO_FINAL":
            self.stage = "QR_FINAL"; self._call_qr_task_async(target_id=6)

        # 5. 충전 스테이션 도착
        elif self.stage == "BATT_GO_FINAL":
            self.stage = "BATT_QR_FINAL"; self._call_qr_task_async(target_id=2)

        # [수정] 중간 좌표 도착 시 빈 박스 가지러가기
        elif self.stage == "GO_AFTER_UNLOAD":
            rospy.loginfo("🎯 중간 좌표 도착. 2번 QR 접근 및 적재 준비를 시작합니다.")
            self.stage = "QR_RELOAD_TASK" # 새로운 상태명: 재적재 작업
            self._call_qr_task_async(target_id=2) # 2번 QR 호출

        # 6. 하역 완료 후 복귀 좌표 도착 (일반/충전후 공통)
        elif self.stage in ["GO_DOOR_OUT_RETURN", "BATT_RET_GO_OUT"]:
            self.stage = "QR_DOOR_OUT_RETURN"; self._call_qr_task_async(target_id=3)

        # 7. 집(Home) 좌표 도착 시 실행되는 새로운 시퀀스
        elif self.stage == "GO_HOME":
            rospy.loginfo("🏁 홈 좌표 도착. 후속 작업을 시작합니다.")
            self.stage = "HOME_SEQUENCING"
            # 별도 스레드에서 시퀀스 실행 (블로킹 방지)
            threading.Thread(target=self.run_home_sequence, daemon=True).start()

        # 8. ★ 추가: 최종 정렬 위치 도착 시 IDLE 전환
        elif self.stage == "FINISHING_HOME":
            rospy.loginfo("🏁 최종 정렬 완료. IDLE 상태로 전환합니다.")
            self.stage = "IDLE"

    def run_home_sequence(self):
        # 0. 우회전
        self.turn_right_teleop(0.5, 2.0)
        self.smart_sleep(0.5)

        # 1. QR 인식 호출
        rospy.loginfo("🏠 [1/6] QR 인식 시작")
        self.stage = "QR_HOME_WAITING" # 상태명 변경 (구분용)
        self._call_qr_task_async(target_id=32)
        
        # QR 완료 대기 (cb_qr_done에서 stage를 바꿔줄 때까지 기다림)
        while self.stage == "QR_HOME_WAITING" and not rospy.is_shutdown():
            rospy.sleep(0.1)


        # 물건 내리기
        self.pub_forklift1.publish(Bool(data=False))
        self.wait_for_forklift()
        self.smart_sleep(3.0)
        # self.move_forward_teleop(0.1, 2.0)
        self.move_backward_imu(0.1, 2.0); 

        # 리프트 업
        self.pub_forklift1.publish(Bool(data=True))
        self.wait_for_forklift()
        self.smart_sleep(3.0)


        # # 2. 지게차 다운
        # rospy.loginfo("🏠 [2/6] 지게차 다운")
        # self.pub_forklift1.publish(Bool(data=False))
        # self.wait_for_forklift()
        # self.smart_sleep(3.0)
        # # 3. 2초 전진
        # rospy.loginfo("🏠 [3/6] 2초 전진")
        # self.move_forward_teleop(0.1, 2.0)
        # self.smart_sleep(0.5)

        # # 4. 2초 후진
        # rospy.loginfo("🏠 [4/6] 2초 후진")
        # self.move_backward_imu(0.1, 2.0)
        # self.smart_sleep(0.5)

        # # 5. 지게차 업
        # rospy.loginfo("🏠 [5/6] 지게차 업")
        # self.pub_forklift1.publish(Bool(data=True))
        # self.wait_for_forklift()
        # self.smart_sleep(3.0)

        # 6. 최종 HOME 정렬 위치 이동
        rospy.loginfo("🏠 [6/6] 최종 정렬 위치로 이동")
        self.stage = "FINISHING_HOME" # 이 상태가 cb_result에서 IDLE을 만듭니다.
        self.send_goal(6.21, -1.07, -70.0)

    # ===============================
    # ✅ QR 완료 시점 제어
    # ===============================
    def cb_qr_done(self, msg):
        if not msg.data or self.is_paused: return
        rospy.loginfo(f"✅ QR 완료 신호 수신 (현재 상태: {self.stage})")

        # [재적재 분기] 2번 QR 접근 완료 시 적재 시퀀스 실행
        if self.stage == "QR_RELOAD_TASK":
            rospy.loginfo("🏗️ 2번 QR 접근 성공. 빈 박스 적재 시퀀스를 시작합니다.")
            self.stage = "FORKLIFT_RELOADING_PROCESS" 
            threading.Thread(target=self.run_forklift_sequence, daemon=True).start()
            return

        # 1. 홈 복귀 시퀀스 (이미 작성하신 run_home_sequence 루프 탈출용)
        if self.stage == "QR_HOME_WAITING":
            rospy.loginfo("🏠 홈 QR 인식 완료")
            self.stage = "QR_HOME_DONE"
            return # 중요: 처리 완료 후 즉시 리턴
        


        # 2. 적재 시퀀스 시작
        elif self.stage == "QR_LOAD":
            rospy.loginfo("🏗️ 적재 프로세스(Thread) 시작")
            self.stage = "FORKLIFT_LOADING_PROCESS" 
            threading.Thread(target=self.run_forklift_sequence, daemon=True).start()
            return

        # 3. 하역 시퀀스 시작
        elif self.stage == "QR_FINAL":
            rospy.loginfo("🏗️ 하역 프로세스(Thread) 시작")
            self.stage = "FORKLIFT_FINAL_PROCESS"
            threading.Thread(target=self.run_final_forklift_sequence, daemon=True).start()
            return

        # 4. 배터리 충전소 도착
        elif self.stage == "BATT_QR_FINAL":
            self.stage = "IDLE_CHARGING"
            rospy.loginfo("⏳ 충전 대기 모드 진입")
            return

        # 5. 문/게이트 대기 단계 전환
        elif self.stage == "QR_DOOR_INNER": 
            self.stage = "WAIT_DOOR_OUT"
            return
        elif self.stage == "BATT_QR_DOOR_INNER": 
            self.stage = "BATT_WAIT_DOOR_OUT"
            return
        elif self.stage == "QR_DOOR_OUT_RETURN": 
            self.stage = "WAIT_DOOR_IN"
            return

    # ===============================
    # 🚪 문 열림 처리
    # ===============================
    def cb_door_open(self, msg):
        if not msg.data or self.is_paused: return
        
        if self.stage == "WAIT_DOOR_OUT":
            self.stage = "GO_DOOR_OUT"; self.send_goal(*self.pos_door_out_forward)
        elif self.stage == "BATT_WAIT_DOOR_OUT":
            self.stage = "BATT_GO_DOOR_OUT"; self.send_goal(*self.pos_door_out_forward)
        elif self.stage == "WAIT_DOOR_IN":
            self.stage = "GO_HOME"; self.send_goal(*self.pos_home)

    def cb_forklift_done(self, msg):
        if not msg.data or self.is_paused: return
        self.forklift_done_flag = True
        rospy.loginfo(f"🏗️ 지게차 작업 완료! 현재 상태: {self.stage}")

    # 2. spin()은 아무것도 건드리지 말고 대기만 합니다.
    def spin(self):
        rospy.loginfo("🏁 AGV Task Manager 스핀 시작")
        rospy.spin()

    def wait_for_forklift(self, timeout=30.0): # 지게차 동작 시간을 고려해 타임아웃 넉넉히
        self.forklift_done_flag = False 
        start_time = rospy.get_time()
        
        while not self.forklift_done_flag and not rospy.is_shutdown():
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("⚠️ 지게차 신호 대기 타임아웃! 다음 동작으로 강제 진행합니다.")
                break
                
            if self.is_paused:
                self.wait_for_resume()
                # 일시정지 후 복귀했을 때 타임아웃 시간을 보정하고 싶다면 start_time을 업데이트 하세요.
                
            rospy.sleep(0.1)
            
        self.forklift_done_flag = False # 다음 사용을 위해 초기화
        rospy.loginfo("✅ 지게차 동작 완료 확인됨.")

    def run_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 적재 시작")
        self.pub_forklift1.publish(Bool(data=False))
        self.wait_for_forklift()
        self.smart_sleep(3.0)
        self.move_forward_teleop(0.1, 2.2)
        self.smart_sleep(0.5)
        self.pub_forklift1.publish(Bool(data=True))
        self.wait_for_forklift()
        self.smart_sleep(3.0)
        self.move_backward_imu(0.1, 2.0)
        self.turn_right_teleop(0.5, 3.0); self.smart_sleep(0.5)

        # 작업 완료 후 어디로 갈지 결정
        if self.stage == "FORKLIFT_RELOADING_PROCESS":
            rospy.loginfo("🚚 재적재 완료! 복귀(DOOR_OUT_RETURN) 좌표로 이동합니다.")
            self.stage = "GO_DOOR_OUT_RETURN" 
            self.send_goal(*self.pos_door_out_return)
        else:
            rospy.loginfo("🚚 일반 적재 완료! 문 안쪽으로 이동합니다.")
            self.stage = "GO_DOOR_INNER"
            self.send_goal(*self.pos_door_in)

    def run_final_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 하역 시작")

        # 안전거리 확보
        # self.move_backward_imu(0.1, 2.0)

        # 물건 내리기
        self.pub_forklift1.publish(Bool(data=False))
        self.wait_for_forklift()
        self.smart_sleep(3.0)
        # self.move_forward_teleop(0.1, 2.0)
        self.move_backward_imu(0.1, 2.0); 

        # 리프트 업
        self.pub_forklift1.publish(Bool(data=True))
        self.wait_for_forklift()
        self.smart_sleep(3.0)


        # self.move_forward_teleop(0.1, 2.0)
        # self.pub_forklift2.publish(Bool(data=False))
        # self.wait_for_forklift()
        # self.smart_sleep(3.0)
        # self.move_backward_imu(0.1, 2.0)
        # self.pub_forklift2.publish(Bool(data=True))
        # self.wait_for_forklift()
        # self.smart_sleep(3.0)
        
        # 2. [수정] 오직 중간 좌표로만 이동 명령을 내립니다.
        rospy.loginfo("🚚 하역 완료. 빈 박스 적재를 위해 중간 좌표로 이동합니다.")
        self.stage = "GO_AFTER_UNLOAD"  # 상태를 먼저 설정
        self.send_goal(*self.pos_after_unload) # 중간 좌표로 출발

    def handle_srv(self, req):
        if req.data and not self.is_paused:
            if self.stage != "IDLE":
                return SetBoolResponse(False, "AGV is already busy")

            rospy.loginfo("🚀 작업 시작 요청 수신 - 작업 완료 후 응답 예정")
            
            # 1. 작업 시작 (첫 동작: 좌회전)
            self.turn_left_teleop(0.5, 3.0) 
            self.stage = "QR_LOAD"
            self._call_qr_task_async(target_id=1)

            # 2. 작업이 완료되어 다시 IDLE이 될 때까지 대기
            rate = rospy.Rate(2) # 0.2초 간격 체크
            while not rospy.is_shutdown():
                if self.stage == "IDLE":
                    rospy.loginfo("🏁 모든 작업이 완료되어 서비스 응답을 보냅니다.")
                    return SetBoolResponse(True, "Task Completed Successfully")
                
                # 비상 정지(PAUSED) 상태인 경우 에러 응답을 보낼지 계속 기다릴지 결정
                if self.is_paused:
                    # 계속 기다리려면 pass, 실패 처리하려면 아래 주석 해제
                    # return SetBoolResponse(False, "Task Interrupted by Fence")
                    pass
                
                rate.sleep()

        return SetBoolResponse(False, "AGV is paused or invalid request")

    def _call_qr_task_async(self, target_id=None):
        if self.is_paused: return
        
        # 1. 파라미터 서버에 목표 ID 설정
        if target_id is not None:
            rospy.set_param('/active_qr_id', target_id)
            rospy.loginfo(f"🔍 QR 목표 ID 설정: {target_id}")
        else:
            # ID가 없으면 파라미터 삭제 (모든 QR 허용 모드)
            if rospy.has_param('/active_qr_id'):
                rospy.delete_param('/active_qr_id')
            rospy.loginfo("🔍 QR 목표 ID 없음: 모든 마커를 허용합니다.")

        self.current_target_id = target_id
        
        # 2. 기존 서비스 호출 (QR 노드 가동 트리거)
        threading.Thread(target=lambda: self.qr_task_srv(True) if self.qr_task_srv else None, daemon=True).start()
        # 별도 스레드에서 실행 (예시)
    def monitor_system(self):
        # 초기화 시 파일 생성 (헤더 작성)
        with open("agv_debug_log.txt", "w") as f:
            f.write("time,tf_delay\n")

        while not rospy.is_shutdown():
            try:
                # TF 지연 시간 계산
                latest_tf_time = self.tf_listener.getLatestCommonTime('base_footprint', 'imu_link')
                tf_delay = (rospy.Time.now() - latest_tf_time).to_sec()
                
                t = rospy.Time.now().to_sec()
                with open("agv_debug_log.txt", "a") as f:
                    f.write(f"{t}, {tf_delay}\n")
                
                # 0.1초 이상 지연 발생 시 터미널에도 경고 출력
                if tf_delay > 0.1:
                    rospy.logwarn_throttle(1, f"⚠️ 시스템 부하 감지: TF 지연 {tf_delay:.4f}초")
                    
            except:
                pass
            rospy.sleep(0.1)

    def send_goal(self, x, y, yaw_deg):
        if self.is_paused: return

        rospy.loginfo("--- [주행 안정화 시퀀스 시작] ---")
        
        # 1. 가속도 잔류 검증 및 강제 초기화
        # 이전 goal이 남긴 속도 명령이 있는지 확인하고 밀어버립니다.
        self.cancel_pub.publish(GoalID())
        for _ in range(3): # 확실히 멈추도록 3번 연속 발행
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.05)

        # 2. TF 데이터 무결성 체크 (로그 기록)
        # imu_link와 base_footprint 사이의 시간차를 계산합니다.
        try:
            (trans, rot) = self.tf_listener.lookupTransform('base_footprint', 'imu_link', rospy.Time(0))
            # TF의 최신 타임스탬프와 현재 시간의 격차를 로그로 출력
            latest_tf_time = self.tf_listener.getLatestCommonTime('base_footprint', 'imu_link')
            time_diff = (rospy.Time.now() - latest_tf_time).to_sec()
            rospy.loginfo(f"📊 TF 지연 상태: {time_diff:.4f}초 (0.1초 이상이면 위험)")
        except Exception as e:
            rospy.logerr(f"❌ TF 체크 실패: {e}")

        # 3. 명시적 정지 대기 (Settling Time)
        # 제어 루프가 '정지'를 완벽히 인지하도록 0.3초 대기
        rospy.sleep(0.3)

        # 4. TF 연산 대기 로직 (로그 포함)
        if not self.wait_for_transform():
            rospy.logwarn("⏳ TF 데이터 불안정: 연산 복구 대기 중...")
            while not self.wait_for_transform() and not rospy.is_shutdown():
                if self.is_paused: return
                rospy.sleep(0.1)
            rospy.loginfo("✅ TF 연산 복구 완료")

        # 5. 최종 Goal 전송 (Current Time Stamp 사용)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now() # 최신 위치 기반 주행 강제
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal.pose.orientation.z = math.sin(yaw/2)
        goal.pose.orientation.w = math.cos(yaw/2)

        rospy.loginfo(f"🎯 최종 Goal 전송: ({x}, {y}) / 각도: {yaw_deg}")
        self.goal_pub.publish(goal)

    def wait_for_resume(self):
        if not self.is_paused: return False
        while self.is_paused and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(Twist()); rospy.sleep(0.1)
        return False

    def smart_sleep(self, duration):
        start_time = rospy.Time.now(); elapsed = 0.0
        while elapsed < duration and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); start_time = rospy.Time.now() - rospy.Duration(elapsed)
            elapsed = (rospy.Time.now() - start_time).to_sec(); rospy.sleep(0.05)
        return False

    def cb_imu(self, msg):
        q = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.current_yaw = yaw

    def get_yaw_error(self, target, current):
        error = target - current
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        return error

    def move_backward_imu(self, speed, duration):
        target_yaw = self.current_yaw; kp = 1.5
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.is_paused: self.wait_for_resume()

            # [추가] 연산 지연 시 멈춰서 기다림
            if not self.wait_for_transform():
                self.cmd_vel_pub.publish(Twist()) # 정지
                rospy.sleep(0.05)
                continue

            error = self.get_yaw_error(target_yaw, self.current_yaw)
            t = Twist(); t.linear.x = -abs(speed); t.angular.z = error * kp
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def turn_left_teleop(self, speed, duration):
        t = Twist(); t.angular.z = abs(speed)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); end += rospy.Duration(0.05)

            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def turn_right_teleop(self, speed, duration):
        t = Twist(); t.angular.z = -abs(speed)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); end += rospy.Duration(0.05)
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def move_forward_teleop(self, speed, duration):
        t = Twist(); t.linear.x = abs(speed)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); end += rospy.Duration(0.05)
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def cb_fence_open(self, msg):
        if msg.data and not self.is_paused:
            rospy.logwarn("🛑 FENCE OPEN! Emergency Stop.")
            self.is_paused = True; self.prev_stage = self.stage; self.stage = "PAUSED"
            self.cancel_pub.publish(GoalID()); self.cmd_vel_pub.publish(Twist())
            if self.qr_task_srv: threading.Thread(target=lambda: self.qr_task_srv(False)).start()
        elif not msg.data and self.is_paused:
            rospy.loginfo("🟢 FENCE CLOSED. Resuming...")
            self.is_paused = False; self.stage = self.prev_stage
            if "GO_" in self.stage:
                if self.stage == "BATT_GO_FINAL": self.send_goal(*self.pos_batt_final)
                elif "DOOR_INNER" in self.stage: self.send_goal(*self.pos_door_in)
                elif "DOOR_OUT" in self.stage: self.send_goal(*self.pos_door_out_forward)
                elif "FINAL" in self.stage: self.send_goal(*self.pos_final)
                elif "RETURN" in self.stage or "RET_GO_OUT" in self.stage: self.send_goal(*self.pos_door_out_return)
                elif "HOME" in self.stage: self.send_goal(*self.pos_home)
                elif self.stage == "GO_AFTER_UNLOAD": self.send_goal(*self.pos_after_unload)
            elif "QR_" in self.stage:
                self._call_qr_task_async(self.current_target_id)

if __name__ == "__main__":
    AgvTaskManager().spin()