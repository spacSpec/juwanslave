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

        rospy.sleep(0.5)
        rospy.loginfo("✅ 시스템 준비 완료")

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

    # ===============================
    # 🎯 메인 사이클 제어 (도착 결과)
    # ===============================
    def cb_result(self, msg):
        if self.is_paused or msg.status.status != 3: return
        rospy.loginfo(f"🎯 도착 완료: {self.stage}")

        # 1. 문 안쪽 도착 (일반/충전 공통)
        if self.stage in ["GO_DOOR_INNER", "BATT_GO_DOOR_INNER"]:
            self.stage = "QR_DOOR_INNER" if self.stage == "GO_DOOR_INNER" else "BATT_QR_DOOR_INNER"
            self._call_qr_task_async(1)

        # 2. 문 밖으로 전진 완료
        elif self.stage == "GO_DOOR_OUT":
            self.stage = "GO_FINAL"; self.send_goal(*self.pos_final)
        
        # 3. 배터리 충전 전용 문 밖 전진 완료
        elif self.stage == "BATT_GO_DOOR_OUT":
            self.stage = "BATT_GO_FINAL"; self.send_goal(*self.pos_batt_final)

        # 4. 일반 하역지 도착
        elif self.stage == "GO_FINAL":
            self.stage = "QR_FINAL"; self._call_qr_task_async(1)

        # 5. 충전 스테이션 도착
        elif self.stage == "BATT_GO_FINAL":
            self.stage = "BATT_QR_FINAL"; self._call_qr_task_async(1)

        # 6. 하역 완료 후 복귀 좌표 도착 (일반/충전후 공통)
        elif self.stage in ["GO_DOOR_OUT_RETURN", "BATT_RET_GO_OUT"]:
            self.stage = "QR_DOOR_OUT_RETURN"; self._call_qr_task_async(1)

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
        self._call_qr_task_async(1)
        
        # QR 완료 대기 (cb_qr_done에서 stage를 바꿔줄 때까지 기다림)
        while self.stage == "QR_HOME_WAITING" and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # 2. 지게차 다운
        rospy.loginfo("🏠 [2/6] 지게차 다운")
        self.pub_forklift1.publish(Bool(data=False))
        self.smart_sleep(2.0)

        # 3. 2초 전진
        rospy.loginfo("🏠 [3/6] 2초 전진")
        self.move_forward_teleop(0.1, 2.0)
        self.smart_sleep(0.5)

        # 4. 2초 후진
        rospy.loginfo("🏠 [4/6] 2초 후진")
        self.move_backward_imu(0.1, 2.0)
        self.smart_sleep(0.5)

        # 5. 지게차 업
        rospy.loginfo("🏠 [5/6] 지게차 업")
        self.pub_forklift1.publish(Bool(data=True))
        self.smart_sleep(1.0)

        # 6. 최종 HOME 정렬 위치 이동
        rospy.loginfo("🏠 [6/6] 최종 정렬 위치로 이동")
        self.stage = "FINISHING_HOME" # 이 상태가 cb_result에서 IDLE을 만듭니다.
        self.send_goal(6.21, -1.07, -70.0)

    # ===============================
    # ✅ QR 완료 시점 제어
    # ===============================
    def cb_qr_done(self, msg):
        if not msg.data or self.is_paused: return
        rospy.loginfo(f"✅ QR 완료: {self.stage}")

        if self.stage == "QR_HOME_WAITING":
            rospy.loginfo("🏠 홈 QR 인식 완료")
            self.stage = "QR_HOME_DONE" # 상태를 바꿔서 run_home_sequence의 루프를 탈출시킴
            return

        if self.stage == "BATT_QR_FINAL":
            self.stage = "IDLE_CHARGING" # 여기서 충전 대기
            rospy.loginfo("⏳ 충전 대기 모드... (10% 초과 시 자동 복귀)")

        elif self.stage == "QR_DOOR_INNER": self.stage = "WAIT_DOOR_OUT"
        elif self.stage == "BATT_QR_DOOR_INNER": self.stage = "BATT_WAIT_DOOR_OUT"
        elif self.stage == "QR_DOOR_OUT_RETURN": self.stage = "WAIT_DOOR_IN" 
            
        elif self.stage == "QR_LOAD":
            self.stage = "FORKLIFT_LOAD"
            threading.Thread(target=self.run_forklift_sequence, daemon=True).start()
        
        elif self.stage == "QR_FINAL":
            self.stage = "FORKLIFT_FINAL" # 하역 작업 시작
            threading.Thread(target=self.run_final_forklift_sequence, daemon=True).start()

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

    # ===============================
    # 🏗️ 하역/적재 시퀀스 및 유틸리티
    # ===============================
    def cb_forklift_done(self, msg):
        if not msg.data or self.is_paused: return
        # 적재 끝났으면 문 안쪽으로 이동
        if self.stage == "FORKLIFT_LOAD":
            self.stage = "GO_DOOR_INNER"; self.send_goal(*self.pos_door_in)
        # 하역 끝났으면 복귀 좌표로 이동 (이 부분이 하역 후 복귀 핵심)
        elif self.stage == "FORKLIFT_FINAL":
            self.stage = "GO_DOOR_OUT_RETURN"; self.send_goal(*self.pos_door_out_return)

    def run_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 적재 시작")
        self.pub_forklift1.publish(Bool(data=False)); self.smart_sleep(3.0)
        self.move_forward_teleop(0.1, 2.2)
        self.pub_forklift1.publish(Bool(data=True)); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0)
        self.turn_right_teleop(0.5, 3.0); self.smart_sleep(0.5)
        self.forklift_done_flag = True

    def run_final_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 하역 시작")
        self.move_backward_imu(0.1, 2.0)
        self.pub_forklift1.publish(Bool(data=False)); self.smart_sleep(1.0)
        self.move_forward_teleop(0.1, 2.0); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0); self.pub_forklift1.publish(Bool(data=True)); self.smart_sleep(2.0)
        self.pub_forklift2.publish(Bool(data=False)); self.smart_sleep(1.0)
        self.move_forward_teleop(0.1, 2.0); self.pub_forklift2.publish(Bool(data=True)); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0)
        self.forklift_done_flag = True

    def handle_srv(self, req):
        if req.data and not self.is_paused:
            rospy.loginfo("🚀 작업 시작 요청 수신")
            self.turn_left_teleop(0.5, 3.0) 
            self.stage = "QR_LOAD"; self._call_qr_task_async(1)
            return SetBoolResponse(True, "Started")
        return SetBoolResponse(False, "Failed")

    def _call_qr_task_async(self, target_id=1):
        if self.is_paused: return
        threading.Thread(target=lambda: self.qr_task_srv(True) if self.qr_task_srv else None, daemon=True).start()

    def send_goal(self, x, y, yaw_deg):
        if self.is_paused: return
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = "map", rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = x, y
        yaw = math.radians(yaw_deg)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
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
            elif "QR_" in self.stage: self._call_qr_task_async(1)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.forklift_done_flag:
                self.forklift_done_flag = False
                self.cb_forklift_done(Bool(data=True))
            rate.sleep()

if __name__ == "__main__":
    AgvTaskManager().spin()