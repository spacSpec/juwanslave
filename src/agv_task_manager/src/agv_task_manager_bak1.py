#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, threading, tf
from std_msgs.msg import Bool, Int8
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID 
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Imu

class AgvTaskManager:
    def __init__(self):
        rospy.init_node('agv_task_manager', anonymous=False)
        rospy.loginfo("🚀 AGV Task Manager: Battery & Photo Integrated Version")

        # --- 상태 변수 ---
        self.stage = "IDLE"
        self.prev_stage = "IDLE"
        self.is_paused = False
        self.forklift_done_flag = False
        self.current_yaw = 0.0
        self.battery_level = 100 

        # --- 목적지 좌표 ---
        self.pos_door_in = (7.98, -2.71, 15.86)     
        self.pos_door_out_forward = (9.89, -2.16, 16.03) 
        self.pos_door_out_return = (9.89, -2.16, -163.03) 
        self.pos_final = (9.28, -1.21, 98.04)       
        self.pos_home = (6.37, -1.18, 109.66)
        self.pos_image_target = (6.28, -0.95, -72.31) # 사진 좌표
        self.pos_batt_final = (10.30, -0.22, 16.06)   # 배터리 복귀 좌표

        # --- Publisher & Subscriber (구독 등록을 최상단으로) ---
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.pub_forklift1 = rospy.Publisher("/forklift1_cmd", Bool, queue_size=1)
        self.pub_forklift2 = rospy.Publisher("/forklift2_cmd", Bool, queue_size=1)

        # 배터리 구독 (타입: Int8)
        rospy.Subscriber('/battery_percent', Int8, self.cb_battery)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.cb_result)
        rospy.Subscriber('/agv/fence_open', Bool, self.cb_fence_open)
        rospy.Subscriber('/agv/door_open', Bool, self.cb_door_open)
        rospy.Subscriber('/forklift_done', Bool, self.cb_forklift_done)
        rospy.Subscriber('/qr_task_done', Bool, self.cb_qr_done)
        rospy.Subscriber('/imu/data_filtered', Imu, self.cb_imu)

        # --- Service ---
        rospy.Service("/agv/request_dispatch", SetBool, self.handle_srv)
        
        # 서비스 연결 시 무한 대기 방지
        self.qr_task_srv = None
        threading.Thread(target=self.init_qr_service, daemon=True).start()

        rospy.sleep(1.0)
        self.pub_forklift1.publish(Bool(data=True)) 
        rospy.loginfo("✅ 시스템 준비 완료 (배터리 모니터링 활성화)")
        

    # ===============================
    # 3. 안전 및 유틸리티 함수
    # ===============================
    def wait_for_resume(self):
        if not self.is_paused: return False
        rospy.logwarn("⚠️ 작업 일시 중지: 펜스 닫힘 대기 중...")
        while self.is_paused and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(Twist()) 
            rospy.sleep(0.1)
        rospy.loginfo("🟢 작업 재개")
        return False

    def smart_sleep(self, duration):
        start_time = rospy.Time.now()
        elapsed = 0.0
        while elapsed < duration and not rospy.is_shutdown():
            if self.is_paused:
                self.wait_for_resume()
                start_time = rospy.Time.now() - rospy.Duration(elapsed)
            elapsed = (rospy.Time.now() - start_time).to_sec()
            rospy.sleep(0.05)
        return False

    def cb_imu(self, msg):
        orientation_q = msg.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def get_yaw_error(self, target, current):
        error = target - current
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        return error

    def move_backward_imu(self, speed, duration):
        target_yaw = self.current_yaw
        kp = 1.5
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.is_paused: self.wait_for_resume()
            error = self.get_yaw_error(target_yaw, self.current_yaw)
            t = Twist(); t.linear.x = -abs(speed); t.angular.z = error * kp
            self.cmd_vel_pub.publish(t)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def turn_left_teleop(self, speed, duration):
        t = Twist(); t.angular.z = abs(speed)
        start_time = rospy.Time.now(); elapsed = 0.0
        while elapsed < duration and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); start_time = rospy.Time.now() - rospy.Duration(elapsed)
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
            elapsed = (rospy.Time.now() - start_time).to_sec()
        self.cmd_vel_pub.publish(Twist())

    def turn_right_teleop(self, speed, duration):
        t = Twist(); t.angular.z = -abs(speed)
        start_time = rospy.Time.now(); elapsed = 0.0
        while elapsed < duration and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); start_time = rospy.Time.now() - rospy.Duration(elapsed)
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
            elapsed = (rospy.Time.now() - start_time).to_sec()
        self.cmd_vel_pub.publish(Twist())

    def move_forward_teleop(self, speed, duration):
        t = Twist(); t.linear.x = abs(speed)
        start_time = rospy.Time.now(); elapsed = 0.0
        while elapsed < duration and not rospy.is_shutdown():
            if self.is_paused: self.wait_for_resume(); start_time = rospy.Time.now() - rospy.Duration(elapsed)
            self.cmd_vel_pub.publish(t); rospy.sleep(0.05)
            elapsed = (rospy.Time.now() - start_time).to_sec()
        self.cmd_vel_pub.publish(Twist())

    # ===============================
    # 1. 메인 사이클 제어
    # ===============================
    def cb_result(self, msg):
        if self.is_paused or msg.status.status != 3: return
        rospy.loginfo(f"🎯 도착 완료: {self.stage}")

        if self.stage == "GO_DOOR_INNER":
            self.stage = "QR_DOOR_INNER"; self._call_qr_task_async(1)
        elif self.stage == "GO_DOOR_OUT":
            self.stage = "GO_FINAL"; self.send_goal(*self.pos_final)
        elif self.stage == "GO_FINAL":
            self.stage = "QR_FINAL"; self._call_qr_task_async(1)
        elif self.stage == "GO_DOOR_OUT_RETURN":
            self.stage = "QR_DOOR_OUT_RETURN"; self._call_qr_task_async(1)
        elif self.stage == "GO_HOME":
            rospy.loginfo("🏁 HOME 도착. 방향 정렬 후 QR 탐색")
            self.turn_right_teleop(0.5, 2.0)
            self.stage = "QR_HOME_FINAL"
            self._call_qr_task_async(1)
        
        # [추가] 사진 위치 도착 시 최종 종료
        elif self.stage == "GO_FINAL_IMAGE_POS":
            rospy.loginfo("🏁 사진상의 최종 목적지에 도착했습니다. 모든 작업 종료.")
            self.stage = "IDLE"

    def cb_qr_done(self, msg):
        if not msg.data or self.is_paused: return
        rospy.loginfo(f"✅ QR 완료: {self.stage}")

        if self.stage == "QR_LOAD":
            self.stage = "FORKLIFT_LOAD"
            threading.Thread(target=self.run_forklift_sequence, daemon=True).start()
        elif self.stage == "QR_DOOR_INNER":
            self.stage = "WAIT_DOOR_OUT"
        elif self.stage == "QR_FINAL":
            self.stage = "FORKLIFT_FINAL"
            threading.Thread(target=self.run_final_forklift_sequence, daemon=True).start()
        elif self.stage == "QR_DOOR_OUT_RETURN":
            self.stage = "WAIT_DOOR_IN"
        elif self.stage == "QR_HOME_FINAL":
            self.stage = "FORKLIFT_HOME_UNLOAD"
            threading.Thread(target=self.run_home_unload_sequence, daemon=True).start()

    def cb_door_open(self, msg):
        if not msg.data or self.is_paused: return
        if self.stage == "WAIT_DOOR_OUT":
            self.stage = "GO_DOOR_OUT"; self.send_goal(*self.pos_door_out_forward)
        elif self.stage == "WAIT_DOOR_IN":
            self.stage = "GO_HOME"; self.send_goal(*self.pos_home)

    def cb_forklift_done(self, msg):
        if not msg.data or self.is_paused: return
        if self.stage == "FORKLIFT_LOAD":
            self.stage = "GO_DOOR_INNER"; self.send_goal(*self.pos_door_in)
        elif self.stage == "FORKLIFT_FINAL":
            self.stage = "GO_DOOR_OUT_RETURN"; self.send_goal(*self.pos_door_out_return)
        
        # [수정] HOME 하역 완료 후 오른쪽 회전 및 사진 좌표로 이동
        elif self.stage == "FORKLIFT_HOME_UNLOAD":
            rospy.loginfo("✅ HOME 하역 완료. 오른쪽으로 5.5초 회전 후 최종 목적지로 이동합니다.")
            self.turn_right_teleop(0.5, 5.5) # 요청하신 5.5초 회전
            self.stage = "GO_FINAL_IMAGE_POS"
            self.send_goal(*self.pos_image_target)

    # ===============================
    # 2. 지게차 시퀀스
    # ===============================
    def run_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 물건 적재 시작")
        self.pub_forklift1.publish(Bool(data=False)); self.smart_sleep(3.0)
        self.move_forward_teleop(0.1, 2.2)
        self.pub_forklift1.publish(Bool(data=True)); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0)
        rospy.loginfo("🔄 적재 완료: 180도 회전")
        self.turn_right_teleop(0.5, 3.0); self.smart_sleep(0.5)
        self.forklift_done_flag = True

    def run_final_forklift_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: 목적지 하역 시퀀스")
        self.move_backward_imu(0.1, 2.0)
        self.pub_forklift1.publish(Bool(data=False)); self.smart_sleep(1.0)
        self.move_forward_teleop(0.1, 2.0); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0); self.pub_forklift1.publish(Bool(data=True)); self.smart_sleep(2.0)
        self.pub_forklift2.publish(Bool(data=False)); self.smart_sleep(1.0)
        self.move_forward_teleop(0.1, 2.0); self.pub_forklift2.publish(Bool(data=True)); self.smart_sleep(1.0)
        self.move_backward_imu(0.1, 2.0)
        self.forklift_done_flag = True

    # HOME 하역 시퀀스 (다운->후진->업)
    def run_home_unload_sequence(self):
        rospy.loginfo("🏗️ 시퀀스: HOME 하역 (다운->전진->후진->업)")
        self.pub_forklift1.publish(Bool(data=False)); self.smart_sleep(3.0)
        self.move_backward_imu(0.1, 2.0)
        self.pub_forklift1.publish(Bool(data=True)); self.smart_sleep(2.0)
        self.forklift_done_flag = True

    # ===============================
    # 서비스 및 시스템 제어
    # ===============================
    def handle_srv(self, req):
        if req.data and not self.is_paused:
            rospy.loginfo("🚀 작업 시작")
            self.turn_left_teleop(0.5, 3.0) 
            self.stage = "QR_LOAD"; self._call_qr_task_async(1)
            return SetBoolResponse(True, "Started")
        return SetBoolResponse(False, "Failed")

    def _call_qr_task_async(self, target_id=1):
        if self.is_paused: return
        threading.Thread(target=lambda: self.qr_task_srv(True), daemon=True).start()

    def send_goal(self, x, y, yaw_deg):
        if self.is_paused: return
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = "map", rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = x, y
        yaw = math.radians(yaw_deg)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.goal_pub.publish(goal)

    def cb_fence_open(self, msg):
        if msg.data and not self.is_paused:
            rospy.logwarn("🛑 FENCE OPEN! Emergency Stop.")
            self.is_paused = True
            self.prev_stage = self.stage
            self.stage = "PAUSED"
            self.cancel_pub.publish(GoalID()) 
            self.cmd_vel_pub.publish(Twist()) 
            try: self.qr_task_srv(False)
            except: pass

        elif not msg.data and self.is_paused:
            rospy.loginfo("🟢 FENCE CLOSED. Resuming...")
            self.is_paused = False
            self.stage = self.prev_stage
            
            if "GO_" in self.stage:
                if self.stage == "GO_DOOR_INNER": self.send_goal(*self.pos_door_in)
                elif self.stage == "GO_DOOR_OUT": self.send_goal(*self.pos_door_out_forward)
                elif self.stage == "GO_FINAL": self.send_goal(*self.pos_final)
                elif self.stage == "GO_DOOR_OUT_RETURN": self.send_goal(*self.pos_door_out_return)
                elif self.stage == "GO_HOME": self.send_goal(*self.pos_home)
                elif self.stage == "GO_FINAL_IMAGE_POS": self.send_goal(*self.pos_image_target)
            elif "QR_" in self.stage:
                self._call_qr_task_async(1)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.forklift_done_flag:
                self.forklift_done_flag = False
                self.cb_forklift_done(Bool(data=True))
            rate.sleep()

if __name__ == "__main__":
    AgvTaskManager().spin()