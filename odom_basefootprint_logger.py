#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomBaseFootprintLogger:
    def __init__(self):
        # 파라미터: 사용할 odom 토픽 (기본값: /odometry/filtered)
        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")

        rospy.loginfo("=== Odom & base_footprint CSV Logger ===")
        rospy.loginfo("사용 중인 Odom 토픽: %s", self.odom_topic)

        # 최신 odom 메시지 저장용
        self.latest_odom = None

        # Odometry 구독
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        # TF Listener (odom → base_footprint)
        self.tf_listener = tf.TransformListener()

        # CSV 파일 생성
        self.csv_file, self.csv_writer = self.create_csv_file()

        # 로깅 주기 (Hz)
        rate_hz = rospy.get_param("~rate", 20.0)
        self.rate = rospy.Rate(rate_hz)

    def create_csv_file(self):
        # 현재 시간 기반 파일 이름 생성
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"odom_basefootprint_log_{now_str}.csv"

        # 현재 디렉토리에 저장 (rosrun 실행 위치 기준)
        filepath = os.path.join(os.getcwd(), filename)

        rospy.loginfo("CSV 파일 생성: %s", filepath)

        f = open(filepath, mode="w", newline="")
        writer = csv.writer(f)

        # 헤더 작성
        writer.writerow([
            "ros_time",
            "odom_x", "odom_y", "odom_yaw_deg",
            "base_footprint_x", "base_footprint_y", "base_footprint_yaw_deg"
        ])

        return f, writer

    def odom_callback(self, msg):
        self.latest_odom = msg

    def get_odom_pose(self):
        """
        nav_msgs/Odometry -> (x, y, yaw_deg)
        """
        if self.latest_odom is None:
            return None

        pose = self.latest_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        yaw_deg = yaw * 180.0 / 3.141592653589793
        return x, y, yaw_deg

    def get_base_footprint_pose(self):
        """
        TF에서 odom → base_footprint 변환을 읽어서 (x, y, yaw_deg) 리턴
        """
        try:
            # (parent, child) = ("odom", "base_footprint")
            (trans, rot) = self.tf_listener.lookupTransform(
                "odom", "base_footprint", rospy.Time(0)
            )
            x = trans[0]
            y = trans[1]
            quat = rot
            (_, _, yaw) = euler_from_quaternion(quat)
            yaw_deg = yaw * 180.0 / 3.141592653589793
            return x, y, yaw_deg
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "TF odom->base_footprint 조회 실패: %s", str(e))
            return None

    def spin(self):
        while not rospy.is_shutdown():
            odom_pose = self.get_odom_pose()
            base_pose = self.get_base_footprint_pose()

            # 아직 데이터가 준비되지 않은 경우 스킵
            if odom_pose is None or base_pose is None:
                self.rate.sleep()
                continue

            now_time = rospy.get_time()

            row = [
                now_time,
                odom_pose[0], odom_pose[1], odom_pose[2],
                base_pose[0], base_pose[1], base_pose[2]
            ]

            self.csv_writer.writerow(row)

            # 너무 자주 로그가 찍히지 않도록 throttle
            rospy.loginfo_throttle(5.0, "logging... t=%.2f, odom(%.3f, %.3f), base(%.3f, %.3f)",
                                   now_time, odom_pose[0], odom_pose[1],
                                   base_pose[0], base_pose[1])

            self.rate.sleep()

    def close(self):
        try:
            if self.csv_file:
                self.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("odom_basefootprint_logger", anonymous=True)
    logger = OdomBaseFootprintLogger()
    try:
        logger.spin()
    finally:
        logger.close()
        rospy.loginfo("CSV Logger 종료")
