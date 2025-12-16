#ifndef MYAGV_H
#define MYAGV_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <sensor_msgs/Imu.h>

class MyAGV
{
public:
    MyAGV();
    ~MyAGV();

    bool init();
    bool execute(double linearX, double linearY, double angularZ);  // cpp와 동일하게 bool 유지

private:
    // ---------------- Serial 통신 Core ----------------
    bool readSpeed();                         // 모터값+IMU 패킷 읽기
    void writeSpeed(double movex, double movey, double rot);
    void restore();                           // 모터 초기 Reset 명령
    void restoreRun();                        // 강제 Restore

    // ---------------- ROS 시간 ----------------
    ros::Time currentTime;
    ros::Time lastTime;

    // ---------------- ODOM 상태 ----------------
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // ---------------- 속도 정보 ----------------
    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;

    // ---------------- IMU Raw 데이터 ----------------
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;

    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

    // (EKF 사용 시 orientation fusion 하므로 raw 기준 유지)
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // 배터리
    double Battery_voltage = 0.0;
    double Backup_Battery_voltage = 0.0;

    // ---------------- ROS Node & Topics ----------------
    ros::NodeHandle n;

    ros::Publisher pub;            // /odom
    ros::Publisher pub_v;          // /Voltage
    ros::Publisher pub_imu;        // /imu/data (필요시 orientation 포함 version)

    tf::TransformBroadcaster odomBroadcaster;
};

#endif
