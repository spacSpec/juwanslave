#include <vector>
#include <iostream>
#include <iomanip>
#include <time.h>

#include "myagv_odometry/myAGV.h"
#include "std_msgs/Int8.h"
#include <sensor_msgs/Imu.h>

const unsigned char header[2] = {0xfe, 0xfe};

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyAMA2");

boost::array<double, 36> odom_pose_covariance = {
    0.05, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 0.05
};

boost::array<double, 36> odom_twist_covariance = {
    0.05, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 0.05
};

MyAGV::MyAGV()
{
    x = y = theta = 0.0;
    vx = vy = vtheta = 0.0;
}

MyAGV::~MyAGV() {}

bool MyAGV::init()
{
    std::cout << "[1] init complited" << std::endl;

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    std::cout << "[2] Serial complited" << std::endl;

    ros::Time::init();
    currentTime = lastTime = ros::Time::now();

    std::cout << "[3] ROS time init complited" << std::endl;

    pub     = n.advertise<nav_msgs::Odometry>("odom", 50);
    pub_v   = n.advertise<std_msgs::Int8>("Voltage", 1000);
    pub_imu = n.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);

    std::cout << "[4] Publisher complited" << std::endl;

    restore();
    std::cout << "[5] restore() complited" << std::endl;

    return true;
}

void MyAGV::restore()
{
    boost::asio::streambuf clear_buffer;
    boost::asio::read(sp, clear_buffer, boost::asio::transfer_at_least(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};
    boost::asio::write(sp, boost::asio::buffer(cmd));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void MyAGV::restoreRun()
{
    int res = 0;
    std::cout << "if you want restore run, pls input 1" << std::endl;
    while (res != 1) std::cin >> res;

    restore();
    std::cout << "restore finished" << std::endl;
}

bool MyAGV::readSpeed()
{
    unsigned char buf_header[1] = {0};
    unsigned char buf[27] = {0};
    boost::system::error_code er2;

    // ---- HEADER 찾기 ----
    bool header_found = false;
    while (!header_found)
    {
        boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (buf_header[0] != header[0]) continue;

        boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (buf_header[0] == header[0]) header_found = true;
    }

    // ---- DATA 읽기 ----
    size_t ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);
    if (ret != 27)
    {
        ROS_ERROR("Read error %ld", ret);
        return false;
    }

    // ---- checksum 검증 ----
    int check = 0;
    for (int i = 0; i < 26; i++) check += buf[i];
    if (check % 256 != buf[26])
    {
        ROS_ERROR("Checksum error!");
        return false;
    }

    // ----- Sensor decoding -----
    const double ODOM_SCALE = 1.214;
    vx = (buf[0] - 128.0) * 0.01 * ODOM_SCALE;
    vy = (buf[1] - 128.0) * 0.01 * ODOM_SCALE;

    ax = ((buf[3] + buf[4] * 256) - 10000) * 0.001;
    ay = ((buf[5] + buf[6] * 256) - 10000) * 0.001;
    az = ((buf[7] + buf[8] * 256) - 10000) * 0.001;

    wx = ((buf[9]  + buf[10] * 256) - 10000) * 0.1;
    wy = ((buf[11] + buf[12] * 256) - 10000) * 0.1;
    wz = ((buf[13] + buf[14] * 256) - 10000) * 0.1;

    // IMU yaw-rate(rad/s)
    double imu_yaw_rate = wz * M_PI / 180.0;
    vtheta = imu_yaw_rate;

    // ---- odom 적분 ----
    currentTime = ros::Time::now();
    double dt = (currentTime - lastTime).toSec();

    double delta_x  = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y  = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_th = vtheta * dt;

    x     += delta_x;
    y     += delta_y;
    theta += delta_th;

    lastTime = currentTime;

    // ---- IMU publish ----
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = currentTime;
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.angular_velocity.x = wx * M_PI / 180.0;
    imu_msg.angular_velocity.y = wy * M_PI / 180.0;
    imu_msg.angular_velocity.z = wz * M_PI / 180.0;

    imu_msg.orientation_covariance[0] = -1;

    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    pub_imu.publish(imu_msg);

    return true;
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    //     // [ADD] ----------------------------------------------------------
    // // 특수 명령: (10, 10, 10)이 들어오면 주행 명령 대신
    // // 배터리 전압을 요청해서 콘솔에 출력만 하고 종료
    // if (movex == 10 && movey == 10 && rot == 10)
    // {
    //     // 예전 코드에서 쓰던 요청 패킷 그대로 사용
    //     unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x01, 0x01, 0x03};
    //     boost::asio::write(sp, boost::asio::buffer(cmd));

    //     unsigned char buf_header[1] = {0};
    //     boost::system::error_code er2;
    //     time_t start_t = time(NULL);
    //     bool header_found = false;

    //     // 0xfe 0xfe 헤더를 다시 찾는다.
    //     while (!header_found && (time(NULL) - start_t) < 3)
    //     {
    //         size_t ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
    //         if (ret != 1) continue;
    //         if (buf_header[0] != header[0]) continue;

    //         // 두 번째 0xfe 확인
    //         ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
    //         if (ret != 1) continue;
    //         if (buf_header[0] != header[0]) continue;

    //         header_found = true;
    //     }

    //     if (!header_found)
    //     {
    //         ROS_ERROR("Get Voltage header timeout");
    //         return;
    //     }

    //     // 이후 3바이트: (명령 ID, 상태, 전압 raw값)을 기대
    //     unsigned char id = 0;
    //     unsigned char status = 0;
    //     unsigned char value = 0;

    //     size_t ret = boost::asio::read(sp, boost::asio::buffer(&id, 1), er2);
    //     if (ret != 1)
    //     {
    //         ROS_ERROR("Get Voltage read id failed");
    //         return;
    //     }

    //     ret = boost::asio::read(sp, boost::asio::buffer(&status, 1), er2);
    //     if (ret != 1)
    //     {
    //         ROS_ERROR("Get Voltage read status failed");
    //         return;
    //     }

    //     ret = boost::asio::read(sp, boost::asio::buffer(&value, 1), er2);
    //     if (ret != 1)
    //     {
    //         ROS_ERROR("Get Voltage read value failed");
    //         return;
    //     }

    //     if (status != 0x01)
    //     {
    //         ROS_ERROR("Get Voltage response status error: 0x%02x", status);
    //         return;
    //     }

    //     int raw = static_cast<int>(value);      // 예전 코드 기준: 0.1V 단위
    //     double voltage = raw / 10.0;            // ex) raw=123 -> 12.3V

    //     ROS_INFO("Battery Voltage: %.1f V (raw=%d)", voltage, raw);

    //     // 여기서는 배터리만 출력하고, 실제 속도 명령은 보내지 않는다.
    //     return;
    // }
    // // [ADD] ----------------------------------------------------------


    if (movex > 1) movex = 1;
    if (movex < -1) movex = -1;
    if (movey > 1) movey = 1;
    if (movey < -1) movey = -1;
    if (rot > 1) rot = 1;
    if (rot < -1) rot = -1;

    unsigned char x_send  = (signed char)(movex * 100) + 128;
    unsigned char y_send  = (signed char)(movey * 100) + 128;
    unsigned char rot_send = (signed char)(rot * 100) + 128;
    unsigned char check   = x_send + y_send + rot_send;

    unsigned char buf[8] = {
        header[0], header[1],
        x_send, y_send, rot_send,
        check
    };

    boost::asio::write(sp, boost::asio::buffer(buf));
}

bool MyAGV::execute(double linearX, double linearY, double angularZ)
{
    writeSpeed(linearX, linearY, angularZ);

    if (!readSpeed())
    {
        ROS_WARN("readSpeed failed!");
        return false;
    }

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.rotation = odom_quat;

    odomBroadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry msg;
    msg.header.stamp = currentTime;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation = odom_quat;
    msg.pose.covariance = odom_pose_covariance;

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = vtheta;
    msg.twist.covariance = odom_twist_covariance;

    pub.publish(msg);

    return true;
}
