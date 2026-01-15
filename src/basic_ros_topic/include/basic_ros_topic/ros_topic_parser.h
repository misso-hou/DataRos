#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

// 包含已编译的protobuf头文件
#include "plusai_common_proto/control/dbw_reports.pb.h"

struct VehicleData {
    double steer_wheel_angle;
    double steer_wheel_torque;
    double wheel_speed;
    double yaw_rate;
};

class MsgParserTest {
    public:
        MsgParserTest();
        ~MsgParserTest();

    public:
        VehicleData getVehicleData();

    private:
        void callback(const std_msgs::String::ConstPtr& msg);
        void writeToCSV(double timestamp, double angle, double torque, double speed, double yaw);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::ofstream csv_file_;
        std::string csv_file_path_;
        std::mutex data_mutex_;

    private:   // 数据成员变量
        double steering_wheel_angle_;
        double steering_wheel_torque_;
        double wheel_speed_;
        double yaw_rate_;
};
