#include "basic_ros_topic/ros_topic_parser.h"
#include <ros/package.h>

MsgParserTest::MsgParserTest() {
    // 获取包的路径
    std::string package_path = ros::package::getPath("basic_ros_topic");
    // 构造绝对路径
    csv_file_path_ = package_path + "/data/test_data01.csv";
    // 打开CSV文件，清空原有内容（相当于Python中的'w'模式）
    csv_file_.open(csv_file_path_.c_str());
    if (!csv_file_.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_file_path_.c_str());
        return;
    }
    
    // 可选：写入CSV表头
    // csv_file_ << "steering_wheel_angle,steering_wheel_torque,wheel_speed,yaw_rate\n";
    
    // 订阅话题
    sub_ = nh_.subscribe("/vehicle/dbw_reports", 1000, &MsgParserTest::callback, this);
    
    ROS_INFO("DBW Reports listener started. Saving data to: %s", csv_file_path_.c_str());
}

MsgParserTest::~MsgParserTest()
{
    if (csv_file_.is_open())
    {
        csv_file_.close();
    }
}

void MsgParserTest::callback(const std_msgs::String::ConstPtr& msg)
{
    try
    {
        // 解析protobuf消息
        drive::common::control::DbwReports dbw_report;
        if (!dbw_report.ParseFromString(msg->data))
        {
            ROS_WARN("Failed to parse protobuf message");
            return;
        }
        
        // 提取数据
        double steering_wheel_angle = dbw_report.steering_report().steering_wheel_angle();
        double steering_wheel_torque = dbw_report.steering_report().steering_wheel_torque();
        double wheel_speed = dbw_report.wheel_speed_report().front_axle_speed();
        double yaw_rate = dbw_report.vehicle_dynamic().angular_velocity().z();

        // 提取→转秒→转时间
        double ts_msec = dbw_report.header().timestamp_msec(); // 提取原始毫秒戳
        double ts_sec = ts_msec / 1000.0;                     // 转秒
        
        // 关键修正：先定义time_t变量（左值），再取地址
        time_t raw_sec = static_cast<time_t>(ts_sec);
        struct tm t;
        gmtime_r(&raw_sec, &t); // 现在可以正常取地址，无编译报错

        // 时区修正：UTC+8（北京时间），取模24避免小时超过23
        int beijing_hour = (t.tm_hour + 8) % 24;
        
        // 仅打印北京时间的时分秒
        ROS_INFO("%02d:%02d:%02d", beijing_hour, t.tm_min, t.tm_sec);

        
        // 如果数据不是0.0，写入CSV
        if (steering_wheel_torque != 0.0)
        {
            writeToCSV(ts_msec, steering_wheel_angle, steering_wheel_torque, wheel_speed, yaw_rate);
        }

        // update record data
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            steering_wheel_angle_ = steering_wheel_angle;
            steering_wheel_torque_ = steering_wheel_torque;
            wheel_speed_ = wheel_speed;
            yaw_rate_ = yaw_rate;
        }
    }
    catch (const std::exception& e)
    {
        ROS_WARN("Failed to decode binary data: %s", e.what());
    }
    catch (...)
    {
        ROS_WARN("Failed to decode binary data: Unknown error");
    }
}

void MsgParserTest::writeToCSV(double timestamp, double angle, double torque, double speed, double yaw)
{
    if (!csv_file_.is_open())
    {
        ROS_ERROR("CSV file is not open!");
        return;
    }
    
    // 使用stringstream格式化数据
    std::stringstream ss;
    ss << timestamp << "," << angle << "," << torque << "," << speed << "," << yaw << "\n";
    
    // 写入文件
    csv_file_ << ss.str();
    
    // 立即刷新缓冲区，确保数据及时写入（可选，影响性能）
    // csv_file_.flush();
    
    // 可选：打印到ROS日志
    // ROS_INFO("Saved: %.3f, %.3f, %.3f, %.3f", angle, torque, speed, yaw);
}

VehicleData MsgParserTest::getVehicleData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {steering_wheel_angle_, 
            steering_wheel_torque_, 
            wheel_speed_,
            yaw_rate_};
}
