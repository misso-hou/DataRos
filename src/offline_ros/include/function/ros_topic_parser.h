#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

// 包含已编译的protobuf头文件
#include "plusai_common_proto/control/dbw_reports.pb.h"
#include "plusai_common_proto/control/control_command.pb.h"
namespace func {
namespace msg_parser {

extern float TS;

struct VehicleSteerData {
    std::string local_time;
    float steer_wheel_angle;
    float steer_wheel_torque_filtered;
    float wheel_speed;
    float yaw_rate;
    float steer_wheel_angle_dot;
    bool pilot_state;
};

struct VehicleBrakeData {
    std::string local_time;
    float ebs_cmd;
    float acc_mes;
    float acc_ref;
    float speed;
    float pitch;
    float brake_pressure_filtered;
    float wheel_speed;
};

enum class DataIndex{
    SWA = 0,
    SWT,
    WHEEL_SPEED,
    YAW_RATE,
    EBS_CMD,
    ACC_MES,
    ACC_REF,
    SPEED,
    PITCH,
    BRAKE_PRESSURE,
    PILOT
};

template <typename E>
constexpr auto to_int(E e) noexcept {
    static_assert(std::is_enum<E>::value, "to_int only works with enum types");
    return static_cast<typename std::underlying_type<E>::type>(e);
}

class MsgParser {
    public:
        MsgParser(int argc, char *argv[]);
        ~MsgParser();

    public:
        VehicleSteerData getVehicleSteerData();
        VehicleBrakeData getVehicleBrakeData();

    private:
        void dbw_callback(const std_msgs::String::ConstPtr& msg);
        void ctrl_callback(const std_msgs::String::ConstPtr& msg);
        void writeToCSV(const long long timestamp, const std::vector<float>& data);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber dbw_sub_;
        ros::Subscriber ctrl_sub_;
        std::ofstream csv_file_;
        std::string csv_file_path_;
        std::mutex data_mutex_;

    private:   // 数据成员变量
        std::string local_time_;
        std::vector<float> record_data_;
        bool first_flag_ = true;
    
    private:  // 后处理数据
        float swa_dot_ = 0.0;
        float swt_filtered_ = 0.0;
        float brake_pressure_filtered_ = 0.0;
};

}
}
