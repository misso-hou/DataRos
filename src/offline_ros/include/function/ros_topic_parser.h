#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <map>

// 包含已编译的protobuf头文件
#include "plusai_common_proto/control/dbw_reports.pb.h"
#include "plusai_common_proto/control/control_command.pb.h"
#include "plusai_common_proto/monitor/app_watchdog_state.pb.h"

namespace func {
namespace msg_parser {

namespace control = drive::common::control;
namespace monitor = drive::common::monitor;

extern float TS;

enum FSMState {
    UNKNOWN = 0, // As a placeholder
    SelfInspection = 1,
    Fault = 2,
    Off = 3,
    Standby = 4,
    Active = 5,
    LevelOneAlarming = 6,
    LevelTwoAlarming = 7,
    LevelThreeAlarming = 8,
    VehicleLevelOneAlarming = 9,
    VehicleLevelTwoAlarming = 10,
    PedestrianLevelOneAlarming = 11,
    PedestrianLevelTwoAlarming = 12,
    VehicleObstacleAlarmingAndEB = 13,
    PedestrianObstacleAlarmingAndEB = 14,
    LeftDepartureAlarming = 15,
    RightDepartureAlarming = 16,
    LeftBlindSpotAlarming = 17,
    RightBlindSpotAlarming = 18,
    BothSidesBlindSpotAlarming = 19,
    NotAvailable = 20,
    STAGE1_SELF_INSPECTION = 21,
    STAGE1_SELF_INSPECTION_FAILED = 22,
    STAGE2_SELF_INSPECTION = 23,
    Disabled = 24,
    MCUTakeover = 25,
    MCUFault = 26,
    NotTrigger = 27,
    BeforeEMP = 28,
    EMPAvailable = 29,
    EMPUnavailable = 30,
    LeftBlindSpotLevelTwoAlarming = 31,
    LeftBlindSpotLevelThreeAlarming = 32,
    RightBlindSpotLevelTwoAlarming = 33,
    RightBlindSpotLevelThreeAlarming = 34
};

struct ComputeData {
    std::string local_time;
    std::vector<std::string> order = {"ebs_cmd", 
                                      "acc_mes",
                                      "acc_ref", 
                                      "speed", 
                                      "pitch", 
                                      "brake_pressure_filtered", 
                                      "wheel_speed",
                                      "brake_gain",
                                      "steer_wheel_angle", 
                                      "steer_wheel_torque_filtered", 
                                      "yaw_rate", 
                                      "steer_wheel_angle_dot",
                                      "steer_wheel_torque_mode",
                                      "long_window_mean",
                                      "short_window_mean",
                                      "adas_state"
                                      };
    std::map<std::string, float> data;

    // 构造函数：自动初始化map
    ComputeData() {
        for (const std::string& key : order) {
            data[key] = std::numeric_limits<float>::lowest();
        }
    }
};

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

struct ButtonAndSwitch {
    std::map<std::string, bool> buttons;
    std::map<std::string, bool> switches;
    ButtonAndSwitch() {
        buttons = {
            {"acc_engage", false},
            {"acc_disengage", false},
            {"acc_restore", false},
            {"pilot_engage", false},
            {"pilot_disengage", false},
            {"acc_increase", false},
            {"acc_decrease", false}
        };
        switches = {
            {"acc_switch", false},
            {"pilot_switch", false},
            {"noa_switch", false}
        };
    }
};

struct Watchdog {
    std::map<std::string, int> state;
    std::vector<std::string> state_order = {"acc", "pilot", "noa", "aeb", "ldw", "dms", "bsd"};
    Watchdog() {
        state = {
            {"acc", 0},
            {"pilot", 0},
            {"noa", 0},
            {"aeb", 0},
            {"ldw", 0},
            {"dms", 0},
            {"bsd", 0},
        };
    }

    std::string FSMStateToString(int value) {
        FSMState state = static_cast<FSMState>(value);
        switch (state) {
            case UNKNOWN:                          return "UNKNOWN";
            case SelfInspection:                   return "SelfInspection";
            case Fault:                            return "Fault";
            case Off:                              return "Off";
            case Standby:                          return "Standby";
            case Active:                           return "Active";
            case LevelOneAlarming:                 return "LevelOneAlarming";
            case LevelTwoAlarming:                 return "LevelTwoAlarming";
            case LevelThreeAlarming:               return "LevelThreeAlarming";
            case VehicleLevelOneAlarming:          return "VehicleLevelOneAlarming";
            case VehicleLevelTwoAlarming:          return "VehicleLevelTwoAlarming";
            case PedestrianLevelOneAlarming:       return "PedestrianLevelOneAlarming";
            case PedestrianLevelTwoAlarming:       return "PedestrianLevelTwoAlarming";
            case VehicleObstacleAlarmingAndEB:     return "VehicleObstacleAlarmingAndEB";
            case PedestrianObstacleAlarmingAndEB:  return "PedestrianObstacleAlarmingAndEB";
            case LeftDepartureAlarming:            return "LeftDepartureAlarming";
            case RightDepartureAlarming:           return "RightDepartureAlarming";
            case LeftBlindSpotAlarming:            return "LeftBlindSpotAlarming";
            case RightBlindSpotAlarming:           return "RightBlindSpotAlarming";
            case BothSidesBlindSpotAlarming:       return "BothSidesBlindSpotAlarming";
            case NotAvailable:                     return "NotAvailable";
            case STAGE1_SELF_INSPECTION:           return "STAGE1_SELF_INSPECTION";
            case STAGE1_SELF_INSPECTION_FAILED:    return "STAGE1_SELF_INSPECTION_FAILED";
            case STAGE2_SELF_INSPECTION:           return "STAGE2_SELF_INSPECTION";
            case Disabled:                         return "Disabled";
            case MCUTakeover:                      return "MCUTakeover";
            case MCUFault:                         return "MCUFault";
            case NotTrigger:                       return "NotTrigger";
            case BeforeEMP:                        return "BeforeEMP";
            case EMPAvailable:                     return "EMPAvailable";
            case EMPUnavailable:                   return "EMPUnavailable";
            case LeftBlindSpotLevelTwoAlarming:    return "LeftBlindSpotLevelTwoAlarming";
            case LeftBlindSpotLevelThreeAlarming:  return "LeftBlindSpotLevelThreeAlarming";
            case RightBlindSpotLevelTwoAlarming:   return "RightBlindSpotLevelTwoAlarming";
            case RightBlindSpotLevelThreeAlarming: return "RightBlindSpotLevelThreeAlarming";
            default: 
                throw std::invalid_argument("Unknown FSMState value: " + std::to_string(static_cast<int>(state)));
        }
    }
};

struct HmiData {
    std::string local_time;
    ButtonAndSwitch button_switch;
    Watchdog watchdog_state;
};

struct RecordData {
    std::map<std::string, float> data;

    std::vector<std::string> order = {"steer_wheel_angle",
                                      "steer_wheel_torque_filtered",
                                      "wheel_speed",
                                      "yaw_rate",
                                      "ebs_cmd",
                                      "acc_mes",
                                      "acc_ref",
                                      "speed",
                                      "pitch",
                                      "brake_pressure",
                                      "adas_state"};
    RecordData() {
        for (const std::string& key : order) {
            data[key] = 0.0f;
        }
    }

    std::vector<float> getData() {
        std::vector<float> data_vector;
        for (const std::string& key : order) {
            data_vector.push_back(data[key]);
        }
        return data_vector;
    }
};

class MsgParser {
    public:
        MsgParser(int argc, char *argv[]);
        MsgParser();
        ~MsgParser();

    public:
        std::shared_ptr<ComputeData> getVehicleData();
        HmiData getHmiData();

    private:
        void dbw_callback(const std_msgs::String::ConstPtr& msg);
        void ctrl_callback(const std_msgs::String::ConstPtr& msg);
        void watchdog_callback(const std_msgs::String::ConstPtr& msg);
        void writeToCSV(const long long timestamp, const std::vector<float>& data);
        void updateHmiData(const control::DbwReports& dbw_report);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber dbw_sub_;
        ros::Subscriber ctrl_sub_;
        ros::Subscriber watchdog_sub_;
        std::ofstream csv_file_;
        std::string csv_file_path_;
        std::mutex data_mutex_;

    private:   // 数据成员变量
        std::string local_time_;
        std::vector<float> record_data_;
        std::shared_ptr<ComputeData> vehicle_data_;
        std::shared_ptr<RecordData> rec_data_;
        bool first_flag_ = true;
        ButtonAndSwitch button_switch_;
        Watchdog watchdog_state_;
        
    private:  // 后处理数据
        float swa_dot_ = 0.0;
        float swt_filtered_ = 0.0;
        float brake_pressure_filtered_ = 0.0;
};

}
}
