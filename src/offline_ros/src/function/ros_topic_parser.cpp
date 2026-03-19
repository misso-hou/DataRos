#include "function/ros_topic_parser.h"
#include <ros/package.h>
#include <tool_box/math_tools.h>

namespace func {
namespace msg_parser {

namespace Math = toolbox::math;

// 设置一个最小阈值，小于该值就写0
const float MIN_WRITE_VALUE = 1e-10f;
float TS = 0.05f;

MsgParser::MsgParser(int argc, char *argv[]) {
    vehicle_data_ = std::make_shared<ComputeData>();
    rec_data_ = std::make_shared<RecordData>();
    // 获取包的路径
    std::string package_path = ros::package::getPath("offline_ros");
    // 构造绝对路径
    std::string filename;
    if(argc <2){
      throw std::runtime_error("!!!!INPUT CSV FILE NAME!!!!");
    }else {
      filename = argv[1];
    }
    csv_file_path_ = package_path + "/data/" + filename + ".csv";
    // 打开CSV文件，清空原有内容（相当于Python中的'w'模式）
    csv_file_.open(csv_file_path_.c_str());
    if (!csv_file_.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_file_path_.c_str());
        return;
    }
    // 可选：写入CSV表头
    csv_file_ << "timestamp" << ",";
    for(const auto& key:rec_data_->order){
        csv_file_ << key << ",";
    }
    csv_file_ << "\n";
    dbw_sub_ = nh_.subscribe("/vehicle/dbw_reports", 1000, &MsgParser::dbw_callback, this);
    ctrl_sub_ = nh_.subscribe("/vehicle/control_cmd", 1000, &MsgParser::ctrl_callback, this);
    watchdog_sub_ = nh_.subscribe("/watchdog/current_state",1000, &MsgParser::watchdog_callback, this);
    ROS_INFO("DBW Reports listener started. Saving data to: %s", csv_file_path_.c_str());
}

MsgParser::MsgParser() {
    dbw_sub_ = nh_.subscribe("/vehicle/dbw_reports", 1000, &MsgParser::dbw_callback, this);
    ctrl_sub_ = nh_.subscribe("/vehicle/control_cmd", 1000, &MsgParser::ctrl_callback, this);
    watchdog_sub_ = nh_.subscribe("/watchdog/current_state",1000, &MsgParser::watchdog_callback, this);
}

MsgParser::~MsgParser()
{
    if (csv_file_.is_open())
    {
        csv_file_.close();
    }
}

void MsgParser::dbw_callback(const std_msgs::String::ConstPtr& msg)
{
    // 解析protobuf消息
    control::DbwReports dbw_report;
    if (!dbw_report.ParseFromString(msg->data))
    {
        ROS_WARN("Failed to parse DbwReports message");
        return;
    }
    
    // 提取数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        //steering wheel angle speed calculation
        if(first_flag_){
            first_flag_ = false;
            swt_filtered_ = rec_data_->data.at("steer_wheel_torque_filtered");
            brake_pressure_filtered_ = rec_data_->data.at("brake_pressure");
        }else{
            swa_dot_ = (dbw_report.steering_report().steering_wheel_angle() - rec_data_->data.at("steer_wheel_angle")) / TS;
        }
        // realtime data
        rec_data_->data.at("steer_wheel_angle") = dbw_report.steering_report().steering_wheel_angle();
        rec_data_->data.at("steer_wheel_torque_filtered") = dbw_report.steering_report().steering_wheel_torque();
        rec_data_->data.at("wheel_speed") = dbw_report.wheel_speed_report().front_axle_speed();
        rec_data_->data.at("yaw_rate") = dbw_report.vehicle_dynamic().angular_velocity().z();
        rec_data_->data.at("brake_pressure") = dbw_report.brake_msg_3().brake_pressure_front_axle_left_wheel();
        rec_data_->data.at("speed") = dbw_report.steering_report().speed();
        rec_data_->data.at("adas_state") = dbw_report.superpilot_enabled();
        // for display and calculation
        swt_filtered_ = Math::LowPassFilter(rec_data_->data.at("steer_wheel_torque_filtered"),swt_filtered_,0.05);
        brake_pressure_filtered_ = Math::LowPassFilter(rec_data_->data.at("brake_pressure"),brake_pressure_filtered_,0.05);
        // HMI data -> buttons and switches
        updateHmiData(dbw_report);
    }

    // 提取→转秒→转时间
    long long ts_msec = dbw_report.header().timestamp_msec(); // 提取原始毫秒戳
    time_t timestamp_ms = static_cast<time_t>(ts_msec);
    // 分离秒和毫秒部分
    long long seconds = timestamp_ms / 1000;      // 整数秒部分
    long long milliseconds = timestamp_ms % 1000; // 毫秒部分（0-999）
    // 转换为UTC时间（秒部分）
    time_t raw_sec = static_cast<time_t>(seconds);
    struct tm t;
    gmtime_r(&raw_sec, &t);
    // 时区修正：UTC+8（北京时间）
    int beijing_hour = (t.tm_hour + 8) % 24;
    // 格式化为时分秒.毫秒
    local_time_ = std::to_string(beijing_hour/10) + std::to_string(beijing_hour%10) + ":" +
                    std::to_string(t.tm_min/10) + std::to_string(t.tm_min%10) + ":" +
                    std::to_string(t.tm_sec/10) + std::to_string(t.tm_sec%10) + "." +
                    std::to_string(milliseconds/100) + 
                    std::to_string((milliseconds/10)%10) + 
                    std::to_string(milliseconds%10);

    auto record_data = rec_data_->getData();

    writeToCSV(ts_msec, record_data);
}

void MsgParser::updateHmiData(const control::DbwReports& dbw_report) {
    button_switch_.buttons.at("acc_engage") = dbw_report.hmi_report().acc_engage_button_pressed();
    button_switch_.buttons.at("acc_disengage") = dbw_report.hmi_report().acc_disengage_button_pressed();
    button_switch_.buttons.at("acc_restore") = dbw_report.hmi_report().acc_restore_button_pressed();
    button_switch_.buttons.at("pilot_engage") = dbw_report.hmi_report().super_pilot_engage_button_pressed();
    button_switch_.buttons.at("pilot_disengage") = dbw_report.hmi_report().super_pilot_disengage_button_pressed();
    button_switch_.buttons.at("acc_increase") = dbw_report.hmi_report().acc_set_inc_button_pressed();
    button_switch_.buttons.at("acc_decrease") = dbw_report.hmi_report().acc_set_dec_button_pressed();
    button_switch_.switches.at("acc_switch") = dbw_report.hmi_report().acc_switch();
    button_switch_.switches.at("pilot_switch") = dbw_report.hmi_report().pilot_switch();
    button_switch_.switches.at("noa_switch") = dbw_report.hmi_report().noa_switch_button_pressed();
}

void MsgParser::ctrl_callback(const std_msgs::String::ConstPtr& msg)
{
    control::ControlCommand control_cmd;
    if (!control_cmd.ParseFromString(msg->data))
    {
        ROS_WARN("Failed to parse ControlCommand message");
        return;
    }
    // 提取数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rec_data_->data.at("ebs_cmd") = control_cmd.brake_cmd().target_acceleration();
        rec_data_->data.at("acc_mes") = control_cmd.debug_cmd().a_report();
        rec_data_->data.at("acc_ref") = control_cmd.debug_cmd().a_target();
        rec_data_->data.at("pitch") =  control_cmd.debug_cmd().pitch_angle();
    }
}

void MsgParser::watchdog_callback(const std_msgs::String::ConstPtr& msg)
{
    monitor::AppWatchdogState watchdog;
    if (!watchdog.ParseFromString(msg->data))
    {
        ROS_WARN("Failed to parse watchdog message");
        return;
    }
    // 提取数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        watchdog_state_.state.at("acc") = watchdog.acc_state();
        watchdog_state_.state.at("pilot") = watchdog.super_pilot_state();
        watchdog_state_.state.at("noa") = watchdog.noa_state();
        watchdog_state_.state.at("aeb") = watchdog.acc_state();
        watchdog_state_.state.at("ldw") = watchdog.acc_state();
        watchdog_state_.state.at("dms") = watchdog.dms_state();
        watchdog_state_.state.at("bsd") = watchdog.bsd_state();
    }
}

void MsgParser::writeToCSV(const long long timestamp, const std::vector<float>& data) {
    if (!csv_file_.is_open())
    {
        ROS_ERROR("CSV file is not open!");
        return;
    }
    // 写入时间戳
    csv_file_ << timestamp;
    // 写入vector中的每个数据
    for (const auto& value : data)
    {
        csv_file_ << ",";
        if (std::abs(value) < MIN_WRITE_VALUE) {
            csv_file_ << "0.0";
        } else {
            csv_file_ << std::fixed << std::setprecision(4) << value;
        }
    }
    csv_file_ << "\n";
    csv_file_.flush();  // 可选：确保数据写入磁盘
}

std::shared_ptr<ComputeData> MsgParser::getVehicleData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    vehicle_data_->local_time = local_time_;
    //steering data
    vehicle_data_->data.at("steer_wheel_angle") = rec_data_->data.at("steer_wheel_angle");
    vehicle_data_->data.at("steer_wheel_torque_filtered") = swt_filtered_;
    vehicle_data_->data.at("wheel_speed") = rec_data_->data.at("wheel_speed");
    vehicle_data_->data.at("yaw_rate") = rec_data_->data.at("yaw_rate");
    vehicle_data_->data.at("steer_wheel_angle_dot") = swa_dot_;
    //brake data
    vehicle_data_->data.at("ebs_cmd") = rec_data_->data.at("ebs_cmd");
    vehicle_data_->data.at("acc_mes") = rec_data_->data.at("acc_mes");
    vehicle_data_->data.at("acc_ref") = rec_data_->data.at("acc_ref");
    vehicle_data_->data.at("speed") = rec_data_->data.at("speed");
    vehicle_data_->data.at("pitch") = rec_data_->data.at("pitch");
    vehicle_data_->data.at("brake_pressure_filtered") = brake_pressure_filtered_;
    return vehicle_data_;
}

VehicleSteerData MsgParser::getVehicleSteerData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            record_data_[to_int(DataIndex::steer_wheel_angle)],
            swt_filtered_,
            record_data_[to_int(DataIndex::wheel_speed)],
            record_data_[to_int(DataIndex::yaw_rate)],
            swa_dot_,
            static_cast<bool>(record_data_[to_int(DataIndex::adas_state)])};
}

VehicleBrakeData MsgParser::getVehicleBrakeData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            record_data_[to_int(DataIndex::ebs_cmd)],
            record_data_[to_int(DataIndex::acc_mes)],
            record_data_[to_int(DataIndex::acc_ref)],
            record_data_[to_int(DataIndex::speed)],
            record_data_[to_int(DataIndex::pitch)],
            brake_pressure_filtered_,
            record_data_[to_int(DataIndex::wheel_speed)]};
}

HmiData MsgParser::getHmiData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            button_switch_,
            watchdog_state_};
}


}
} // namespace name