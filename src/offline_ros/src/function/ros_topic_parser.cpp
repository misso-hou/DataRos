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
    // 初始化proto对象
    frame_data_ = std::make_shared<ProtoRecordData::FrameData>();
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

    // 改为.proto文件（二进制格式）
    proto_file_path_ = package_path + "/data/" + filename + ".proto_data";
    proto_file_.open(proto_file_path_, std::ios::out | std::ios::binary);
    if (!proto_file_.is_open()) {
        ROS_ERROR("Failed to open proto file: %s", proto_file_path_.c_str());
        return;
    }

    dbw_sub_ = nh_.subscribe("/vehicle/dbw_reports", 1000, &MsgParser::dbw_callback, this);
    ctrl_sub_ = nh_.subscribe("/vehicle/control_cmd", 1000, &MsgParser::ctrl_callback, this);
    watchdog_sub_ = nh_.subscribe("/watchdog/current_state",1000, &MsgParser::watchdog_callback, this);
    status_report_sub_ = nh_.subscribe("/vehicle/status_report",1000, &MsgParser::statusReport_callback, this);
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
    // updataFrameData(dbw_report); //proto数据
    updataCsvData(dbw_report);   //csv数据
}

void MsgParser::updataFrameData(const control::DbwReports& dbw_report){
    std::lock_guard<std::mutex> lock(data_mutex_);
    //steering wheel angle speed calculation
    if(first_flag_){
        first_flag_ = false;
    }else{
        swa_dot_ = (dbw_report.steering_report().steering_wheel_angle() - dbw_report.steering_report().steering_wheel_angle()) / TS;
    }
    // for display and calculation
    swt_filtered_ = Math::LowPassFilter(dbw_report.steering_report().steering_wheel_torque(),swt_filtered_,0.05);
    brake_pressure_filtered_ = Math::LowPassFilter(dbw_report.brake_msg_3().brake_pressure_front_axle_left_wheel(),brake_pressure_filtered_,0.05);
    frame_data_->set_timestamp_msec(dbw_report.header().timestamp_msec());
    //realtime data
    auto* realtime_data = frame_data_->mutable_vehicle_data();
    realtime_data->set_steer_wheel_angle(dbw_report.steering_report().steering_wheel_angle());
    realtime_data->set_steer_wheel_torque(dbw_report.steering_report().steering_wheel_torque());
    realtime_data->set_wheel_speed(dbw_report.wheel_speed_report().front_axle_speed());
    realtime_data->set_yaw_rate(dbw_report.vehicle_dynamic().angular_velocity().z());
    realtime_data->set_brake_pressure(dbw_report.brake_msg_3().brake_pressure_front_axle_left_wheel());
    realtime_data->set_speed(dbw_report.steering_report().speed());
    realtime_data->set_acc_enable(dbw_report.acc_enabled());
    realtime_data->set_pilot_enable(dbw_report.superpilot_enabled());
    //computed data
    auto* computed_data = frame_data_->mutable_computed_data();
    computed_data->set_steer_wheel_torque_filtered(swt_filtered_);
    computed_data->set_brake_pressure_filtered(brake_pressure_filtered_);
    computed_data->set_steer_wheel_angle_dot(swa_dot_);
    //hmi data
    auto* hmi = frame_data_->mutable_hmi_data();
    auto* button = hmi->mutable_buttons();
    button->set_acc_engage(dbw_report.hmi_report().acc_engage_button_pressed());
    button->set_acc_disengage(dbw_report.hmi_report().acc_disengage_button_pressed());
    button->set_acc_restore(dbw_report.hmi_report().acc_restore_button_pressed());
    button->set_pilot_engage(dbw_report.hmi_report().super_pilot_engage_button_pressed());
    button->set_pilot_disengage(dbw_report.hmi_report().super_pilot_disengage_button_pressed());
    button->set_acc_increase(dbw_report.hmi_report().acc_set_inc_button_pressed());
    button->set_acc_decrease(dbw_report.hmi_report().acc_set_dec_button_pressed());
    auto* switch_pb = hmi->mutable_switches();
    switch_pb->set_acc_switch(dbw_report.hmi_report().acc_switch());
    switch_pb->set_pilot_switch(dbw_report.hmi_report().pilot_switch());
    switch_pb->set_noa_switch(dbw_report.hmi_report().noa_switch_button_pressed());
    //记录数据
    writeFrameToFile(*frame_data_);
    // 提取→转秒→转时间
    long long ts_msec = dbw_report.header().timestamp_msec(); // 提取原始毫秒戳
    updateLocalTime(ts_msec);
}

void MsgParser::updataCsvData(const control::DbwReports& dbw_report){
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
    rec_data_->data.at("gas_pedal") = dbw_report.throttle_report().pedal_input();
    rec_data_->data.at("brake_pedal") = dbw_report.brake_report().pedal_input();
    if(dbw_report.superpilot_enabled()){
        rec_data_->data.at("adas_state") = 2;
    }else if(dbw_report.acc_enabled()){
        rec_data_->data.at("adas_state") = 1;
    }else{
        rec_data_->data.at("adas_state") = 0;
    }
    // for display and calculation
    swt_filtered_ = Math::LowPassFilter(rec_data_->data.at("steer_wheel_torque_filtered"),swt_filtered_,0.05);
    brake_pressure_filtered_ = Math::LowPassFilter(rec_data_->data.at("brake_pressure"),brake_pressure_filtered_,0.05);
    // HMI data -> buttons and switches
    updateHmiData(dbw_report);
    // 提取→转秒→转时间
    long long ts_msec = dbw_report.header().timestamp_msec(); // 提取原始毫秒戳
    updateLocalTime(ts_msec);
    auto RecordData = rec_data_->getData();
    writeToCSV(ts_msec, RecordData);
}

void MsgParser::updateLocalTime(const long long& ts_msec){
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
}

void MsgParser::writeFrameToFile(const ProtoRecordData::FrameData& frame) {
    if (!proto_file_.is_open()) return;
    
    // 序列化
    std::string serialized_data;
    if (!frame.SerializeToString(&serialized_data)) {
        ROS_ERROR("Failed to serialize frame data");
        return;
    }
    
    // 写入长度前缀 + 数据
    uint32_t size = serialized_data.size();
    proto_file_.write(reinterpret_cast<const char*>(&size), sizeof(size));
    proto_file_.write(serialized_data.data(), size);
    
    // 可选：定期flush
    static int frame_count = 0;
    if (++frame_count % 100 == 0) {
        proto_file_.flush();
    }
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
        rec_data_->data.at("acc_mes") = control_cmd.debug_cmd().a_report();
        rec_data_->data.at("acc_ref") = control_cmd.debug_cmd().a_target();
        rec_data_->data.at("pitch") =  control_cmd.debug_cmd().pitch_angle();

        //realtime data(control)
        auto* realtime_data = frame_data_->mutable_vehicle_data();
        realtime_data->set_acc_mes(control_cmd.debug_cmd().a_report());
        realtime_data->set_acc_ref(control_cmd.debug_cmd().a_target());
        realtime_data->set_pitch(control_cmd.debug_cmd().pitch_angle());
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

void MsgParser::statusReport_callback(const std_msgs::String::ConstPtr& msg)
{
    m_msg::StatusReport status_report;
    if (!status_report.ParseFromString(msg->data))
    {
        ROS_WARN("Failed to parse status_report message");
        return;
    }
    // 提取数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (auto vehicle_state : status_report.vehicle_states()) {
            auto enum_type = vehicle_state.state_type();
            if (enum_type == m_msg::VehicleState_StateType_VEHICLE_WIPER_STATUS) {
                status_report_.status.at("VEHICLE_WIPER_STATUS") = vehicle_state.value();
            }
            if (enum_type == m_msg::VehicleState_StateType_BSD_RIGHT_LED_NA) {
                status_report_.status.at("BSD_RIGHT_LED_NA") = vehicle_state.value();
            }
            if (enum_type == m_msg::VehicleState_StateType_BSD_LEFT_LED_NA) {
                status_report_.status.at("BSD_LEFT_LED_NA") = vehicle_state.value();
            }
            if (enum_type == m_msg::VehicleState_StateType_BSD_SWITCH_ERR) {
                status_report_.status.at("BSD_SWITCH_ERR") = vehicle_state.value();
            }
        }
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
    vehicle_data_->data.at("adas_state") = rec_data_->data.at("adas_state");
    vehicle_data_->data.at("gas_pedal") = rec_data_->data.at("gas_pedal");
    vehicle_data_->data.at("brake_pedal") = rec_data_->data.at("brake_pedal");
    return vehicle_data_;
}

HmiData MsgParser::getHmiData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            button_switch_,
            watchdog_state_,
            status_report_};
}

}
} // namespace name