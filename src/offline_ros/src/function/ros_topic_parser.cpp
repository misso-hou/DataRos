#include "function/ros_topic_parser.h"
#include <ros/package.h>
#include <tool_box/math_tools.h>

namespace func {
namespace msg_parser {

namespace Math = toolbox::math;

// 设置一个最小阈值，小于该值就写0
const float MIN_WRITE_VALUE = 1e-10f;
float TS = 0.05f;

MsgParser::MsgParser() {
    // 获取包的路径
    std::string package_path = ros::package::getPath("offline_ros");
    // 构造绝对路径
    csv_file_path_ = package_path + "/data/record_data02.csv";
    // 打开CSV文件，清空原有内容（相当于Python中的'w'模式）
    csv_file_.open(csv_file_path_.c_str());
    if (!csv_file_.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_file_path_.c_str());
        return;
    }
    // 可选：写入CSV表头
    csv_file_ << "timestamp,\
                  SWA,\
                  SWT,\
                  WHEEL_SPEED,\
                  YAW_RATE,\
                  EBS_CMD,\
                  ACC_MES,\
                  ACC_REF,\
                  SPEED,\
                  PITCH,\
                  BRAKE_PRESSURE\n";
    record_data_.resize(static_cast<int>(DataIndex::BRAKE_PRESSURE)+1);
    dbw_sub_ = nh_.subscribe("/vehicle/dbw_reports", 1000, &MsgParser::dbw_callback, this);
    ctrl_sub_ = nh_.subscribe("/vehicle/control_cmd", 1000, &MsgParser::ctrl_callback, this);
    ROS_INFO("DBW Reports listener started. Saving data to: %s", csv_file_path_.c_str());
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
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            //steering wheel angle speed calculation
            if(first_flag_){
                first_flag_ = false;
            }else{
                swa_dot_ = (dbw_report.steering_report().steering_wheel_angle() - record_data_[static_cast<int>(DataIndex::SWA)]) / TS;
                std::cout << "debug->current angle:" << dbw_report.steering_report().steering_wheel_angle() << "; last angle:" << record_data_[static_cast<int>(DataIndex::SWA)] <<
                             "; angle rate:" << swa_dot_ << std::endl;
            }
            // realtime data
            record_data_[static_cast<int>(DataIndex::SWA)] = dbw_report.steering_report().steering_wheel_angle();
            record_data_[static_cast<int>(DataIndex::SWT)] = dbw_report.steering_report().steering_wheel_torque();
            record_data_[static_cast<int>(DataIndex::WHEEL_SPEED)] = dbw_report.wheel_speed_report().front_axle_speed();
            record_data_[static_cast<int>(DataIndex::YAW_RATE)] = dbw_report.vehicle_dynamic().angular_velocity().z();
            record_data_[static_cast<int>(DataIndex::BRAKE_PRESSURE)] = dbw_report.brake_msg_3().brake_pressure_front_axle_left_wheel();
            record_data_[static_cast<int>(DataIndex::SPEED)] = dbw_report.steering_report().speed();
            // for display and calculation
            swt_filtered_ = Math::LowPassFilter(record_data_[static_cast<int>(DataIndex::SWT)],0.05);
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

        writeToCSV(ts_msec, record_data_);
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

void MsgParser::ctrl_callback(const std_msgs::String::ConstPtr& msg)
{
    try
    {
        drive::common::control::ControlCommand control_cmd;
        if (!control_cmd.ParseFromString(msg->data))
        {
            ROS_WARN("Failed to parse protobuf message");
            return;
        }
        
        // 提取数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            record_data_[static_cast<int>(DataIndex::EBS_CMD)] = control_cmd.brake_cmd().target_acceleration();
            record_data_[static_cast<int>(DataIndex::ACC_MES)] = control_cmd.debug_cmd().a_report();
            record_data_[static_cast<int>(DataIndex::ACC_REF)] = control_cmd.debug_cmd().a_target();
            record_data_[static_cast<int>(DataIndex::PITCH)] =  control_cmd.debug_cmd().pitch_angle();
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

VehicleSteerData MsgParser::getVehicleSteerData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            record_data_[static_cast<int>(DataIndex::SWA)],
            swt_filtered_,
            record_data_[static_cast<int>(DataIndex::WHEEL_SPEED)],
            record_data_[static_cast<int>(DataIndex::YAW_RATE)],
            swa_dot_};
}

VehicleBrakeData MsgParser::getVehicleBrakeData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {local_time_,
            record_data_[static_cast<int>(DataIndex::EBS_CMD)],
            record_data_[static_cast<int>(DataIndex::ACC_MES)],
            record_data_[static_cast<int>(DataIndex::ACC_REF)],
            record_data_[static_cast<int>(DataIndex::SPEED)],
            record_data_[static_cast<int>(DataIndex::PITCH)],
            record_data_[static_cast<int>(DataIndex::BRAKE_PRESSURE)]};
}

}
} // namespace name