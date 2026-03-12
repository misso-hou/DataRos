#include <ros/ros.h>
#include <ros/package.h>
#include "function/ros_topic_parser.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "function/animation.h"
#include "tool_box/base_time_struct.h"

#include <matplotlibcpp17/pyplot.h>
#include "tool_box/rate_controller.h"

#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;
using namespace matplotlibcpp17;
using namespace func::msg_parser;
namespace Anim = modules::animation;
Anim::Animation *Animator = Anim::Animation::GetInstance();

int main(int argc, char *argv[]) {
  std::cout << "Main thread running..." << std::endl;
  ros::init(argc, argv, "vehicle_monitor_starting");
  ros::NodeHandle nh;
  // 创建监听器对象
  MsgParser msg_parser(argc, argv);
  pybind11::scoped_interpreter guard{};
  Animator->InitVehicleMonitor();
  //主程序线程
  ros::Rate rt(20);
  while (ros::ok()) {
    ros::spinOnce();

    auto realtime_data = msg_parser.getVehicleSteerData();
    vector<float> plt_data(8);
    plt_data.at(0) = realtime_data.steer_wheel_angle;
    plt_data.at(1) = realtime_data.steer_wheel_torque_filtered;
    plt_data.at(2) = realtime_data.wheel_speed;
    plt_data.at(3) = realtime_data.yaw_rate;
    plt_data.at(4) = realtime_data.steer_wheel_angle_dot;
    bool pilot_state = static_cast<bool>(realtime_data.pilot_state);
    plt_data.at(5) = 0.0f;
    plt_data.at(6) = 0.0f;
    plt_data.at(7) = 0.0f;
    Animator->SetSteerWheelData(plt_data);
    // hmi data    
    auto hmi_data = msg_parser.getHmiData();
    Animator->SetHmiData(hmi_data);
    /*------动画显示-----*/
    Animator->VehicleMonitor(plt_data.at(0),pilot_state);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}