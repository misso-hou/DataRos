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
#include "algorithm/weighted_window_mode.h"
#include "tool_box/rate_controller.h"

#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;
using namespace matplotlibcpp17;
using namespace func::msg_parser;
namespace Anim = modules::animation;
namespace AlgWW = ALG::WeightedWindows;
Anim::Animation *Animator = Anim::Animation::GetInstance();


/*
 * ---------数据回放使用方法----------：
 * 执行命令：./csvPlt+"播放速度设置“+”播放位置设置“+”csv文件夹序号“+”csv文件夹内部文件序号“
 * 播放速度默认为1
 * 播放位置默认从头开始
 * 文件夹默认为csv文件不带后缀序号
 * csv文件内部默认只有一组数据
 */
int main(int argc, char *argv[]) {
  std::cout << "Main thread running..." << std::endl;
  ros::init(argc, argv, "steer_realtime_module");
  ros::NodeHandle nh;
  // 创建监听器对象
  MsgParser msg_parser(argc, argv);
  AlgWW::WeightedWindows windows(100,20);
  pybind11::scoped_interpreter guard{};
  Animator->InitWeightedWindowsPlt();
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
    auto mode = windows.getWeightedMode(realtime_data.steer_wheel_torque_filtered,
                                        realtime_data.wheel_speed,
                                        realtime_data.yaw_rate,
                                        pilot_state);
    plt_data.at(5) = mode;
    plt_data.at(6) = windows.getLongMean();
    plt_data.at(7) = windows.getShortMean();
    Animator->SetSteerWheelData(plt_data);
    /*------动画显示-----*/
    Animator->SWTorqueMonitor(600,realtime_data.local_time);
    auto freq01 = windows.getLongFreqency();
    auto freq02 = windows.getShortFreqency();
    // Animator->BarPlot(freq01,freq02); //REVIEW:crash
    Animator->SteeringWheelMonitor(realtime_data.steer_wheel_angle,pilot_state);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}




