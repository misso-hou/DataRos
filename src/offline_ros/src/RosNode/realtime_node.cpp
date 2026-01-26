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

using namespace std;
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
  ros::init(argc, argv, "realtime_module");
  ros::NodeHandle nh;
  // 创建监听器对象
  func::msg_parser::MsgParser msg_parser;
  AlgWW::WeightedWindows windows(2000,400);
  pybind11::scoped_interpreter guard{};
  Animator->InitWeightedWindowsPlt();
  //主程序线程
  ros::Rate rt(20);
  while (ros::ok()) {
    ros::spinOnce();
    auto realtime_data = msg_parser.getVehicleSteerData();
    vector<float> data_row(4);
    data_row[0] = realtime_data.steer_wheel_angle;
    data_row[1] = realtime_data.steer_wheel_torque_filtered;
    data_row[2] = realtime_data.wheel_speed;
    data_row[3] = realtime_data.yaw_rate;
    data_row[4] = realtime_data.steer_wheel_angle_dot;
    auto mode = windows.getWeightedMode(realtime_data.steer_wheel_torque_filtered,
                                        realtime_data.wheel_speed,
                                        realtime_data.yaw_rate);
    data_row.push_back(mode);
    Animator->SetSteerWheelData(data_row);
    /*------动画显示-----*/
    Animator->SWTorqueMonitor(600,realtime_data.local_time);
    auto freq01 = windows.GetLongFreqency();
    auto freq02 = windows.GetShortFreqency();
    Animator->BarPlot(freq01,freq02);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}




