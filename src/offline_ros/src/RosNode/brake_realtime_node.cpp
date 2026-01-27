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
#include "algorithm/observer.h"
#include "tool_box/rate_controller.h"

#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;
using namespace matplotlibcpp17;

using namespace std;
namespace Anim = modules::animation;
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
  ros::init(argc, argv, "brake_realtime_module");
  ros::NodeHandle nh;
  // 创建监听器对象
  func::msg_parser::MsgParser msg_parser(argc, argv);
  ALG::BrakeTorqueObserver observer;
  pybind11::scoped_interpreter guard{};
  Animator->InitBrakeSysPlt();
  //主程序线程
  ros::Rate rt(20);
  while (ros::ok()) {
    ros::spinOnce();
    auto realtime_data = msg_parser.getVehicleBrakeData();
    vector<float> data_row(8);
    data_row.at(0) = realtime_data.ebs_cmd;
    data_row.at(1) = realtime_data.acc_mes;
    data_row.at(2) = realtime_data.acc_ref;
    data_row.at(3) = realtime_data.speed;
    data_row.at(4) = realtime_data.pitch;
    data_row.at(5) = realtime_data.brake_pressure_filtered*(-0.01);
    data_row.at(6) = realtime_data.wheel_speed;
    auto brake_gain = observer.estimateBrakeGain(realtime_data.speed,
                                                 realtime_data.acc_mes,
                                                 realtime_data.brake_pressure_filtered);   
    data_row.at(7) = brake_gain*(0.01);
    Animator->SetBrakeData(data_row);
    /*------动画显示-----*/
    Animator->BrakeMonitor(600,realtime_data.local_time);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}




