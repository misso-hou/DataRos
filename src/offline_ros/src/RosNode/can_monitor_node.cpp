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

using namespace std;
namespace Anim = modules::animation;
Anim::Animation *Animator = Anim::Animation::GetInstance();

int main(int argc, char *argv[]) {
  using namespace func::msg_parser;
  std::cout << "Main thread running..." << std::endl;
  ros::init(argc, argv, "can_monitor_module");
  ros::NodeHandle nh;
  // 创建监听器对象
  MsgParser msg_parser(argc, argv);
  pybind11::scoped_interpreter guard{};
  Animator->InitCanMsgMonitor();
  //主程序线程
  ros::Rate rt(20);
  while (ros::ok()) {
    ros::spinOnce();
    vector<PlotCanMsg> plot_can_array;
    PlotCanMsg plot_can01(0x18FF41ED,0,16,"PlusPilot_char_display");
    msg_parser.getCanSignal(plot_can01);
    plot_can_array.push_back(plot_can01);
    PlotCanMsg plot_can02(0x18F0A1ED,36,2,"lateral_function_status");
    msg_parser.getCanSignal(plot_can02);
    plot_can_array.push_back(plot_can02);
    PlotCanMsg plot_can03(0x18F0A1ED,40,4,"NOA_status");
    msg_parser.getCanSignal(plot_can03);
    plot_can_array.push_back(plot_can03);
    PlotCanMsg plot_can04(0x18FF28ED,0,8,"MessageDisplayRequest");
    msg_parser.getCanSignal(plot_can04);
    plot_can_array.push_back(plot_can04);
    PlotCanMsg plot_can05(0x0CFDCC21,8,4,"TurnSignalSw");
    msg_parser.getCanSignal(plot_can05);
    plot_can_array.push_back(plot_can05);
    PlotCanMsg plot_can06(0x18FF3921,6,2,"LeftTurnSignalLightsSts");
    msg_parser.getCanSignal(plot_can06);
    plot_can_array.push_back(plot_can06);
    PlotCanMsg plot_can07(0x18FF3921,8,2,"RightTurnSignalLightsSts");
    msg_parser.getCanSignal(plot_can07);
    plot_can_array.push_back(plot_can07);
    PlotCanMsg plot_can08(0x18FF0E7B,0,2,"FMS_WarningLevel");
    msg_parser.getCanSignal(plot_can08);
    plot_can_array.push_back(plot_can08);
    PlotCanMsg plot_can09(0x18FF0E7B,2,4,"FMS_WarningType");
    msg_parser.getCanSignal(plot_can09);
    plot_can_array.push_back(plot_can09);
    /*------动画显示-----*/
    Animator->CanMonitor(plot_can_array);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}




