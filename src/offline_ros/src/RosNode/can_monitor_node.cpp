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
    PlotCanMsg plot_can01(0x04F0090B,46,2,0);
    msg_parser.getCanSignal(plot_can01);
    plot_can_array.push_back(plot_can01);
    PlotCanMsg plot_can02(0x09FF75F8,46,2,0);
    msg_parser.getCanSignal(plot_can02);
    plot_can_array.push_back(plot_can02);
    /*------动画显示-----*/
    Animator->CanMonitor(plot_can_array);
    rt.sleep();
  }
  pybind11::finalize_interpreter();
  ROS_INFO("实时模块正常退出");
  return 0;
}




