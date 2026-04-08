#include "function/display_control.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "function/animation.h"
#include "tool_box/base_time_struct.h"

#include <matplotlibcpp17/pyplot.h>
#include "algorithm/observer.h"

#include <algorithm>
#include <vector>
#include <iomanip>
#include <tool_box/math_tools.h>

using namespace std;
using namespace matplotlibcpp17;
using namespace func::msg_parser;
namespace Anim = modules::animation;
namespace Math = toolbox::math;
Anim::Animation *Animator = Anim::Animation::GetInstance();
std::unique_ptr<DisplayControl> disp_ctrl_ptr = std::make_unique<DisplayControl>();

/*
 * ---------数据回放使用方法----------：
 * 执行命令：./csvPlt+"播放速度设置“+”播放位置设置“+”csv文件夹序号“+”csv文件夹内部文件序号“
 * 播放速度默认为1
 * 播放位置默认从头开始
 * 文件夹默认为csv文件不带后缀序号
 * csv文件内部默认只有一组数据
 */
int main(int argc, char *argv[]) {
  ALG::BrakeTorqueObserver observer;
  pybind11::scoped_interpreter guard{};
  disp_ctrl_ptr->SetParam(argc, argv);
  auto record_data = disp_ctrl_ptr->ExtractDataInColumn(argc, argv);
  Animator->InitBrakeSysPlt();
  for (int i = disp_ctrl_ptr->start_index_; i < disp_ctrl_ptr->data_length_;)  //数据行遍历
  {
    //键盘控制
    if (!disp_ctrl_ptr->KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    auto vehicel_data = std::make_shared<func::msg_parser::ComputeData>();
    //数据获取
    vehicel_data->local_time = disp_ctrl_ptr->getLogTimestamp(i);
    vehicel_data->data.at("ebs_cmd") = record_data.at("ebs_cmd")[i];
    vehicel_data->data.at("acc_mes") = record_data.at("acc_mes")[i];
    vehicel_data->data.at("acc_ref") = record_data.at("acc_ref")[i];
    vehicel_data->data.at("speed") = record_data.at("speed")[i];
    vehicel_data->data.at("pitch") = record_data.at("pitch")[i];
    vehicel_data->data.at("wheel_speed") = record_data.at("wheel_speed")[i];
    static float filtered_bp = record_data.at("brake_pressure")[i];
    filtered_bp = Math::LowPassFilter(record_data.at("brake_pressure")[i],filtered_bp,0.1);
    auto brake_gain = observer.estimateBrakeGain(vehicel_data->data.at("speed"),
                                                 vehicel_data->data.at("acc_mes"),
                                                 filtered_bp);
    vehicel_data->data.at("brake_pressure_filtered") = filtered_bp*(-0.01);
    vehicel_data->data.at("brake_gain") = brake_gain*(0.01);
    vehicel_data->data.at("steer_wheel_angle") = record_data.at("steer_wheel_angle")[i];
    /*------动画显示-----*/
    Animator->BrakeMonitor(vehicel_data);
    int64_t end_time = TimeToolKit::TimeSpecSysCurrentMs();
    int64_t remaining_T = disp_ctrl_ptr->cycle_time_ - (end_time - start_time);
    if (remaining_T > 0) {
      this_thread::sleep_for(chrono::milliseconds(remaining_T));
    }
    if (!disp_ctrl_ptr->back_) i++;
    disp_ctrl_ptr->erase_ = false;
  }
  return 0;
  pybind11::finalize_interpreter();
}




