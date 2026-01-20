#include "function/display_control.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "function/animation.h"
#include "tool_box/base_time_struct.h"

#include <matplotlibcpp17/pyplot.h>
#include "algorithm/weighted_window_mode.h"
// #include "function/ros_topic_parser.h"

#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;
using namespace matplotlibcpp17;

using namespace std;
namespace Anim = modules::animation;
namespace AlgWW = ALG::WeightedWindows;
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
  AlgWW::WeightedWindows windows(2000,400);
  pybind11::scoped_interpreter guard{};
  disp_ctrl_ptr->SetParam(argc, argv);
  disp_ctrl_ptr->ExtractData();
  Animator->InitializePlt();
  for (int i = disp_ctrl_ptr->start_index_; i < disp_ctrl_ptr->data_length_;)  //数据行遍历
  {
    //键盘控制
    if (!disp_ctrl_ptr->KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    auto data_row = disp_ctrl_ptr->data_mat_[i];
    string local_time = disp_ctrl_ptr->getLogTimestamp(i);
    auto filter_torque01 = disp_ctrl_ptr->LowPassFilter01(data_row[static_cast<int>(DataIndex::SWT)],0.05);
    auto filter_torque02 = disp_ctrl_ptr->LowPassFilter02(data_row[static_cast<int>(DataIndex::SWT)],0.1);
    data_row.push_back(filter_torque01);
    data_row.push_back(filter_torque02);
    data_row[static_cast<int>(DataIndex::SWA)]*=2;
    auto mode = windows.getWeightedMode(filter_torque02,data_row[static_cast<int>(DataIndex::WHEEL_SPEED)],data_row[static_cast<int>(DataIndex::SWA)]);
    data_row.push_back(mode);
    Animator->SetSteerWheelData(data_row);
    /*------动画显示-----*/
    Animator->Monitor(600,local_time);
    auto freq01 = windows.GetLongFreqency();
    // Animator->BarPlot01(freq01);
    auto freq02 = windows.GetShortFreqency();
    Animator->BarPlot01(freq01,freq02);
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




