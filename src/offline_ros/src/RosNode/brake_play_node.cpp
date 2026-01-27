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
  auto data_mat2D = disp_ctrl_ptr->ExtractData();
  Animator->InitBrakeSysPlt();
  for (int i = disp_ctrl_ptr->start_index_; i < data_mat2D.size();)  //数据行遍历
  {
    //键盘控制
    if (!disp_ctrl_ptr->KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    //数据获取
    string local_time = disp_ctrl_ptr->getLogTimestamp(i);
    auto data_row = data_mat2D[i];
    static float filtered_bp = data_row[to_int(DataIndex::BRAKE_PRESSURE)];
    filtered_bp = Math::LowPassFilter(data_row[to_int(DataIndex::BRAKE_PRESSURE)],filtered_bp,0.1);
    data_row[to_int(DataIndex::BRAKE_PRESSURE)] = filtered_bp;
    auto brake_gain = observer.estimateBrakeGain(data_row[to_int(DataIndex::SPEED)],
                                                 data_row[to_int(DataIndex::ACC_MES)],
                                                 data_row[to_int(DataIndex::BRAKE_PRESSURE)]);                                          
    vector<float> plt_data(8);
    plt_data.at(0) = data_row.at(to_int(DataIndex::EBS_CMD));
    plt_data.at(1) = data_row.at(to_int(DataIndex::ACC_MES));
    plt_data.at(2) = data_row.at(to_int(DataIndex::ACC_REF));
    plt_data.at(3) = data_row.at(to_int(DataIndex::SPEED));
    plt_data.at(4) = data_row.at(to_int(DataIndex::PITCH));
    plt_data.at(5) = data_row.at(to_int(DataIndex::BRAKE_PRESSURE))*(-0.01);
    plt_data.at(6) = data_row.at(to_int(DataIndex::WHEEL_SPEED));
    plt_data.at(7) = brake_gain*(0.01);
    Animator->SetBrakeData(plt_data);
    /*------动画显示-----*/
    Animator->BrakeMonitor(600,local_time);
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




