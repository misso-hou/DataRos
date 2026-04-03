#include "function/display_control.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "function/animation.h"
#include "tool_box/base_time_struct.h"

#include <matplotlibcpp17/pyplot.h>
#include "algorithm/weighted_window_mode.h"

#include <algorithm>
#include <vector>
#include <iomanip>
#include <tool_box/math_tools.h>

using namespace std;
using namespace matplotlibcpp17;

using namespace std;
using namespace func::msg_parser;

namespace Anim = modules::animation;
namespace AlgWW = ALG::WeightedWindows;
namespace Math = toolbox::math;
Anim::Animation *Animator = Anim::Animation::GetInstance();
std::unique_ptr<DisplayControl> disp_ctrl_ptr = std::make_unique<DisplayControl>();

int main(int argc, char *argv[]) {
  AlgWW::WeightedWindows windows(100,20);
  pybind11::scoped_interpreter guard{};
  disp_ctrl_ptr->SetParam(argc, argv);
  auto record_data = disp_ctrl_ptr->ExtractDataInColumn(argc, argv);
  Animator->InitWeightedWindowsPlt();
  for (int i = disp_ctrl_ptr->start_index_; i < disp_ctrl_ptr->data_length_;)  //数据行遍历
  {
    //键盘控制
    if (!disp_ctrl_ptr->KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    auto vehicle_data = std::make_shared<func::msg_parser::ComputeData>();
    //数据获取
    static float swt_filtered = record_data.at("steer_wheel_torque_filtered")[i];
    swt_filtered = Math::LowPassFilter(record_data.at("steer_wheel_torque_filtered")[i],swt_filtered,0.05);
    float swa_dot = i <= 1 ? 0 : 
                    (record_data.at("steer_wheel_angle")[i] - record_data.at("steer_wheel_angle")[i-1]) / TS;
    vehicle_data->data.at("adas_state") = record_data.at("adas_state")[i];
    vehicle_data->data.at("steer_wheel_angle") = record_data.at("steer_wheel_angle")[i];
    vehicle_data->data.at("steer_wheel_torque_filtered") = swt_filtered;
    vehicle_data->data.at("wheel_speed") = record_data.at("wheel_speed")[i];
    vehicle_data->data.at("yaw_rate") = record_data.at("yaw_rate")[i];
    vehicle_data->data.at("steer_wheel_angle_dot") = swa_dot;
    bool pilot_enabled = (vehicle_data->data.at("adas_state") >= 2);
    auto mode = windows.getWeightedMode(vehicle_data->data.at("steer_wheel_torque_filtered"),
                                        vehicle_data->data.at("wheel_speed"),
                                        vehicle_data->data.at("yaw_rate"),
                                        pilot_enabled);
    vehicle_data->data.at("steer_wheel_torque_mode") = mode;
    vehicle_data->data.at("long_window_mean") = windows.getLongMean();
    vehicle_data->data.at("short_window_mean") = windows.getShortMean();
     /*------动画显示-----*/
     auto freq01 = windows.getLongFreqency();
     auto freq02 = windows.getShortFreqency();
     Animator->SWTorqueMonitor(vehicle_data,freq01,freq02);
    //周期控制
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




