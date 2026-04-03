#include "function/display_control.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "tool_box/base_time_struct.h"

#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;


using namespace std;
using namespace func::msg_parser;

std::unique_ptr<DisplayControl> disp_ctrl_ptr = std::make_unique<DisplayControl>();

int main(int argc, char *argv[]) {
  disp_ctrl_ptr->SetParam(argc, argv);
  auto record_frames = disp_ctrl_ptr->loadFramesData(argc, argv);
  for (int i = disp_ctrl_ptr->start_index_; i < disp_ctrl_ptr->data_length_;)  //数据行遍历
  {
    //键盘控制
    if (!disp_ctrl_ptr->KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    auto frame_data = record_frames[i];
    const auto& vehicle_data = frame_data.vehicle_data();
    float swt = vehicle_data.steer_wheel_torque();
    const auto& status = frame_data.report_status();
    int vehicle_wiper_status = status.vehicle_wiper_status();
    std::cout << "test:" << i << " ; swt:" << swt << std::endl;
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
}