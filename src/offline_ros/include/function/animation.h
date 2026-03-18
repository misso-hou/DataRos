#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include "tool_box/singleton.h"
#include "function/animation_functions.h"

namespace modules {
namespace animation {

using namespace std;

class Animation : public utilities::Singleton<Animation>, public AnimationFunctions{
  friend class Singleton<Animation>;

 private:
  Animation() {}
  ~Animation() {}

 public:
  void SetSteerWheelData(const vector<float>& new_data);
  void SetBrakeData(const vector<float>& new_data);
  void SetHmiData(const func::msg_parser::HmiData hmi_data);
  void SWTorqueMonitor(const shared_ptr<func::msg_parser::ComputeData> data,
                       const std::unordered_map<int, int>& freq01,
                       const std::unordered_map<int, int>& freq02);
  void BrakeMonitor(const shared_ptr<func::msg_parser::ComputeData> data);
  void VehicleMonitor(const shared_ptr<func::msg_parser::ComputeData> data);
  void InitWeightedWindowsPlt();
  void InitBrakeSysPlt();
  void InitVehicleMonitor();
};
}  // namespace animation
}  // namespace modules
