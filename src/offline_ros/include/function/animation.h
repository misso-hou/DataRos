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
  void SWTorqueMonitor(const string& time, 
                       const float& angle, 
                       const bool pilot,
                       const std::unordered_map<int, int>& freq01,
                       const std::unordered_map<int, int>& freq02);
  void BrakeMonitor(const string& time,const float& angle, const bool pilot);
  void VehicleMonitor(const float& angle, const bool pilot);
  void InitWeightedWindowsPlt();
  void InitBrakeSysPlt();
  void InitVehicleMonitor();
};
}  // namespace animation
}  // namespace modules
