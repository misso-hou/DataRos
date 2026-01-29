#pragma once

#include <iostream>
#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/figure.h>
#include <matplotlibcpp17/pyplot.h>
#include <memory>
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <vector>


#include "tool_box/base_time_struct.h"
#include "tool_box/singleton.h"

namespace modules {
namespace animation {

using namespace std;
namespace py = pybind11;
namespace mpl = matplotlibcpp17;
using matplotlibcpp17::gridspec::GridSpec;

using mesh2D = vector<vector<float>>;

class Animation : public utilities::Singleton<Animation> {
  friend class Singleton<Animation>;

 private:
  Animation()  {}
  ~Animation() {}

 public:
  void SetSteerWheelData(const vector<float>& new_data);
  void SetBrakeData(const vector<float>& new_data);
  void SWTorqueMonitor(const string& time, 
                       const float& angle, 
                       const bool pilot,
                       const std::unordered_map<int, int>& freq01,
                       const std::unordered_map<int, int>& freq02);
  void BrakeMonitor(const string& time);
  void InitWeightedWindowsPlt();
  void InitBrakeSysPlt();

 private:
  bool FrequencyCtrl(int T, int64_t& last_time_stamp);
  void SWTorquePltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range);
  void BrakePltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range);
  void drawSteeringData(const string& time);
  void drawSteeringWheel(const float& angle,const bool pilot);
  void drawBarPlot(const std::unordered_map<int, int>& frequency01,const std::unordered_map<int, int>& frequency02);

 private:
  //画框
  mpl::pyplot::PyPlot data_plt_;
  //轴系
  shared_ptr<mpl::gridspec::GridSpec> data_gs_ptr_;
  shared_ptr<mpl::axes::Axes> data_axes01_ptr_;
  shared_ptr<mpl::axes::Axes> data_axes02_ptr_;
  shared_ptr<mpl::axes::Axes> data_axes03_ptr_;
  shared_ptr<mpl::axes::Axes> bar_axes_ptr_;
  shared_ptr<mpl::axes::Axes> steering_wheel_axes_ptr_;
  // figure
  shared_ptr<mpl::figure::Figure> data_figure_ptr_;
  shared_ptr<mpl::figure::Figure> bar_figure_ptr_;
  shared_ptr<mpl::figure::Figure> steering_wheel_figure_ptr_;
  // background
  py::object data_background_;
  py::object bar_background_;
  py::object steering_wheel_background_;
  py::object jet_cmap_;

  // data
  vector<float> steer_wheel_plt_data_;
  vector<float> brake_plt_data_;

};
}  // namespace animation
}  // namespace modules
