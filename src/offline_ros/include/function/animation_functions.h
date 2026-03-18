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
#include "function/ros_topic_parser.h"

namespace modules {
namespace animation {

using namespace std;
namespace py = pybind11;
namespace mpl = matplotlibcpp17;
using matplotlibcpp17::gridspec::GridSpec;
using mesh2D = vector<vector<float>>;

const float CMD_X_RANGE = 100;
const int Y_RANGE = 10;
const float BAR_X = 3.0;
const int DATA_BUFFER = 300;

const vector<string> COLORS = {
  // 基本颜色
  "red",        // 红色
  "orange",     // 橙色
  "yellow",     // 黄色
  "green",      // 绿色
  "cyan",       // 青色
  "blue",       // 蓝色
  "purple",     // 紫色
  "magenta",    // 洋红/品红
  "lime",       // 亮绿色
  "navy",       // 深蓝色
  "teal",       // 蓝绿色
  "maroon",     // 褐红色/栗色
  "olive",      // 橄榄绿
  "fuchsia",    // 紫红色
  "aqua",       // 浅青色
  "indigo",     // 靛蓝色
  "violet",     // 紫罗兰色
  "coral",      // 珊瑚色
  "gold",       // 金色
  "tomato",     // 番茄红
  "chocolate",  // 巧克力色
  "sienna",     // 赭色
  "slategray",  // 石板灰
  "darkgreen",  // 深绿色
  "darkblue",   // 深蓝色
  "darkred",    // 深红色
  "darkcyan",   // 深青色
  "darkmagenta",// 深洋红
  "darkorange", // 深橙色
  "lightcoral", // 浅珊瑚色
  "lightblue",  // 浅蓝色
  "lightgreen", // 浅绿色
};

class AnimationFunctions {
  protected:
    AnimationFunctions() {}
    ~AnimationFunctions() {}

  protected:
    bool frequencyCtrl(int T, int64_t& last_time_stamp);
    void drawSteeringData(const string& time);
    void drawSteeringWheel(const shared_ptr<func::msg_parser::ComputeData> data, const float& line_width);
    void drawBarPlot(const std::unordered_map<int, int>& frequency01,const std::unordered_map<int, int>& frequency02);
    void drawBrakeData(const shared_ptr<func::msg_parser::ComputeData> data);
    void drawHmiData();
    void drawWatchdogState();
    void initSteerWheel(mpl::axes::Axes& axes);

  protected:
    //画框
    mpl::pyplot::PyPlot data_plt_;
    //轴系
    shared_ptr<mpl::axes::Axes> data_axes01_ptr_;
    shared_ptr<mpl::axes::Axes> data_axes02_ptr_;
    shared_ptr<mpl::axes::Axes> data_axes03_ptr_;
    shared_ptr<mpl::axes::Axes> bar_axes_ptr_;
    shared_ptr<mpl::axes::Axes> steering_wheel_axes_ptr_;
    shared_ptr<mpl::axes::Axes> hmi_patches_axes_ptr_;
    shared_ptr<mpl::axes::Axes> watchdog_axes_ptr_;
    // figure
    shared_ptr<mpl::figure::Figure> data_figure_ptr_;
    // background
    py::object data_background_;
    py::object vehicle_monitor_background_;
    py::object jet_cmap_;
    // artist
    py::object hmi_rect_patches_; 

    // data
    vector<float> steer_wheel_plt_data_;
    vector<float> brake_plt_data_;
    func::msg_parser::HmiData hmi_plt_data_;
};
}  // namespace animation
}  // namespace modules
