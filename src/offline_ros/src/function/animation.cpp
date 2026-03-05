#include "function/animation.h"

#include <filesystem>
#include <iostream>
#include "algorithm/weighted_window_mode.h"
#include <matplotlibcpp17/patches.h>
#include <Eigen/Dense>

namespace modules {
namespace animation {

const int DURATION = 50;
const float button_width = 0.7;
const float button_height = 0.18;

// canvas and flush events
auto canvas_update_flush_events = [](pybind11::object figure) {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_update_attr = canvas_attr.attr("update");
  pybind11::object canvas_flush_events_attr = canvas_attr.attr("flush_events");
  pybind11::object ret01 = canvas_update_attr();
  pybind11::object ret02 = canvas_flush_events_attr();
};

// canvas_copy_from_bbox
auto canvas_copy_from_bbox = [](pybind11::object figure) -> pybind11::object {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_copy_from_bbox_attr = canvas_attr.attr("copy_from_bbox");
  pybind11::object fig_bbox = figure.attr("bbox");
  pybind11::object ret = canvas_copy_from_bbox_attr(fig_bbox);
  return ret;
};

// canvas_restore_region
auto canvas_restore_region = [](pybind11::object figure, pybind11::object bg) {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_restore_region_attr = canvas_attr.attr("restore_region");
  canvas_restore_region_attr(bg);
};

void Animation::InitWeightedWindowsPlt() {
  pybind11::dict fig_kwargs("figsize"_a = py::make_tuple(14, 7), "dpi"_a = 100, "tight_layout"_a = true);
  auto x_axis_range = CMD_X_RANGE;
  data_plt_ = mpl::pyplot::import();                                
  mpl::figure::Figure figure = data_plt_.figure(Args(), fig_kwargs); 
  data_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);

  auto gs = data_figure_ptr_->add_gridspec(4, 4,
                                            Kwargs("left"_a = 0.03, "right"_a = 0.99,
                                                  "bottom"_a = 0.04, "top"_a = 0.97,
                                                  "wspace"_a = 0.15, "hspace"_a = 0.15));
  //----------------------------data curve----------------------------                                            
  //axes01
  auto axes_obj_01 = figure.add_subplot(Args(gs(py::slice(0, 2, 1), py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "gray"));           
  data_axes01_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_01);    
  data_axes01_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes01_ptr_->set_ylim(Args(-4.0, 4.0));
  data_axes01_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes02
  auto axes_obj_02 = figure.add_subplot(Args(gs(2, py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "darkgrey"));           
  data_axes02_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_02);    
  data_axes02_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes02_ptr_->set_ylim(Args(-1, 30));   
  data_axes02_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes03
  auto axes_obj_03 = figure.add_subplot(Args(gs(3, py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "darkgrey"));           
  data_axes03_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_03);    
  data_axes03_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes03_ptr_->set_ylim(Args(-2.5, 2.5));  
  data_axes03_ptr_->set_xticklabels(Args(py::list())); 
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //------------------------------------steering wheel------------------------------------
  auto steering_wheel_axes_obj = figure.add_subplot(Args(gs(py::slice(0, 2, 1), 3).unwrap()));
  initSteerWheel(steering_wheel_axes_obj);
  data_plt_.axis(Args("scaled"));
  //------------------------------------统计数据---------------------------------------
  auto bar_axes_obj = figure.add_subplot(Args(gs(py::slice(2, 4, 1), 3).unwrap())); 
  bar_axes_ptr_ = make_shared<mpl::axes::Axes>(bar_axes_obj);    
  bar_axes_ptr_->set_xlim(Args(-BAR_X/2, BAR_X/2));
  bar_axes_ptr_->set_ylim(Args(-50.0, 100.0));  
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  bar_axes_ptr_->set_title(Args("statistic"));
  //canvas背景保存
  data_plt_.pause(Args(0.1));
  data_background_ = canvas_copy_from_bbox(data_figure_ptr_->unwrap());
}

void Animation::InitBrakeSysPlt() {
  pybind11::dict fig_kwargs("figsize"_a = py::make_tuple(14, 7), "dpi"_a = 100);
  auto x_axis_range = CMD_X_RANGE;
  data_plt_ = mpl::pyplot::import();                                
  mpl::figure::Figure figure = data_plt_.figure(Args(), fig_kwargs); 
  data_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);

  auto gs = data_figure_ptr_->add_gridspec(4, 4,
                                            Kwargs("left"_a = 0.03, "right"_a = 0.99,
                                                  "bottom"_a = 0.04, "top"_a = 0.97,
                                                  "wspace"_a = 0.15, "hspace"_a = 0.15));
  //----------------------------data curve----------------------------    
  //axes01
  auto axes_obj_01 = figure.add_subplot(Args(gs(py::slice(0, 2, 1), py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "gray"));           
  data_axes01_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_01);  
  data_axes01_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes01_ptr_->set_ylim(Args(-5, 0.5)); 
  data_axes01_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes02  
  auto axes_obj_02 = figure.add_subplot(Args(gs(2, py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "darkgrey"));            
  data_axes02_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_02);    
  data_axes02_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes02_ptr_->set_ylim(Args(-6, 1));   
  data_axes02_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes03
  auto axes_obj_03 = figure.add_subplot(Args(gs(3, py::slice(0, 3, 1)).unwrap()),Kwargs("facecolor"_a = "silver"));           
  data_axes03_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_03);    
  data_axes03_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes03_ptr_->set_ylim(Args(-0.1, 30));  
  data_axes03_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //------------------------------------steering wheel------------------------------------
  auto steering_wheel_axes_obj = figure.add_subplot(Args(gs(py::slice(0, 2, 1), 3).unwrap()));
  initSteerWheel(steering_wheel_axes_obj);
  data_plt_.axis(Args("scaled"));
  data_plt_.pause(Args(0.1));
  data_background_ = canvas_copy_from_bbox(data_figure_ptr_->unwrap());
}

void Animation::InitVehicleMonitor() {
  pybind11::dict fig_kwargs("figsize"_a = py::make_tuple(14, 7), "dpi"_a = 100, "tight_layout"_a = true);
  data_plt_ = mpl::pyplot::import();                                
  mpl::figure::Figure figure = data_plt_.figure(Args(), fig_kwargs); 
  data_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);

  auto gs = data_figure_ptr_->add_gridspec(4, 8,
                                            Kwargs("left"_a = 0.03, "right"_a = 0.99,
                                                  "bottom"_a = 0.04, "top"_a = 0.97,
                                                  "wspace"_a = 0.15, "hspace"_a = 0.15));

  //----------------------------data curve----------------------------    
  //axes01->hmi 按键&开关&状态 //NOTE:占用figure的比例与坐标轴的范围对应
  auto patches_axes_obj = figure.add_subplot(Args(gs(0, py::slice(0, 7, 1)).unwrap()),Kwargs("facecolor"_a = "gray")); 
  hmi_patches_axes_ptr_ = make_shared<mpl::axes::Axes>(patches_axes_obj);    
  hmi_patches_axes_ptr_->unwrap().attr("set_axis_off")();
  hmi_patches_axes_ptr_->set_title(Args("HMI"));
  py::list lst;
  func::msg_parser::ButtonAndSwitch button_switch;
  vector<string> button_name = {"buttons:"};
  for (const auto& pair : button_switch.buttons) {
    const auto& key = pair.first;
    button_name.push_back(key);
  }
  //绘制buttons
  for(int i=0;i<button_name.size();i++){
    // 矩形左下角坐标
    float rect_x = 0 + i*0.05 + i*button_width;
    float rect_y = 1 - button_height;
    // 计算矩形中心点坐标（使用正确的左下角坐标）
    double center_x = rect_x + button_width/2.0;
    double center_y = rect_y + button_height/2.0;
    int fontsize = i>0 ? 10 : 16;
    string color = i>0 ? "black" : "red";
    // 在矩形中心添加文字
    hmi_patches_axes_ptr_->text(Args(center_x, center_y, button_name[i]),
                                Kwargs("ha"_a = "center",
                                        "va"_a = "center",
                                        "color"_a = color,
                                        "fontsize"_a = fontsize));
    auto r = mpl::patches::Rectangle(Args(py::make_tuple(rect_x, rect_y), button_width, button_height),
                                    Kwargs("ec"_a = "gray", "fc"_a = "gray", "fill"_a = true));
    if(i>0){
      lst.append(r.unwrap());
    }
  }
  //绘制switches
  vector<string> switches_name = {"switches:"};
  for (const auto& pair : button_switch.switches) {
    const auto& key = pair.first;
    switches_name.push_back(key);
  }
  for(int i=0;i<switches_name.size();i++){
    // 矩形左下角坐标
    float rect_x = 0 + i*0.05 + i*button_width;
    float rect_y = 1 - 2.5*button_height;
    // 计算矩形中心点坐标（使用正确的左下角坐标）
    double center_x = rect_x + button_width/2.0;
    double center_y = rect_y + button_height/2.0;
    int fontsize = i>0 ? 12 : 16;
    string color = i>0 ? "black" : "red";
    // 在矩形中心添加文字
    hmi_patches_axes_ptr_->text(Args(center_x, center_y, switches_name[i]),
                                Kwargs("ha"_a = "center",
                                        "va"_a = "center",
                                        "color"_a = color,
                                        "fontsize"_a = fontsize));
    auto r = mpl::patches::Rectangle(Args(py::make_tuple(rect_x, rect_y), button_width, button_height),
                                    Kwargs("ec"_a = "gray", "fc"_a = "gray", "fill"_a = true));
    if(i>0){
      lst.append(r.unwrap());
    }
  }
  auto p = mpl::collections::PatchCollection(Args(lst), Kwargs("alpha"_a = 0.5));
  hmi_rect_patches_ = p.unwrap();
  hmi_patches_axes_ptr_->add_collection(Args(hmi_rect_patches_));
  hmi_patches_axes_ptr_->set_xlim(Args(0, 7));
  hmi_patches_axes_ptr_->set_ylim(Args(0, 1));
  //------------------------------------steering wheel------------------------------------
  auto steering_wheel_axes_obj = figure.add_subplot(Args(gs(0, 7).unwrap()));
  initSteerWheel(steering_wheel_axes_obj);
  data_plt_.axis(Args("scaled"));
  data_plt_.pause(Args(0.1));
  vehicle_monitor_background_ = canvas_copy_from_bbox(data_figure_ptr_->unwrap());                                          
}



void Animation::SetSteerWheelData(const vector<float>& new_data) {
  steer_wheel_plt_data_ = new_data;
}

void Animation::SetBrakeData(const vector<float>& new_data) {
  brake_plt_data_ = new_data;
}

void Animation::SetHmiData(const func::msg_parser::HmiData hmi_data) {
  hmi_plt_data_ = hmi_data;
}

void Animation::SWTorqueMonitor(const string& time, 
                                const float& angle, 
                                const bool pilot,
                                const std::unordered_map<int, int>& freq01,
                                const std::unordered_map<int, int>& freq02) {
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (frequencyCtrl(DURATION, last_sim_time_stamp)) return;
  /******绘图******/
  canvas_restore_region(data_figure_ptr_->unwrap(), data_background_);
  drawSteeringData(time);
  drawSteeringWheel(angle,pilot);
  drawBarPlot(freq01,freq02);
  canvas_update_flush_events(data_figure_ptr_->unwrap());
}


void Animation::BrakeMonitor(const string& time,
                             const float& angle, 
                             const bool pilot) {
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (frequencyCtrl(DURATION, last_sim_time_stamp)) return;
  /******绘图******/
  canvas_restore_region(data_figure_ptr_->unwrap(), data_background_);
  drawBrakeData(time);
  drawSteeringWheel(angle,pilot);
  canvas_update_flush_events(data_figure_ptr_->unwrap());
}

void Animation::VehicleMonitor() {
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (frequencyCtrl(DURATION, last_sim_time_stamp)) return;
  /******绘图******/
  canvas_restore_region(data_figure_ptr_->unwrap(), vehicle_monitor_background_);
  drawHmiData();
  drawSteeringWheel(0,false); //TODO:
  canvas_update_flush_events(data_figure_ptr_->unwrap());
}

}
}