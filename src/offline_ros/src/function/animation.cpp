#include "function/animation.h"

#include <filesystem>
#include <iostream>
#include "algorithm/weighted_window_mode.h"
#include <matplotlibcpp17/patches.h>
#include <Eigen/Dense>

namespace modules {
namespace animation {

const float CMD_X_RANGE = 100;
const int DURATION = 50;
const int Y_RANGE = 10;
const float BAR_X = 3.0;
const int DATA_BUFFER = 600;
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
  /***速度监视器***/
  pybind11::dict kwargs("figsize"_a = py::make_tuple(14, 7), "dpi"_a = 100, "tight_layout"_a = true);
  SWTorquePltInit(kwargs, CMD_X_RANGE);
  BarPltInit();
  SteeringWheelPltInit();
}

void Animation::InitBrakeSysPlt() {
  /***速度监视器***/
  pybind11::dict kwargs("figsize"_a = py::make_tuple(14, 7), "dpi"_a = 100, "tight_layout"_a = true);
  BrakePltInit(kwargs, CMD_X_RANGE);
}

bool Animation::FrequencyCtrl(int T, int64_t& last_time_stamp) {
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();  // 获取当前时间戳
  // 时间控制
  int record_time = current_time_stamp - last_time_stamp;
  if (record_time < T && last_time_stamp != 0) {
    return true;
  }
  last_time_stamp = current_time_stamp;
  return false;
}

void Animation::BarPltInit() {
  auto plt = mpl::pyplot::import();  
  auto subplots = plt.subplots();       
  //NOTE: noetic模式使用c++14版本编译,17可以使用更简单写法
  auto figure = std::get<0>(subplots);
  auto axes = std::get<1>(subplots);                       
  bar_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);    
  bar_axes_ptr_ = make_shared<mpl::axes::Axes>(axes);    
  bar_axes_ptr_->set_xlim(Args(-BAR_X/2, BAR_X/2));
  bar_axes_ptr_->set_ylim(Args(-100, 200.0));  
  plt.show(Args(), Kwargs("block"_a = 0));
  plt.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  bar_axes_ptr_->set_title(Args("long window"));
  plt.pause(Args(0.1));
  bar_background_ = canvas_copy_from_bbox(bar_figure_ptr_->unwrap());
}

void Animation::SteeringWheelPltInit() {
  auto plt = mpl::pyplot::import();  
  auto subplots = plt.subplots(Kwargs("figsize"_a = py::make_tuple(3, 3),"tight_layout"_a = true));                              
  //NOTE: noetic模式使用c++14版本编译,17可以使用更简单写法
  auto figure = std::get<0>(subplots);
  auto axes = std::get<1>(subplots);
  steering_wheel_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);    
  steering_wheel_axes_ptr_ = make_shared<mpl::axes::Axes>(axes); 
  steering_wheel_axes_ptr_->unwrap().attr("set_axis_off")();   
  plt.show(Args(), Kwargs("block"_a = 0));
  steering_wheel_axes_ptr_->set_title(Args("steering wheel"));
  //绘制圆环
  pybind11::dict kwargs("width"_a = 0.8, "facecolor"_a="k", "edgecolor"_a="r", "linewidth"_a=2, "alpha"_a=0.7);
  auto circle = mpl::patches::Wedge(Args(py::make_tuple(0, 0), 5, 0, 360),kwargs);
  steering_wheel_axes_ptr_->add_patch(Args(circle.unwrap()));
  plt.axis(Args("scaled"));
  steering_wheel_axes_ptr_->set_aspect(Args("equal"));
  plt.pause(Args(0.1));
  steering_wheel_background_ = canvas_copy_from_bbox(steering_wheel_figure_ptr_->unwrap());
}

void Animation::SWTorquePltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range) {
  data_plt_ = mpl::pyplot::import();                                
  mpl::figure::Figure figure = data_plt_.figure(Args(), fig_kwargs); 
  data_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);

  auto gs = data_figure_ptr_->add_gridspec(4, 1,
                                            Kwargs("left"_a = 0.03, "right"_a = 0.99,
                                                  "bottom"_a = 0.01, "top"_a = 0.97,
                                                  "wspace"_a = 0.1, "hspace"_a = 0.15));

  //axes01
  auto axes_obj_01 = figure.add_subplot(Args(gs(py::slice(0, 2, 1),0).unwrap()),Kwargs("facecolor"_a = "gray"));           
  data_axes01_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_01);    
  data_axes01_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes01_ptr_->set_ylim(Args(-5.0, 5.0));
  data_axes01_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes02
  auto axes_obj_02 = figure.add_subplot(Args(gs(2,0).unwrap()),Kwargs("facecolor"_a = "darkgrey"));           
  data_axes02_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_02);    
  data_axes02_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes02_ptr_->set_ylim(Args(-1, 30));   
  data_axes02_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes03
  auto axes_obj_03 = figure.add_subplot(Args(gs(3,0).unwrap()),Kwargs("facecolor"_a = "darkgrey"));           
  data_axes03_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_03);    
  data_axes03_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes03_ptr_->set_ylim(Args(-0.5, 0.5));  
  data_axes03_ptr_->set_xticklabels(Args(py::list())); 
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  // data_axes01_ptr_->unwrap().attr("set_axis_off")();

  data_plt_.pause(Args(0.1));
  data_background_ = canvas_copy_from_bbox(data_figure_ptr_->unwrap());
}

void Animation::BrakePltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range) {
  data_plt_ = mpl::pyplot::import();                                
  mpl::figure::Figure figure = data_plt_.figure(Args(), fig_kwargs); 
  data_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);

  auto gs = data_figure_ptr_->add_gridspec(4, 1,
                                           Kwargs("left"_a = 0.03, "right"_a = 0.99,
                                                  "bottom"_a = 0.01, "top"_a = 0.97,
                                                  "wspace"_a = 0.1, "hspace"_a = 0.15));

  //axes01
  auto axes_obj_01 = figure.add_subplot(Args(gs(py::slice(0, 2, 1),0).unwrap()),Kwargs("facecolor"_a = "gray"));           
  data_axes01_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_01);  
  data_axes01_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes01_ptr_->set_ylim(Args(-5, 0.5)); 
  data_axes01_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes02  
  auto axes_obj_02 = figure.add_subplot(Args(gs(2,0).unwrap()),Kwargs("facecolor"_a = "darkgrey"));            
  data_axes02_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_02);    
  data_axes02_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes02_ptr_->set_ylim(Args(-6, 1));   
  data_axes02_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  //axes03
  auto axes_obj_03 = figure.add_subplot(Args(gs(3,0).unwrap()),Kwargs("facecolor"_a = "silver"));           
  data_axes03_ptr_ = make_shared<mpl::axes::Axes>(axes_obj_03);    
  data_axes03_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  data_axes03_ptr_->set_ylim(Args(-0.1, 30));  
  data_axes03_ptr_->set_xticklabels(Args(py::list()));
  data_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  data_plt_.show(Args(), Kwargs("block"_a = 0));
  data_plt_.pause(Args(0.1));
  data_background_ = canvas_copy_from_bbox(data_figure_ptr_->unwrap());
}

void Animation::SetSteerWheelData(const vector<float>& new_data) {
  steer_wheel_plt_data_ = new_data;
}

void Animation::SetBrakeData(const vector<float>& new_data) {
  brake_plt_data_ = new_data;
}

void Animation::SWTorqueMonitor(int buffer_length,const string& time) {
  static bool once_flag = true;
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(DURATION, last_sim_time_stamp)) return;
  canvas_restore_region(data_figure_ptr_->unwrap(), data_background_);
  /******数据计算******/
  /*step01->实时数据更新*/
  static vector<float> time_array;
  static float test_tick = 0;
  test_tick += 0.4;
  time_array.push_back(test_tick);
  int data_num = steer_wheel_plt_data_.size();
  static mesh2D line_data(data_num);
  static vector<py::object> lines_artist(data_num);
  static vector<py::object> legend_artist(3);
  //标注数据
  static py::object text_artist;
  string local_time = "local time: " + time;
  for(uint i=0;i<data_num;i++){
    line_data[i].push_back(steer_wheel_plt_data_[i]);
  }
  // 数据更新
  if (time_array.size() > buffer_length) {
    time_array.erase(time_array.begin());
    for(auto& line:line_data){
      line.erase(line.begin());
    }
  }
  /*step02->static artist生成*/
  static vector<string> lables = {"SWA", "SWT", "wheel_speed", "yaw_rate", "SWA_dot", "bias_T"};
  if (once_flag) {
    once_flag = false;
    py::object trans_figure = data_axes01_ptr_->unwrap().attr("transAxes");
    text_artist = data_axes01_ptr_->text(Args(0.5, 1.0, local_time),Kwargs("transform"_a = trans_figure,"va"_a = "bottom", "ha"_a = "center", "fontsize"_a = "large", "fontweight"_a = "bold")).unwrap();
    for (int i = 0; i < line_data.size(); i++) {
      if(i==0 || i==1 || i==4 || i==5){
        lines_artist[i] = data_axes01_ptr_->plot(Args(time_array, line_data[i]), Kwargs("c"_a = COLORS[i], "lw"_a = 1.0,"label"_a = lables[i])).unwrap().cast<py::list>()[0];
        legend_artist[0] = data_axes01_ptr_->legend(Args(),Kwargs("loc"_a = "lower right")).unwrap();
      }
      else if(i==2){
        lines_artist[i] = data_axes02_ptr_->plot(Args(time_array, line_data[i]), Kwargs("c"_a = COLORS[i], "lw"_a = 1.0,"label"_a = lables[i])).unwrap().cast<py::list>()[0];
        legend_artist[1] = data_axes02_ptr_->legend(Args(),Kwargs("loc"_a = "lower right")).unwrap();
      }else if(i==3){
        lines_artist[i] = data_axes03_ptr_->plot(Args(time_array, line_data[i]), Kwargs("c"_a = COLORS[i], "lw"_a = 1.0,"label"_a = lables[i])).unwrap().cast<py::list>()[0];
        legend_artist[2] = data_axes03_ptr_->legend(Args(),Kwargs("loc"_a = "lower right")).unwrap();
      }
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (int i = 0; i < lines_artist.size(); i++) {
    lines_artist[i].attr("set_data")(time_array, line_data[i]);
    if(i==0 || i==1 || i==4 || i==5){
      data_axes01_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
    }
    else if(i==2){
      data_axes02_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
    }else if(i==3){
      data_axes03_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
    }
  }
  data_axes01_ptr_->unwrap().attr("draw_artist")(legend_artist[0]);
  data_axes02_ptr_->unwrap().attr("draw_artist")(legend_artist[1]);
  data_axes03_ptr_->unwrap().attr("draw_artist")(legend_artist[2]);
  // text数据
  text_artist.attr("set_text")(local_time);
  data_axes01_ptr_->unwrap().attr("draw_artist")(text_artist);
  /******axis计算******/
  auto axes_xlim = data_axes01_ptr_->get_xlim();
  if (time_array.back() > get<1>(axes_xlim) - 10) {
    float x_min = get<1>(axes_xlim) - 20.f;
    float x_max = x_min + CMD_X_RANGE;
    data_axes01_ptr_->set_xlim(Args(x_min, x_max));
    data_axes02_ptr_->set_xlim(Args(x_min, x_max));
    data_axes03_ptr_->set_xlim(Args(x_min, x_max));
  }
  canvas_update_flush_events(data_figure_ptr_->unwrap());
}

// void Animation::BarPlot(const std::unordered_map<int, int>& frequency01,
//                        const std::unordered_map<int, int>& frequency02) {
//   std::cout << "BarPlot start" << std::endl;
  
//   try {
//     using namespace ALG::WeightedWindows;
//     static bool once_flag = true;
    
//     const int num = BAR_X/RESOLUTION;
//     std::cout << "num=" << num << std::endl;
    
//     static vector<py::object> bar_artists(num);
//     static vector<py::object> bar02_artists(num);
    
//     if(once_flag){
//       std::cout << "First init" << std::endl;
//       once_flag = false;
      
//       vector<float> bin(num), nums(num, 0);
//       for(int i=0;i<num;i++) bin[i] = -BAR_X/2 + RESOLUTION*i;
      
//       auto artists = bar_axes_ptr_->bar(Args(bin, nums, RESOLUTION*0.8),
//                                        Kwargs("color"_a = "blue"))
//                      .unwrap().cast<py::list>();
//       auto artists02 = bar_axes_ptr_->bar(Args(bin, nums, RESOLUTION*0.8),
//                                          Kwargs("color"_a = "green"))
//                        .unwrap().cast<py::list>();
      
//       std::cout << "Artists sizes: " << artists.size() << ", " << artists02.size() << std::endl;
      
//       for(int i=0;i<num;i++){
//         bar_artists[i] = artists[i];
//         bar02_artists[i] = artists02[i];
//       }
//     }
    
//     // 更新数据
//     for (const auto& pair : frequency01){
//       int index = pair.first + std::floor(BAR_X/(2*RESOLUTION));
//       if (index >= 0 && index < num) {
//         bar_artists[index].attr("set_height")(pair.second);
//         bar_axes_ptr_->unwrap().attr("draw_artist")(bar_artists[index]);
//       }
//     }
    
//     for (const auto& pair : frequency02){
//       int index = pair.first + std::floor(BAR_X/(2*RESOLUTION));
//       if (index >= 0 && index < num) {
//         bar02_artists[index].attr("set_height")(-pair.second);
//         bar_axes_ptr_->unwrap().attr("draw_artist")(bar02_artists[index]);
//       }
//     }
    
//     canvas_update_flush_events(bar_figure_ptr_->unwrap());
//     std::cout << "BarPlot done" << std::endl;
    
//   } catch (const py::error_already_set& e) {
//     std::cerr << "Python Error: " << e.what() << std::endl;
//     throw;
//   }
// }

void Animation::BarPlot(const std::unordered_map<int, int>& frequency01,const std::unordered_map<int, int>& frequency02) {
  using namespace ALG::WeightedWindows;
  static bool once_flag = true;
  canvas_restore_region(bar_figure_ptr_->unwrap(), bar_background_);
  /******数据计算******/
  /*step02->static artist生成*/
  const int num = BAR_X/RESOLUTION;
  static vector<py::object> bar_artists(num);
  static vector<py::object> bar02_artists(num);
  // std::unordered_map<int, int> frequency;
  static vector<float> bin(num);
  static vector<float> nums(num);
  if(once_flag){
    once_flag = false;
    for(int i=0;i<num;i++){
      bin[i] = -BAR_X/2 + RESOLUTION*i;
      nums[i] = 0;
    }
    auto artists = bar_axes_ptr_->bar(Args(bin, nums, RESOLUTION*0.8),Kwargs("color"_a = "blue")).unwrap().cast<py::list>();
    auto artists02 = bar_axes_ptr_->bar(Args(bin, nums, RESOLUTION*0.8),Kwargs("color"_a = "green")).unwrap().cast<py::list>();
    for(int i=0;i<num;i++){
      bar_artists[i] = artists[i];
      bar02_artists[i] = artists02[i];
    }
  }
  /*step03->artist实时数据更新并绘制*/
  int max_num = 0;
  for (const auto pair : frequency01){
    float index = pair.first+std::floor(BAR_X/(2*RESOLUTION));
    bar_artists[index].attr("set_height")(pair.second);
    bar_axes_ptr_->unwrap().attr("draw_artist")(bar_artists[index]);
    if(max_num<pair.second){
      max_num = pair.second;
    }
  }

  int max_num02 = 0;
  for (const auto pair : frequency02){
    float index = pair.first+std::floor(BAR_X/(2*RESOLUTION));
    bar02_artists[index].attr("set_height")(-pair.second);
    bar_axes_ptr_->unwrap().attr("draw_artist")(bar02_artists[index]);
    if(max_num02<pair.second){
      max_num02 = pair.second;
    }
  }

  pybind11::list axes_ylim = bar_axes_ptr_->unwrap().attr("get_ylim")();
  int y_min = axes_ylim[0].cast<float>();
  int y_max = axes_ylim[1].cast<float>();
  bool change = false;
  if (max_num > (y_max - 5)) {
    y_max = max_num + 5;
    change = true;
  }
  if(-max_num02 < (y_min + 5)) {
    change = true;
    y_min = -max_num02 - 5;
  }
  if(change){
    bar_axes_ptr_->unwrap().attr("set_ylim")(Args(y_min, y_max));
  }

  canvas_update_flush_events(bar_figure_ptr_->unwrap());
}

void Animation::BrakeMonitor(int buffer_length,const string& time) {
  static bool once_flag = true;
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(DURATION, last_sim_time_stamp)) return;
  canvas_restore_region(data_figure_ptr_->unwrap(), data_background_);
  /******数据计算******/
  /*step01->实时数据更新*/
  static vector<float> time_array;
  static float test_tick = 0;
  test_tick += 0.4;
  time_array.push_back(test_tick);
  int data_num = brake_plt_data_.size();
  static mesh2D line_data(data_num);
  static vector<py::object> lines_artist(data_num);
  static vector<py::object> legend_artist(3);
  //标注数据
  static py::object text_artist;

  for(uint i=0; i<data_num; i++){
    line_data[i].push_back(brake_plt_data_[i]);
  }
  
  if (time_array.size() > buffer_length) {
    time_array.erase(time_array.begin());
    for(auto& line:line_data){
      line.erase(line.begin());
    }
  }
  /*step02->static artist生成*/
  static vector<string> lables = {"ebs_cmd","acc_mes", "acc_ref","speed","pitch" ,"brake_pressure", "wheel_speed", "brake_gain"};
  if (once_flag) {
    once_flag = false;
    try {
      py::object trans_figure = data_axes01_ptr_->unwrap().attr("transAxes");
      text_artist = data_axes01_ptr_->text(
        Args(0.5, 1.0, "local time: " + time),
        Kwargs("transform"_a = trans_figure, "va"_a = "bottom", "ha"_a = "center")
      ).unwrap();
      for (int i = 0; i < line_data.size(); i++) {
        if(i==0 || i==5 || i==7){
          lines_artist[i] = data_axes01_ptr_->plot(
            Args(time_array, line_data[i]), 
            Kwargs("c"_a = COLORS[i % COLORS.size()], "lw"_a = 1.0, "label"_a = lables[i])
          ).unwrap().cast<py::list>()[0];
        }else if(i==1 || i==2 || i==4){
          lines_artist[i] = data_axes02_ptr_->plot(
            Args(time_array, line_data[i]), 
            Kwargs("c"_a = COLORS[i % COLORS.size()], "lw"_a = 1.0, "label"_a = lables[i])
          ).unwrap().cast<py::list>()[0];
        }else if(i==3 || i==6){
          lines_artist[i] = data_axes03_ptr_->plot(
            Args(time_array, line_data[i]), 
            Kwargs("c"_a = COLORS[i % COLORS.size()], "lw"_a = 1.0, "label"_a = lables[i])
          ).unwrap().cast<py::list>()[0];
        }
      }
      legend_artist[0] = data_axes01_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
      legend_artist[1] = data_axes02_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
      legend_artist[2] = data_axes03_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    } catch (const exception& e) {
      cout << "[Brake] 首次执行异常: " << e.what() << endl;
      throw; // 重新抛出，让bug暴露
    }
  }
  
  for (int i = 0; i < lines_artist.size(); i++) {
    try {
      lines_artist[i].attr("set_data")(time_array, line_data[i]);
      if(i==0 || i==5 || i==7){
        data_axes01_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
      }else if(i==1 || i==2 || i==4){
        data_axes02_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
      }else if(i==3 || i==6){
        data_axes03_ptr_->unwrap().attr("draw_artist")(lines_artist[i]);
      }
    } catch (const exception& e) {
      cout << "[Brake] 更新线条 " << i << " 异常: " << e.what() << endl;
    }
  }
  
  try {
    text_artist.attr("set_text")("local time: " + time);
    data_axes01_ptr_->unwrap().attr("draw_artist")(text_artist);
    data_axes01_ptr_->unwrap().attr("draw_artist")(legend_artist[0]);
    data_axes02_ptr_->unwrap().attr("draw_artist")(legend_artist[1]);
    data_axes03_ptr_->unwrap().attr("draw_artist")(legend_artist[2]);
  } catch (const exception& e) {
    cout << "[Brake] 更新文本/图例异常: " << e.what() << endl;
  }
  
  auto axes_xlim = data_axes01_ptr_->get_xlim();
  if (time_array.back() > get<1>(axes_xlim) - 5) {
    float x_min = get<1>(axes_xlim) - 10.f;
    float x_max = x_min + CMD_X_RANGE;
    data_axes01_ptr_->set_xlim(Args(x_min, x_max));
    data_axes02_ptr_->set_xlim(Args(x_min, x_max));
    data_axes03_ptr_->set_xlim(Args(x_min, x_max));
  }
  canvas_update_flush_events(data_figure_ptr_->unwrap());
}

void Animation::SteeringWheelMonitor(const float& angle,const bool pilot){
  static bool once_flag = true;
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(DURATION, last_sim_time_stamp)) return;
  canvas_restore_region(steering_wheel_figure_ptr_->unwrap(), steering_wheel_background_);
  /******数据计算******/
  const int point_num = 7;
  const vector<float> fix_angle{-M_PI/2,-M_PI,-M_PI/2, -M_PI/2, -M_PI/2, 0.0f,-M_PI/2};
  const Eigen::Vector2f origin_point(4.5f, 0.0f);
  const Eigen::Vector2f central_point(0.8f, -0.0f);
  vector<float> frame_x(point_num);
  vector<float> frame_y(point_num);
  Eigen::Matrix2f rotateM;
  for(int i=0;i<point_num;i++){
    float rotate_angle = fix_angle[i] + angle;
    rotateM << cos(rotate_angle), -sin(rotate_angle), 
               sin(rotate_angle), cos(rotate_angle);
    auto new_point = ((i & 1) == 0) ? rotateM * central_point : rotateM * origin_point;
    frame_x[i] = new_point(0);
    frame_y[i] = new_point(1);
  }
  /*step02->static artist生成*/
  static py::object frame_artist;
  if (once_flag) {
      once_flag = false;
      frame_artist = steering_wheel_axes_ptr_->plot(Args(frame_x, frame_y), Kwargs("c"_a = "k", "lw"_a = 20.0,"alpha"_a = 0.7)).unwrap().cast<py::list>()[0];
  }
  /*step03->draw artist*/
  string color = pilot ? "k" : "orange"; //TODO:可以优化
  frame_artist.attr("set_color")(color);
  frame_artist.attr("set_data")(frame_x, frame_y);
  steering_wheel_axes_ptr_->unwrap().attr("draw_artist")(frame_artist);
  canvas_update_flush_events(steering_wheel_figure_ptr_->unwrap());
}
}
}