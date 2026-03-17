#include "function/animation_functions.h"

#include <filesystem>
#include <iostream>
#include "algorithm/weighted_window_mode.h"
#include <matplotlibcpp17/patches.h>
#include <Eigen/Dense>

namespace modules {
namespace animation {

bool AnimationFunctions::frequencyCtrl(int T, int64_t& last_time_stamp) {
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();  // 获取当前时间戳
  // 时间控制
  int record_time = current_time_stamp - last_time_stamp;
  if (record_time < T && last_time_stamp != 0) {
    return true;
  }
  last_time_stamp = current_time_stamp;
  return false;
}

void AnimationFunctions::drawSteeringData(const string& time){
  static bool once_flag = true;
  /******数据计算******/
  /*step01->实时数据更新*/
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
  // 横轴数据更新
  static vector<int> time_array;
  static int tick = 0;
  time_array.push_back(tick++);
  // 纵轴数据更新
  if (time_array.size() > DATA_BUFFER) {
    time_array.erase(time_array.begin());
    for(auto& line:line_data){
      line.erase(line.begin());
    }
  }
  /*step02->static artist生成*/
  static vector<string> lables = {"SWA", "SWT", "wheel_speed", "yaw_rate", "SWA_dot", "bias_T","l_mean","s_mean"};
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
      }else if(i==3 || i==6 || i==7){
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
    }else if(i==3 || i==6 || i==7){
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
}

void AnimationFunctions::drawSteeringWheel(const float& angle,const bool pilot, const float& line_width){
  static bool once_flag = true;
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
      frame_artist = steering_wheel_axes_ptr_->plot(Args(frame_x, frame_y), Kwargs("c"_a = "k", "lw"_a = line_width,"alpha"_a = 0.7)).unwrap().cast<py::list>()[0];
  }
  /*step03->draw artist*/
  string color = pilot ? "k" : "orange"; //TODO:可以优化
  frame_artist.attr("set_color")(color);
  frame_artist.attr("set_data")(frame_x, frame_y);
  steering_wheel_axes_ptr_->unwrap().attr("draw_artist")(frame_artist);
}

void AnimationFunctions::drawBarPlot(const std::unordered_map<int, int>& frequency01,const std::unordered_map<int, int>& frequency02) {
  using namespace ALG::WeightedWindows;
  static bool once_flag = true;
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
    if (index < 0 || index >= num) {
      // std::cerr << "ERROR: index out of bounds!" << std::endl;
      continue;
    }
    bar_artists[index].attr("set_height")(pair.second);
    bar_axes_ptr_->unwrap().attr("draw_artist")(bar_artists[index]);
    if(max_num<pair.second){
      max_num = pair.second;
    }
  }

  int max_num02 = 0;
  for (const auto pair : frequency02){
    float index = pair.first+std::floor(BAR_X/(2*RESOLUTION));
    if (index < 0 || index >= num) {
      // std::cerr << "ERROR: index out of bounds!" << std::endl;
      continue;
    }
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
}

void AnimationFunctions::drawBrakeData(const string& time){
  /******数据计算******/
  static bool once_flag = true;
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
  
  if (time_array.size() > DATA_BUFFER) {
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
        Kwargs("transform"_a = trans_figure,"va"_a = "bottom", "ha"_a = "center", "fontsize"_a = "large", "fontweight"_a = "bold")
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
}

void AnimationFunctions::drawHmiData(){
  py::list color_lst;
  auto button_switch = hmi_plt_data_.button_switch;
  for (const auto& pair : button_switch.buttons) {
    string color = pair.second ? "orange" : "gray";
    color_lst.append(color);
  }
  for (const auto& pair : button_switch.switches) {
    string color = pair.second ? "orange" : "gray";
    color_lst.append(color);
  }
  hmi_rect_patches_.attr("set_facecolors")(color_lst);
  hmi_patches_axes_ptr_->unwrap().attr("draw_artist")(hmi_rect_patches_);
}

void AnimationFunctions::drawWatchdogState(){
  static bool once_flag = true;
  static vector<py::object> text_artist;
  auto watchdog = hmi_plt_data_.watchdog_state;
  vector<string> state_name = {"watchdog:"};
  for (const auto& key : watchdog.state_order) {
    int value = watchdog.state[key];
    string state = watchdog.FSMStateToString(value);
    string each_state_name = key + ": " + state;
    state_name.push_back(each_state_name);
  }
  if (once_flag) {
    once_flag = false;
    try {
      for(int i=0;i<state_name.size();i++){
        // 计算矩形中心点坐标（使用正确的左下角坐标）
        float text_x = i>0 ? 0.02 : 0.5;
        double center_y = 0.95 - i *0.06;
        int fontsize = i>0 ? 12 : 16;
        string color = i>0 ? "white" : "red";
        string ha = i>0 ? "left" : "center";
        // 在矩形中心添加文字
        auto artist = watchdog_axes_ptr_->text(Args(text_x, center_y, state_name[i]),
                                                Kwargs("ha"_a = ha,
                                                        "va"_a = "center",
                                                        "color"_a = color,
                                                        "fontsize"_a = fontsize));
        text_artist.push_back(artist.unwrap());
      }    
    } catch (const exception& e) {
      cout << "[Brake] 首次执行异常: " << e.what() << endl;
      throw; // 重新抛出，让bug暴露
    }
  }
  //绘制watchdog状态
  for (int i = 0; i < state_name.size(); i++) {
    text_artist[i].attr("set_text")(state_name[i]);
    watchdog_axes_ptr_->unwrap().attr("draw_artist")(text_artist[i]);
  }
}

void AnimationFunctions::initSteerWheel(mpl::axes::Axes& axes){
  //------------------------------------steering wheel------------------------------------
  steering_wheel_axes_ptr_ = make_shared<mpl::axes::Axes>(axes);    
  steering_wheel_axes_ptr_->unwrap().attr("set_axis_off")();
  // steering_wheel_axes_ptr_->set_title(Args("steering wheel"));
  //绘制圆环
  pybind11::dict kwargs("width"_a = 0.8, "facecolor"_a="k", "edgecolor"_a="r", "linewidth"_a=2, "alpha"_a=0.7);
  auto circle = mpl::patches::Wedge(Args(py::make_tuple(0, 0), 5, 0, 360),kwargs);
  steering_wheel_axes_ptr_->add_patch(Args(circle.unwrap()));
  steering_wheel_axes_ptr_->set_aspect(Args("equal"));
}

}
}