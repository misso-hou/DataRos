#include "function/animation_functions.h"

#include <filesystem>
#include <iostream>
#include "algorithm/weighted_window_mode.h"
#include <matplotlibcpp17/patches.h>
#include <Eigen/Dense>

namespace modules {
namespace animation {

using namespace func::msg_parser;
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

void AnimationFunctions::rangeSet(bool is_all,const int max_x) {
  auto axes_xlim = data_axes01_ptr_->get_xlim();
  if(is_all){
    auto range = get<1>(axes_xlim) - get<0>(axes_xlim);
    if (max_x > (range*0.6)) {
      float x_min = get<0>(axes_xlim); 
      float x_max = get<1>(axes_xlim) + 2;
      data_axes01_ptr_->set_xlim(Args(x_min, x_max));
      data_axes02_ptr_->set_xlim(Args(x_min, x_max));
      data_axes03_ptr_->set_xlim(Args(x_min, x_max));
    }
  }else{
    int left_space = 10;
    if (max_x > get<1>(axes_xlim) - left_space) {
      float x_min = get<1>(axes_xlim) - 2*left_space;
      float x_max = x_min + CMD_X_RANGE;
      data_axes01_ptr_->set_xlim(Args(x_min, x_max));
      data_axes02_ptr_->set_xlim(Args(x_min, x_max));
      data_axes03_ptr_->set_xlim(Args(x_min, x_max));
    }
  }
}

void AnimationFunctions::drawHmiData(){
  static bool once_flag = true;
  /******数据计算******/
  /*step01->实时数据更新*/
  int data_num = hmi_plt_data_.status_report.status.size();
  static map<string,vector<float>> line_data;
  static map<string,py::object> lines_artists;
  static vector<py::object> legend_artist(3);
  //标注数据
  static py::object text_artist;
  string local_time = "local time: " + hmi_plt_data_.local_time;
  line_data["VEHICLE_WIPER_STATUS"].push_back(hmi_plt_data_.status_report.status.at("VEHICLE_WIPER_STATUS"));
  line_data["BSD_RIGHT_LED_NA"].push_back(hmi_plt_data_.status_report.status.at("BSD_RIGHT_LED_NA"));
  line_data["BSD_LEFT_LED_NA"].push_back(hmi_plt_data_.status_report.status.at("BSD_LEFT_LED_NA"));
  // 横轴数据更新
  static vector<int> time_array;
  static int tick = 0;
  time_array.push_back(tick++);
  // 纵轴数据更新
  if (time_array.size() > DATA_BUFFER) {
    time_array.erase(time_array.begin());
    for(auto& pair:line_data){
      pair.second.erase(pair.second.begin());
    }
  }
  /*step02->static artist生成*/
  if (once_flag) {
    once_flag = false;
    py::object trans_figure = data_axes01_ptr_->unwrap().attr("transAxes");
    text_artist = data_axes01_ptr_->text(Args(0.5, 1.0, "local time: " + hmi_plt_data_.local_time),Kwargs("transform"_a = trans_figure,"va"_a = "bottom", "ha"_a = "center", "fontsize"_a = "large", "fontweight"_a = "bold")).unwrap();
    lines_artists["VEHICLE_WIPER_STATUS"] = data_axes01_ptr_->plot(Args(time_array, line_data.at("VEHICLE_WIPER_STATUS")), Kwargs("c"_a = COLORS[0], "lw"_a = 1.0,"label"_a = "VEHICLE_WIPER_STATUS")).unwrap().cast<py::list>()[0];
    legend_artist[0] = data_axes01_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    lines_artists["BSD_RIGHT_LED_NA"] = data_axes02_ptr_->plot(Args(time_array, line_data.at("BSD_RIGHT_LED_NA")), Kwargs("c"_a = COLORS[1], "lw"_a = 1.0,"label"_a = "BSD_RIGHT_LED_NA")).unwrap().cast<py::list>()[0];
    legend_artist[1] = data_axes02_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    lines_artists["BSD_LEFT_LED_NA"] = data_axes03_ptr_->plot(Args(time_array, line_data.at("BSD_LEFT_LED_NA")), Kwargs("c"_a = COLORS[2], "lw"_a = 1.0,"label"_a = "BSD_LEFT_LED_NA")).unwrap().cast<py::list>()[0];
    legend_artist[2] = data_axes03_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
  }
  /*step03->artist实时数据更新并绘制*/
  for (auto& line_artist : lines_artists) {
    auto key = line_artist.first;
    line_artist.second.attr("set_data")(time_array, line_data.at(key));
    data_axes01_ptr_->unwrap().attr("draw_artist")(line_artist.second);
  }

  text_artist.attr("set_text")("local time: " + hmi_plt_data_.local_time);
  data_axes01_ptr_->unwrap().attr("draw_artist")(text_artist);
  data_axes01_ptr_->unwrap().attr("draw_artist")(legend_artist[0]);
  data_axes02_ptr_->unwrap().attr("draw_artist")(legend_artist[1]);
  data_axes03_ptr_->unwrap().attr("draw_artist")(legend_artist[2]);
  /******axis计算******/
  rangeSet(true,time_array.back());
}

void AnimationFunctions::drawSteeringData(const shared_ptr<ComputeData> vehicle_data){
  static bool once_flag = true;
  /******数据计算******/
  /*step01->实时数据更新*/
  int data_num = vehicle_data->order.size();
  static map<string,vector<float>> line_data;
  static map<string,py::object> lines_artists;
  static vector<py::object> legend_artist(3);
  //标注数据
  static py::object text_artist;
  string local_time = "local time: " + vehicle_data->local_time;
  for(const auto& key : vehicle_data->order){
    if(vehicle_data->data.at(key) == std::numeric_limits<float>::lowest()) continue;
    line_data[key].push_back(vehicle_data->data.at(key));
  }
  // 横轴数据更新
  static vector<int> time_array;
  static int tick = 0;
  time_array.push_back(tick++);
  // 纵轴数据更新
  if (time_array.size() > DATA_BUFFER) {
    time_array.erase(time_array.begin());
    for(auto& pair:line_data){
      pair.second.erase(pair.second.begin());
    }
  }
  /*step02->static artist生成*/
  if (once_flag) {
    once_flag = false;
    py::object trans_figure = data_axes01_ptr_->unwrap().attr("transAxes");
    text_artist = data_axes01_ptr_->text(Args(0.5, 1.0, "local time: " + vehicle_data->local_time),Kwargs("transform"_a = trans_figure,"va"_a = "bottom", "ha"_a = "center", "fontsize"_a = "large", "fontweight"_a = "bold")).unwrap();
    int color_count = 0;
    for (const std::string& key : vehicle_data->order) {
      if(key=="steer_wheel_angle" || 
         key=="steer_wheel_torque_filtered" || 
         key=="steer_wheel_angle_dot" || 
         key=="steer_wheel_torque_mode"){
        lines_artists[key] = data_axes01_ptr_->plot(Args(time_array, line_data.at(key)), Kwargs("c"_a = COLORS[color_count], "lw"_a = 1.0,"label"_a = key)).unwrap().cast<py::list>()[0];
        
      }
      else if(key=="wheel_speed"){
        lines_artists[key] = data_axes02_ptr_->plot(Args(time_array, line_data.at(key)), Kwargs("c"_a = COLORS[color_count], "lw"_a = 1.0,"label"_a = key)).unwrap().cast<py::list>()[0];
      }else if(key=="yaw_rate" || 
               key=="long_window_mean" || 
               key=="short_window_mean"){
        lines_artists[key] = data_axes03_ptr_->plot(Args(time_array, line_data.at(key)), Kwargs("c"_a = COLORS[color_count], "lw"_a = 1.0,"label"_a = key)).unwrap().cast<py::list>()[0];
      }else{
        if(color_count > 0) color_count--;
      }
      color_count++;
    }
    legend_artist[0] = data_axes01_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    legend_artist[1] = data_axes02_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    legend_artist[2] = data_axes03_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
  }
  /*step03->artist实时数据更新并绘制*/
  for (auto& line_artist : lines_artists) {
    auto key = line_artist.first;
    line_artist.second.attr("set_data")(time_array, line_data.at(key));
    if(key=="steer_wheel_angle" || 
       key=="steer_wheel_torque_filtered" || 
       key=="steer_wheel_angle_dot" || 
       key=="steer_wheel_torque_mode"){
      data_axes01_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }else if(key=="wheel_speed"){
      data_axes02_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }else if(key=="yaw_rate" || 
              key=="long_window_mean" || 
              key=="short_window_mean"){
      data_axes03_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }
  }

  text_artist.attr("set_text")("local time: " + vehicle_data->local_time);
  data_axes01_ptr_->unwrap().attr("draw_artist")(text_artist);
  data_axes01_ptr_->unwrap().attr("draw_artist")(legend_artist[0]);
  data_axes02_ptr_->unwrap().attr("draw_artist")(legend_artist[1]);
  data_axes03_ptr_->unwrap().attr("draw_artist")(legend_artist[2]);
  /******axis计算******/
  rangeSet(false,time_array.back());
}

void AnimationFunctions::drawSteeringWheel(const shared_ptr<ComputeData> vehicle_data, const float& line_width){
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
    float rotate_angle = fix_angle[i] + vehicle_data->data.at("steer_wheel_angle");
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
  int pilot = static_cast<int>(vehicle_data->data.at("adas_state"));
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

void AnimationFunctions::drawBrakeData(const shared_ptr<ComputeData> vehicle_data){
  /******数据计算******/
  static bool once_flag = true;
  /*step01->实时数据更新*/
  static vector<float> time_array;
  static float test_tick = 0;
  test_tick += 0.4;
  time_array.push_back(test_tick);
  int data_num = vehicle_data->order.size();
  static map<string,vector<float>> line_data;
  static map<string,py::object> lines_artists;
  static vector<py::object> legend_artist(3);
  //标注数据
  static py::object text_artist;
  for(const auto& key : vehicle_data->order){
    if(vehicle_data->data.at(key) == std::numeric_limits<float>::lowest()) continue;
    line_data[key].push_back(vehicle_data->data.at(key));
  }
  
  if (time_array.size() > DATA_BUFFER) {
    time_array.erase(time_array.begin());
    for(auto& pair:line_data){
      pair.second.erase(pair.second.begin());
    }
  }
  /*step02->static artist生成*/
  if (once_flag) {
    once_flag = false;
    py::object trans_figure = data_axes01_ptr_->unwrap().attr("transAxes");
    text_artist = data_axes01_ptr_->text(Args(0.5, 1.0, "local time: " + vehicle_data->local_time),
                                         Kwargs("transform"_a = trans_figure,"va"_a = "bottom", "ha"_a = "center", "fontsize"_a = "large", "fontweight"_a = "bold")
                                        ).unwrap();
    int color_count = 0;
    for (const std::string& key : vehicle_data->order) {
      if(key == "ebs_cmd" || key == "brake_pressure_filtered" || key == "brake_gain"){
        lines_artists[key] = data_axes01_ptr_->plot(Args(time_array, line_data.at(key)), 
                                                    Kwargs("c"_a = COLORS[color_count % COLORS.size()], "lw"_a = 1.0, "label"_a = key)
                                                  ).unwrap().cast<py::list>()[0];
      }else if(key == "acc_mes" || key == "acc_ref" || key == "pitch"){
        lines_artists[key] = data_axes02_ptr_->plot(Args(time_array, line_data.at(key)), 
                                                    Kwargs("c"_a = COLORS[color_count % COLORS.size()], "lw"_a = 1.0, "label"_a = key)
                                                    ).unwrap().cast<py::list>()[0];
      }else if(key == "speed" || key == "wheel_speed"){
        lines_artists[key] = data_axes03_ptr_->plot(Args(time_array, line_data.at(key)), 
                                                    Kwargs("c"_a = COLORS[color_count % COLORS.size()], "lw"_a = 1.0, "label"_a = key)
                                                    ).unwrap().cast<py::list>()[0];
      }else{
        if(color_count > 0) color_count--;
      }
      color_count++;
    }
    legend_artist[0] = data_axes01_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    legend_artist[1] = data_axes02_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
    legend_artist[2] = data_axes03_ptr_->legend(Args(),Kwargs("loc"_a = "upper right")).unwrap();
  }

  for(auto& line_artist : lines_artists){
    auto key = line_artist.first;
    line_artist.second.attr("set_data")(time_array, line_data.at(key));
    if(key == "ebs_cmd" || key == "brake_pressure_filtered" || key == "brake_gain"){
      data_axes01_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }else if(key == "acc_mes" || key == "acc_ref" || key == "pitch"){
      data_axes02_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }else if(key == "speed" || key == "wheel_speed"){
      data_axes03_ptr_->unwrap().attr("draw_artist")(line_artist.second);
    }
  }
  
  text_artist.attr("set_text")("local time: " + vehicle_data->local_time);
  data_axes01_ptr_->unwrap().attr("draw_artist")(text_artist);
  data_axes01_ptr_->unwrap().attr("draw_artist")(legend_artist[0]);
  data_axes02_ptr_->unwrap().attr("draw_artist")(legend_artist[1]);
  data_axes03_ptr_->unwrap().attr("draw_artist")(legend_artist[2]);
  //axis set
  rangeSet(false,time_array.back());
}

void AnimationFunctions::drawHmiButtonAndSwitch(){
  py::list color_lst;
  auto button_switch = hmi_plt_data_.button_switch;
  for (const auto& pair : button_switch.buttons) {
    string color = pair.second ? "orange" : "gray";
    color_lst.append(color);
  }
  for (const auto& key : button_switch.switch_order) {
    string color = button_switch.switches.at(key) ? "orange" : "gray";
    color_lst.append(color);
  }
  hmi_rect_patches_.attr("set_facecolors")(color_lst);
  hmi_patches_axes_ptr_->unwrap().attr("draw_artist")(hmi_rect_patches_);
}

void AnimationFunctions::drawWatchdogState(){
  static bool once_flag = true;
  static vector<py::object> text_artist;
  auto watchdog = hmi_plt_data_.watchdog_state;
  vector<string> state_name;
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
        double center_y = 0.85 - i *0.12;
        // 在矩形中心添加文字
        auto artist = watchdog_axes_ptr_->text(Args(0.03, center_y, state_name[i]),
                                                Kwargs("ha"_a = "left",
                                                        "va"_a = "center",
                                                        "color"_a = "white",
                                                        "fontsize"_a = 12));
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

void AnimationFunctions::drawStatusReport(){
  static bool once_flag = true;
  static vector<py::object> text_artist;
  auto status_report = hmi_plt_data_.status_report;
  vector<string> status_name;
  for (const auto& pair : status_report.status) {
    auto key = pair.first;
    int value = pair.second;
    string each_name = key + ": " + to_string(value);
    status_name.push_back(each_name);
  }
  if (once_flag) {
    once_flag = false;
    try {
      for(int i=0;i<status_name.size();i++){
        // 计算矩形中心点坐标（使用正确的左下角坐标）
        double center_y = 0.95 - i *0.07;
        // 在矩形中心添加文字
        auto artist = status_report_axes_ptr_->text(Args(0.03, center_y, status_name[i]),
                                                    Kwargs("ha"_a = "left",
                                                            "va"_a = "center",
                                                            "color"_a = "white",
                                                            "fontsize"_a = 12));
        text_artist.push_back(artist.unwrap());
      }    
    } catch (const exception& e) {
      cout << "[Brake] 首次执行异常: " << e.what() << endl;
      throw; // 重新抛出，让bug暴露
    }
  }
  //绘制watchdog状态
  for (int i = 0; i < status_name.size(); i++) {
    text_artist[i].attr("set_text")(status_name[i]);
    status_report_axes_ptr_->unwrap().attr("draw_artist")(text_artist[i]);
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

void AnimationFunctions::drawPedalBar(const shared_ptr<ComputeData> vehicle_data){
  auto artists = bar_artists_.cast<py::list>();
  vector<float> rate(2);
  rate[0] = vehicle_data->data.at("gas_pedal");
  rate[1] = vehicle_data->data.at("brake_pedal");
  vector<string> labels(2);
  char buf[64];
  // 保留 1 位小数
  snprintf(buf, sizeof(buf), "Trottle %.1f%%", rate[0] * 100);
  labels[0] = buf;
  snprintf(buf, sizeof(buf), "Brake %.1f%%", rate[1] * 100);
  labels[1] = buf;
  //process bar
  for(int i=0;i<artists.size();i++){
    artists[i].attr("set_width")(rate[i]);
    bar_axes_ptr_->unwrap().attr("draw_artist")(artists[i]);
  }
  //test 
  static bool once_flag = true;
  static vector<py::object> text_artist(2);
  if (once_flag) {
    once_flag = false;
    for(int i=0;i<labels.size();i++){
      // 计算矩形中心点坐标（使用正确的左下角坐标）
      double center_y = 0.85 - i*0.4;
      // 在矩形中心添加文字
      auto artist = bar_axes_ptr_->text(Args(0.0, center_y, labels[0]),
                                              Kwargs("ha"_a = "left",
                                                      "va"_a = "center",
                                                      "color"_a = "black",
                                                      "fontsize"_a = 12));
      text_artist[i] = artist.unwrap();
    }    
  }
  //绘制watchdog状态
  for (int i = 0; i < labels.size(); i++) {
    text_artist[i].attr("set_text")(labels[i]);
    bar_axes_ptr_->unwrap().attr("draw_artist")(text_artist[i]);
  }
}

}
}