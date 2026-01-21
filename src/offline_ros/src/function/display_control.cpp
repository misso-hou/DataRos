#include "function/display_control.h"
#include <filesystem> 
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <algorithm>
#include <vector>
#include <iomanip>

using namespace std;

/**
 * @brief:按行读取csv数据(分组数据需要设置组中对应的行序列)
 * @param:
 *      file_name:csv文件路径
 *      row_num:csv每组数据的行数(例如三行为一组)
 *      bias_index:数据在csv数据组中对应的行索引
 * @return:csv数据转成的二维数组数据
 */
vector<vector<float>> DisplayControl::RowDataReader(string file_name,vector<long long>& time, int row_num, int bias_index) {
  ifstream file(file_name);  // csv文件导入
  if (!file.is_open()) {
    cout << file_name << "----->"
         << "文件不存在" << endl;
  }
  if (file.eof()) {
    cout << file_name << "----->"
         << "文件为空" << endl;
  }
  vector<vector<float>> all_data;
  string line;
  int line_index = 1;

  vector<long long> exact_timestamps;  // 存储精确时间戳
  while (getline(file, line)) {  // 按行提取csv文件
    // 将障碍物数据中index的倍数行提取
    if ((line_index - bias_index) % row_num == 0) {  // note:障碍物三行数据为一组
      stringstream s1(line);
      string charItem;
      float fItem = 0.f;
      vector<float> one_line_data;
      int col_count = 0;  //NOTE:时间戳独立解析
      long long exact_ts = 0;
      /*提取行点集数据*/
      while (getline(s1, charItem, ',')) {
        if (col_count == 0) {  // 第一列：时间戳
          exact_ts = stoll(charItem);  // 改为stoll获取精确值
          exact_timestamps.push_back(exact_ts);  // 存储精确值
        } else {  // 其他列
          one_line_data.push_back(stof(charItem));
        }
        col_count++;
      }
      all_data.push_back(one_line_data);
    } else {
      line_index++;
      continue;
    }
    line_index++;
  }
  // ifs.close();
  time = exact_timestamps;
  return all_data;
}

/**
 * @brief:获取一个字符输入(不需要按enter)
 * @return:键盘输入的字符
 */
 char DisplayControl::GetKey() {
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);           // 获取当前终端设置
  newt = oldt;                              // 复制当前设置
  newt.c_lflag &= ~(ICANON | ECHO);         // 禁用回显和规范模式
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 设置新的终端设置
  ch = getchar();                           // 获取一个字符
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复原来的终端设置
  return ch;
}

/**
 * @brief:获取一个字符输入 (不需要按enter),并处理方向键的多字节序列
 * @return:键盘输入的字符
 */
string DisplayControl::GetKeyWithTimeout(int timeout_ms) {
  struct termios oldt, newt;
  fd_set readfds;
  struct timeval tv;
  char ch;
  string input = "";

  // 获取当前终端设置
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);         // 禁用回显和规范模式
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 设置新的终端设置

  FD_ZERO(&readfds);               // 清除文件描述符集
  FD_SET(STDIN_FILENO, &readfds);  // 将标准输入文件描述符添加到集

  // 设置超时时间(10毫秒)
  tv.tv_sec = 0;                   // 秒
  tv.tv_usec = timeout_ms * 1000;  // 微秒(10毫秒 = 10000微秒)

  // 使用 select() 检查是否有输入
  int result = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

  if (result > 0) {  // 如果有输入
    // 获取一个字符
    ch = getchar();
    input += ch;
    // 如果是ESC (27), 可能是方向键的开始字符
    if (ch == 27) {
      ch = getchar();  // 获取下一个字符
      input += ch;
      if (ch == '[') {
        ch = getchar();  // 获取方向键的最后一个字符
        input += ch;
      }
    }
  }
  // 恢复终端设置
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return input;
}

/**
 * @brief：数据播放键盘交互控制
 * @param:
 * @return: 键盘输入的字符
 */
bool DisplayControl::KeyboardCtrl(int &index) {
  static int copy_time = 0;
  // 暂停&快进&后退控制
  back_ = false;
  if (!pause_) {
    string input = GetKeyWithTimeout(1);  //设置1秒超时
    // 暂停程序
    if (input == " ") {
      cout << "\033[33m 循环暂停，按任意键继续前进一帧，按<-后退一帧\33[0m" << endl;
      pause_ = true;
    }
    // 退出程序(按 'q' 退出循环)
    else if (input == "q") {
      cout << "退出程序!!!" << endl;
      return false;
    }
    // 上键速度X2
    else if (input == "\033[A") {
      cycle_time_ /= 2;
      cycle_time_ = max(1, cycle_time_);
      cout << "loop step cycle_time_ :" << cycle_time_ << "(us)" << endl;
    }
    // 下键速度/2
    else if (input == "\033[B") {
      cycle_time_ *= 2;
      cycle_time_ = min(cycle_time_, 100);
      cout << "loop step cycle_time_ :" << cycle_time_ << "(us)" << endl;
    }
    // 右键快进
    else if (input == "\033[C") {
      index += 300;
      index = min(data_length_ - 1, index);
      erase_ = true;
    }
    // 左键倒退
    else if (input == "\033[D") {
      index -= 100;
      index = max(index, 0);
      erase_ = true;
    }
    copy_time = cycle_time_;
  }
  // 空格暂停
  else {
    cycle_time_ = 5;  // 防止键盘输入等待
    //等待用户输入（持续等待）
    char key = GetKey();
    // 方向按键识别
    if (key == 27) {             // 方向键的起始字符是 ESC (27)
      char nextChar = GetKey();  // 获取方向键的第二个字符
      if (nextChar == '[') {
        nextChar = GetKey();  // 获取最后一个字符
        if (nextChar == 'D') {
          index--;
          index = max(index, 0);
          erase_ = true;
          back_ = true;
        }
      }
    }
    // 退出阻塞模式
    if (key == 'c') {
      cout << "继续循环..." << endl;
      pause_ = false;
      erase_ = false;
      cycle_time_ = copy_time;
    }
  }
  //数据结束后控制逻辑
  if (index == data_length_ - 1) {
    cout << "\033[31m !!!已显示全部数据!!! \33[0m" << endl;
    cout << "\033[32m 按 'c' 继续程序，'q' 退出程序。\33[0m" << endl;
    //等待用户输入
    char key = GetKey();
    if (key == 'c') {
      index = 0;
      erase_ = true;
    } else if (key == 'q') {
      cout << "!!!退出程序!!!" << endl;
      return false;  //按'q'退出循环
    } else {
      cout << "***再次确认下一步动作***" << endl;
      index--;
    }
  }

  return true;
}

void DisplayControl::SetParam(int argc, char *argv[]) {
  //文件路径配置
  char currentPath[FILENAME_MAX];
  getcwd(currentPath, sizeof(currentPath));
  string current_path = string(currentPath);
  // 设置播放速度(默认周期20ms)
  cycle_time_ = argc >= 2 ? stoi(argv[1]) : 10;
  // 播放位置设置（tick累计)
  start_index_ = argc >= 3 ? atoi(argv[2]) : 0;
}

/**
 *@brief:csv数据提取
 */
void DisplayControl::ExtractData() {
  // 从当前工作目录出发
  std::string data_file = "src/offline_ros/data/swa01.csv";  // 相对当前目录
  std::cout << "数据文件: " << data_file << std::endl;
  //数据提取
  data_mat_ = RowDataReader(data_file, ts_, 1, 1);
  data_length_ = data_mat_.size();
}


float DisplayControl::LowPassFilter01(const float& data,const float& alpha) {
  static bool first_flag = true;
  static float filtered_data = 0.0;

  if (first_flag) {  // first time enter
    first_flag = false;
    filtered_data = data;
  } else {
    filtered_data = alpha * data + (1.0f - alpha) * filtered_data;
  }
  return filtered_data;
}

float DisplayControl::LowPassFilter02(const float& data,const float& alpha) {
  static bool first_flag = true;
  static float filtered_data = 0.0;

  if (first_flag) {  // first time enter
    first_flag = false;
    filtered_data = data;
  } else {
    filtered_data = alpha * data + (1.0f - alpha) * filtered_data;
  }
  return filtered_data;
}

string DisplayControl::getLogTimestamp(const int index){
  // 关键修正：先定义time_t变量（左值），再取地址
  time_t raw_sec = ts_[index];
  struct tm t;
  gmtime_r(&raw_sec, &t); // 现在可以正常取地址，无编译报错
  // 时区修正：UTC+8（北京时间），取模24避免小时超过23
  int beijing_hour = (t.tm_hour + 8) % 24;
  // 仅打印北京时间的时分秒
  string local_time = std::to_string(beijing_hour/10) + std::to_string(beijing_hour%10) + ":" +
                      std::to_string(t.tm_min/10) + std::to_string(t.tm_min%10) + ":" +
                      std::to_string(t.tm_sec/10) + std::to_string(t.tm_sec%10);
  return local_time;
}





