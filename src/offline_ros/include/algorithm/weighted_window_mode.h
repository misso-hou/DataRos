#pragma once

#include <iostream>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <algorithm>

namespace ALG{
namespace WeightedWindows{

// 全局常量（可放在头文件中）
const float RESOLUTION = 0.1;
const float HISTORY_WEIGHT = 0.35;
const float B_HIGH = 0.2;
const float B_LOW = 0.1;
const float SPEED_THD = 20.0;  
const float ANGLE_THD = 0.1;
const float BIAS_THD = 1.5f;

struct MovingWindow
{
    int size;
    std::deque<float> data;
    std::unordered_map<int, int> frequency;
    float mode;
    float mean;
};

class WeightedWindows {
public:
    WeightedWindows(int long_size,int short_size);
    float getWeightedMode(const float& torque,
                          const float& speed,
                          const float& yaw,
                          const bool pilot);
    std::unordered_map<int, int> getLongFreqency(){return long_window_.frequency;}    
    std::unordered_map<int, int> getShortFreqency(){return short_window_.frequency;}
    float getLongMean(){return long_window_.mean;}    
    float getShortMean(){return short_window_.mean;}                        

private:
    bool segmentData(const float& speed,const float& angle);
    bool segmentData(const bool pilot);
    void updateWindow(MovingWindow& window,const float& new_data);
    void calLongWindowMode(MovingWindow& window);
    void calShortWindowMode(MovingWindow& window);
    float computeWeightedMode();

private:
    float last_result_;
    bool update_flag_ = false;

private: // tuneable parameters
    MovingWindow long_window_;
    MovingWindow short_window_;
    float resolution_;
    float history_weight_;
    float b_high_;
    float b_low_ = 1.0;
    float speed_thd_;
    float yaw_thd_;
};

inline WeightedWindows::WeightedWindows(int long_size,int short_size){
    long_window_.size = long_size;
    short_window_.size = short_size;
}

inline bool WeightedWindows::segmentData(const float& speed,const float& angle){
    bool update = false;
    if(speed > SPEED_THD && std::abs(angle) < ANGLE_THD){
        update = true;
    }
    return update;
}

inline bool WeightedWindows::segmentData(const bool pilot){
    bool update = false;
    if(pilot){
        update = true;
    }
    return update;
}

inline void WeightedWindows::updateWindow(MovingWindow& window,
                                          const float& new_data) {
    if (window.data.size() >= window.size && window.data.size()>1) {
        float first_data = window.data.front();
        int index = static_cast<int>(std::round(first_data/RESOLUTION)); //NOTE:index计算
        window.data.pop_front();
        window.frequency[index]--;
        if (window.frequency[index] == 0) {
            window.frequency.erase(index);
        }
    }
    int new_group = static_cast<int>(std::round(new_data/RESOLUTION));
    window.data.push_back(new_data);
    window.frequency[new_group]++;
}

inline void WeightedWindows::calLongWindowMode(MovingWindow& window) {
    if (window.data.empty()) return;
    // 众数计算(找到出现频率最高的元素)
    float mode = window.data.front();
    int index = static_cast<int>(std::round(mode/RESOLUTION)); //NOTE:index计算
    int max_count = window.frequency[index];
    for (const auto& pair : window.frequency) {
        if (pair.second > max_count) {
            mode = pair.first*RESOLUTION;
            max_count = pair.second;
        }
    }
    window.mode = mode;
    // 均值计算
    float sum = std::accumulate(window.data.begin(), window.data.end(), 0.0f);
    window.mean = sum / static_cast<float>(window.data.size());
}

inline void WeightedWindows::calShortWindowMode(MovingWindow& window) {
    if (window.data.empty()) return;
    // 1. 提取所有频率值（去重，避免重复处理相同频率）
    std::vector<int> frequencies;
    for (const auto& pair : window.frequency) {
        frequencies.push_back(pair.second);
    }
    // 找到出现频率最高的元素
    float mode = long_window_.mode;

    // 2. 去重并排序（从大到小）
    std::sort(frequencies.begin(), frequencies.end(), std::greater<int>());
    // 去重（保留第一个出现的最大/第二大值）
    auto last = std::unique(frequencies.begin(), frequencies.end());
    frequencies.erase(last, frequencies.end());

    // 3. 获取最高和第二高频率
    int max1 = frequencies[0]; // 最高频率
    int max2 = (frequencies.size() >= 2) ? frequencies[1] : -1; // 第二高频率（若存在）

    if((float)max1/(float)max2 > BIAS_THD){
        // std::cout << "test:!!!!!!:" << (float)max1/(float)max2 << std::endl;
        // 4. 找到对应频率的元素（可能有多个元素同频，这里取第一个）
        for (const auto& pair : window.frequency) {
            if (pair.second == max1) {
                mode = pair.first*RESOLUTION; // 最高频率元素
            } 
        }
    }
    window.mode = mode;
    // 均值计算
    float sum = std::accumulate(window.data.begin(), window.data.end(), 0.0f);
    window.mean = sum / static_cast<float>(window.data.size());
}


inline float WeightedWindows::computeWeightedMode() {
    // calculate weighting parameters
    float a2,a3;
    auto delta = std::abs(long_window_.mode - short_window_.mode) / B_HIGH;
    if(delta < (B_LOW/B_HIGH)) {
        a2 = 1 - HISTORY_WEIGHT;
        a3 = 0;
    } else if(delta > 1) {
        a2 = 0;
        a3 = 1 - HISTORY_WEIGHT;
    } else {
        a2 = (1-delta)*(1-HISTORY_WEIGHT);
        a3 = delta*(1-HISTORY_WEIGHT);
    }
    float result = last_result_;
    result = HISTORY_WEIGHT*last_result_ + a2*long_window_.mode + a3*short_window_.mode;
    last_result_ = result;
    return result;
}

inline float WeightedWindows::getWeightedMode(const float& torque,
                                              const float& speed,
                                              const float& angle,
                                              const bool pilot) {
    //step01->segment data
    // update_flag_ = segmentData(speed,angle);
    update_flag_ = segmentData(pilot);
    if(!update_flag_) {
        return last_result_;
    }

    if(update_flag_) {
        updateWindow(long_window_,torque);
        updateWindow(short_window_,torque);
        calLongWindowMode(long_window_);
        calShortWindowMode(short_window_);
    }
    //step04->compute weighted windows mode
    float offset_torque = computeWeightedMode();
    return offset_torque;
}

}
}