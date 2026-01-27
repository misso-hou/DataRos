#pragma once
#include <iostream>

namespace toolbox {
namespace math {

inline float LowPassFilter(const float& data,const float& alpha) {
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

}
}