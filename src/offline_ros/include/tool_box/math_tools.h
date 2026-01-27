#pragma once
#include <iostream>

namespace toolbox {
namespace math {

inline float LowPassFilter(const float& new_data, const float& old_data, const float& alpha) {
  auto filtered_data = alpha * new_data + (1.0f - alpha) * old_data;
  return filtered_data;
}

}
}