#pragma once

#include <cmath>

namespace buff_algo {

// 弧度转角度
inline constexpr double r2d(const double rad) {
    return rad * 180.0 / M_PI;
}

// 角度转弧度
inline constexpr double d2r(const double deg) {
    return deg * M_PI / 180.0;
}

// 把角度（弧度制）修正到 -pi ~ pi 之间
inline double rad_period_correction(const double rad) {
    return rad + std::round((-rad) / (2.0 * M_PI)) * (2.0 * M_PI);
}

} // namespace buff_algo
