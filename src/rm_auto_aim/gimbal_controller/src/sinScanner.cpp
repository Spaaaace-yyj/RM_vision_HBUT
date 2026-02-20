//
// Created by spaaaaace on 2026/2/20.
//

#include "../include/gimbal_controller/sinScanner.h"

double SineScanner::getValue()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - start_time_;
    double t = dt.count();

    return A_ * std::sin(2.0 * M_PI * f_ * t + phi_);
}