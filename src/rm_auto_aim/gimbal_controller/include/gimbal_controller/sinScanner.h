//
// Created by spaaaaace on 2026/2/20.
//

#ifndef BUILD_SINSCANNER_H
#define BUILD_SINSCANNER_H

#include <cmath>
#include <chrono>
class SineScanner
{
public:
    SineScanner(double amplitude, double frequency, double phase = 0.0)
        : A_(amplitude), f_(frequency), phi_(phase)
    {
        start_time_ = std::chrono::steady_clock::now();
    }

    double getValue();

private:
    double A_;
    double f_;
    double phi_;
    std::chrono::steady_clock::time_point start_time_;
};

#endif //BUILD_SINSCANNER_H