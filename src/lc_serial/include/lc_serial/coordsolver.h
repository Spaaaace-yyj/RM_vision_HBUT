/**
 * @file coordsolver.h
 * @brief 位姿解算器，主要是用来解算弹道的
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-04-09
 * 
 */

#ifndef SERIAL__COORDSOLVER_H_
#define SERIAL__COORDSOLVER_H_

// c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

// eigen
#include <Eigen/Core>
#include <Eigen/Dense>

class CoordSolver
{
public:
    CoordSolver(int max_iter, float stop_error, int rk_iterations);
    // ~CoordSolver();
    
    double dynamicCalcPitchOffset(const Eigen::Vector3d& xyz) const;
    
    double calcYaw(const Eigen::Vector3d& xyz) const;
    double calcPitch(const Eigen::Vector3d& xyz) const;
    Eigen::Vector2d calcYawPitch(const Eigen::Vector3d& xyz) const;

    double simulateTrajectory(double pitch_rad, double delta_x) const;

    std::pair<double, double> rungeKuttaStep(double u, double v, double dx, double k) const;

    double bullet_speed = 15;            //TODO:弹速可变
    // double k = 0.0389;                //25°C,1atm,小弹丸
    double k = 0.000000001;                //25°C,1atm,小弹丸
    // double k = 0.0111;                //25°C,1atm,大弹丸
private:
    int max_iterations;
    float stop_error;
    int runge_kutta_iterations;
    // const int bullet_speed = 16;            //TODO:弹速可变
    const double g = 7.001;
    double air_resistance_coeff;
    const double RAD2DEG = 180.0 / M_PI;
    const double DEG2RAD = M_PI / 180.0;
};

#endif  // SERIAL__COORDSOLVER_H_