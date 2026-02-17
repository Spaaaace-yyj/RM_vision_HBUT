#include "../include/gimbal_controller/coordsolver.h"
#include <cmath>
#include <stdexcept>

// 需要根据实际情况调整的常量
constexpr double STANDARD_GRAVITY = 9.80665;     // 重力加速度（单位：m/s²）
constexpr double MIN_HORIZONAL_DISTANCE = 1e-3;  // 视为垂直的最小水平距离（单位：m）
constexpr double SAFE_SQUARED_NORM = 1e-6;       // 安全模长平方阈值

CoordSolver::CoordSolver(int max_iter, float stop_error, int rk_iterations)
    : max_iterations(max_iter),
      stop_error(stop_error),
      runge_kutta_iterations(rk_iterations)
{
	bullet_speed = 25.0;// 根据实际弹速调整
    air_resistance_coeff = 0.02;// 根据实际空气阻力系数调整
    // 参数校验
    if(max_iterations <= 0 || runge_kutta_iterations <= 0){
        throw std::invalid_argument("Iteration counts must be positive");
    }
}

double CoordSolver::calcPitch(const Eigen::Vector3d& xyz) const
{
    const double horizontal_norm = Eigen::Vector2d(xyz.x(), xyz.y()).norm();
    return std::atan2(xyz.z(), horizontal_norm) * RAD2DEG;
}

double CoordSolver::calcYaw(const Eigen::Vector3d& xyz) const
{
    return std::atan2(xyz.x(), xyz.y()) * RAD2DEG;
}

Eigen::Vector2d CoordSolver::calcYawPitch(const Eigen::Vector3d& xyz) const
{
    return {calcYaw(xyz), calcPitch(xyz)};
}

double CoordSolver::dynamicCalcPitchOffset(const Eigen::Vector3d& xyz) const
{
    // 输入校验
    const double squared_norm = xyz.squaredNorm();
    if(squared_norm < SAFE_SQUARED_NORM){
        throw std::invalid_argument("Input vector is too small");
    }

    const double dist_vertical = xyz.z();
    const double dist_horizonal = std::sqrt(squared_norm - dist_vertical*dist_vertical);
    
    if(dist_horizonal < MIN_HORIZONAL_DISTANCE){
        return 0.0;
    }

    const double initial_pitch = std::atan(dist_vertical / dist_horizonal) * RAD2DEG;
    double current_pitch = initial_pitch;
    double accumulated_height = dist_vertical;
    
    const double delta_x = dist_horizonal / runge_kutta_iterations;
    bool converged = false;

    for(int i = 0; i < max_iterations; ++i){
        const double y = simulateTrajectory(current_pitch * DEG2RAD, delta_x);
        const double error = dist_vertical - y;
        
        if(std::abs(error) <= stop_error){
            converged = true;
            break;
        }
        
        accumulated_height += error;
        current_pitch = std::atan(accumulated_height / dist_horizonal) * RAD2DEG;
    }

    if(!converged){
        // 根据实际需求处理未收敛情况
        // 可记录日志或抛出异常
    }
    
    return current_pitch - initial_pitch;
}

// 私有成员函数实现
double CoordSolver::simulateTrajectory(double pitch_rad, double delta_x) const
{
    double x = 0.0, y = 0.0;
    double u = bullet_speed * std::cos(pitch_rad);
    double v = bullet_speed * std::sin(pitch_rad);
    const double k = air_resistance_coeff;

    for(int step = 0; step < runge_kutta_iterations; ++step){
        const auto [du, dv] = rungeKuttaStep(u, v, delta_x, k);
        u += du;
        v += dv;
        y += v * delta_x;
        x += delta_x;
    }
    return y;
}

std::pair<double, double> CoordSolver::rungeKuttaStep(
    double u, double v, double dx, double k) const
{
    const auto calc_du = [k](double u, double v) {
        return -k * u * std::sqrt(u*u + v*v);
    };

    const auto calc_dv = [k](double u, double v) {
        return -STANDARD_GRAVITY - k * v * std::sqrt(u*u + v*v);
    };

    // K1
    const double k1u = calc_du(u, v);
    const double k1v = calc_dv(u, v);
    
    // K2
    const double u2 = u + k1u * dx/2;
    const double v2 = v + k1v * dx/2;
    const double k2u = calc_du(u2, v2);
    const double k2v = calc_dv(u2, v2);
    
    // K3
    const double u3 = u + k2u * dx/2;
    const double v3 = v + k2v * dx/2;
    const double k3u = calc_du(u3, v3);
    const double k3v = calc_dv(u3, v3);
    
    // K4
    const double u4 = u + k3u * dx;
    const double v4 = v + k3v * dx;
    const double k4u = calc_du(u4, v4);
    const double k4v = calc_dv(u4, v4);

    const double du = dx/6 * (k1u + 2*k2u + 2*k3u + k4u);
    const double dv = dx/6 * (k1v + 2*k2v + 2*k3v + k4v);
    
    return {du, dv};
}
// 当前假设：
// - Y轴指向目标正前方
// - Z轴垂直向上
// 若坐标系定义不同，需修改xyz分量的使用方式