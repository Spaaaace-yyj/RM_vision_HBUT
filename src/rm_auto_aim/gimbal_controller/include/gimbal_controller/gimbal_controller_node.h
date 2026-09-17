//
// Created by spaaaaace on 2026/2/17.
//

#ifndef BUILD_GIMBAL_CONTROLLER_NODE_H
#define BUILD_GIMBAL_CONTROLLER_NODE_H

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

//Ros2
#include <rclcpp/rclcpp.hpp>
//Interface
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "auto_aim_interfaces/msg/gimbal_control.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/debug_controller.hpp"
#include "auto_aim_interfaces/msg/gimbal_feed.hpp"

#include "coordsolver.h"
#include "sinScanner.h"

#include "axis_tiny_mpc.h"

struct BulletPitchResult
{
    bool success = false;
    double pitch = 0.0; // rad，抬头为正
    double fly_time = 0.0; // s
};

static BulletPitchResult calcBallisticPitch(
    double bullet_speed, // 子弹速度，m/s
    double horizontal_dist, // 水平距离 sqrt(x*x + y*y)，m
    double height, // 目标高度 z，m
    double g = 9.7833) // 重力加速度
{
    BulletPitchResult result;

    if (!std::isfinite(bullet_speed) ||
        !std::isfinite(horizontal_dist) ||
        !std::isfinite(height) ||
        bullet_speed <= 1e-6 ||
        horizontal_dist <= 1e-6)
    {
        return result;
    }

    const double v0 = bullet_speed;
    const double d = horizontal_dist;
    const double h = height;

    // 弹道方程：
    // h = d * tan(theta) - g * d^2 / (2 * v0^2 * cos^2(theta))
    //
    // 令 T = tan(theta)
    // cos^2(theta) = 1 / (1 + T^2)
    //
    // 可化为：
    // a*T^2 - d*T + (a + h) = 0
    //
    // 其中：
    // a = g*d^2 / (2*v0^2)

    const double a = g * d * d / (2.0 * v0 * v0);
    const double b = -d;
    const double c = a + h;

    const double delta = b * b - 4.0 * a * c;

    if (delta < 0.0)
    {
        // 当前弹速、距离、高度下理论无解
        return result;
    }

    const double sqrt_delta = std::sqrt(delta);

    const double tan_pitch_1 = (-b + sqrt_delta) / (2.0 * a);
    const double tan_pitch_2 = (-b - sqrt_delta) / (2.0 * a);

    const double pitch_1 = std::atan(tan_pitch_1);
    const double pitch_2 = std::atan(tan_pitch_2);

    const double cos_1 = std::cos(pitch_1);
    const double cos_2 = std::cos(pitch_2);

    if (std::fabs(cos_1) <= 1e-6 || std::fabs(cos_2) <= 1e-6)
    {
        return result;
    }

    const double fly_time_1 = d / (v0 * cos_1);
    const double fly_time_2 = d / (v0 * cos_2);

    if (!std::isfinite(fly_time_1) || !std::isfinite(fly_time_2) ||
        fly_time_1 <= 0.0 || fly_time_2 <= 0.0)
    {
        return result;
    }

    // 选择飞行时间更短的低抛解
    if (fly_time_1 < fly_time_2)
    {
        result.pitch = pitch_1;
        result.fly_time = fly_time_1;
    }
    else
    {
        result.pitch = pitch_2;
        result.fly_time = fly_time_2;
    }

    result.success = true;
    return result;
}

class GimbalControllerNode : public rclcpp::Node
{
public:
    GimbalControllerNode();

private:
    struct AimReference
    {
        double yaw = 0.0;
        double pitch = 0.0;
        double pitch_gain = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        int armor_index = 0;
        double min_yaw = 0.0;
    };

    void TargetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg);

    void GimbalFeedCallback(auto_aim_interfaces::msg::GimbalFeed::SharedPtr msg);

    AimReference calcAimReferenceAtTime(
        double pos_t,
        double yaw_t,
        double yaw,
        double r1,
        double r2,
        double xc,
        double yc,
        double za,
        double vx,
        double vy,
        double vz,
        double dz,
        double v_yaw,
        size_t a_n,
        bool apply_pitch_gain);

    bool buildOpenSourceStyleMpcTrajectory(
        double pos_center_time,
        double yaw_center_time,
        double yaw,
        double r1,
        double r2,
        double xc,
        double yc,
        double za,
        double vx,
        double vy,
        double vz,
        double dz,
        double v_yaw,
        size_t a_n,
        std::vector<double>& yaw_ref_relative,
        std::vector<double>& pitch_ref,
        double& yaw0,
        double& yaw_start_vel,
        double& pitch_start_vel,
        AimReference& center_ref);

    //subscription
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::GimbalFeed>::SharedPtr gimbal_feed_sub_;
    //publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::GimbalControl>::SharedPtr control_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugController>::SharedPtr debug_pub_;

    //弹道解算器
    std::unique_ptr<CoordSolver> coord_solver_;
    SineScanner* pitch_scanner_;

    //mpc
    std::unique_ptr<AxisTinyMPC> yaw_mpc_;
    std::unique_ptr<AxisTinyMPC> pitch_mpc_;

    bool mpc_enable_ = true;
    int mpc_horizon_ = 100;
    int mpc_half_horizon_ = 50;
    double mpc_dt_ = 0.01;

    int lock_armor_idx_ = -1;

    //param
    double shoot_speed_; //子弹飞行速度
    double shoot_delay_; //发射延迟
    double shoot_delay_spin_; //旋转延迟

    double gimbal_delay_; //云台响应延迟，单位为秒(s)
    double max_move_yaw_; //最优装甲板的最大角度，大于这个角度就不击打，单位为度

    double fire_angle_threshold_; //开火角度阈值，yaw和pitch同时小与这个值就开火

    double z_gain_; // 整车中心坐标的xyz静态补偿
    double y_gain_;
    double x_gain_;

    // pitch/yaw静态补偿
    double pitch_gain_;
    double yaw_gain_;

    double pitch_gain_factor_; //pitch动态补偿系数

    double timestamp_offset_ = 0; //时间戳偏移量
    bool is_track_;
    bool is_pitch_gain_;

    bool sentray_mode_ = false; //哨兵模式，丢失目标后云台来回扫描
    double pitch_scan_range_; //pitch扫描幅度，单位度
    double pitch_scan_f_; //pitch扫描频率，单位Hz
    double yaw_scan_speed_; //yaw旋转速度，单位度每秒
    rclcpp::Time lost_time;

    double gimbal_yaw_ = 0; //云台的yaw，pitch的偏差，用于判断是否开火
    double gimbal_pitch_ = 0; //来源与imu数据

    double send_pitch = 0;
    double send_yaw = 0;
    double send_is_fire = 0;

    double pitch_ref_ = 0;
    double yaw_ref_ = 0;
};

#endif //BUILD_GIMBAL_CONTROLLER_NODE_H
