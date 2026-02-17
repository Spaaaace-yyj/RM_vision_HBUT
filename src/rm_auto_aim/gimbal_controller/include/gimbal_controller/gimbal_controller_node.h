//
// Created by spaaaaace on 2026/2/17.
//

#ifndef BUILD_GIMBAL_CONTROLLER_NODE_H
#define BUILD_GIMBAL_CONTROLLER_NODE_H

//Ros2
#include <rclcpp/rclcpp.hpp>
//Interface
#include "auto_aim_interfaces/msg/target.hpp"


#include "coordsolver.h"

class GimbalControllerNode : public rclcpp::Node
{
public:
    GimbalControllerNode();

private:
    void TargetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg);

    //subscription
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

    //弹道解算器
    std::unique_ptr<CoordSolver> coord_solver_;

    //param
    double shoot_speed_;        //子弹飞行速度
    double shoot_delay_;        //发射延迟
    double shoot_delay_spin_;   //旋转延迟

    double gimbal_delay_;       //云台响应延迟，单位为秒(s)
    double max_move_yaw_;       //最优装甲板的最大角度，大于这个角度就不击打，单位为度

    double fire_angle_threshold_;   //开火角度阈值，yaw和pitch同时小与这个值就开火

    double z_gain_;             //整车中心坐标的xyz静态补偿
    double y_gain_;
    double x_gain_;

    double pitch_gain_factor_;  //pitch动态补偿系数

    double timestamp_offset_ = 0;   //时间戳偏移量
    bool is_track_;
    bool is_pitch_gain_;

    double gimbal_yaw_;         //云台的yaw，pitch的偏差，用于判断是否开火
    double gimbal_pitch_;
};

#endif //BUILD_GIMBAL_CONTROLLER_NODE_H