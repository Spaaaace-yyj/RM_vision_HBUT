//
// Created by spaaaaace on 2026/2/17.
//

#include "../include/gimbal_controller/gimbal_controller_node.h"

GimbalControllerNode::GimbalControllerNode() : Node("GimbalControllerNode")
{
    //参数初始化
    shoot_speed_ = declare_parameter("shoot_speed", 15.0);
    shoot_delay_ = declare_parameter("shoot_delay", 0.4);
    shoot_delay_spin_ = declare_parameter("shoot_delay_spin_", 0.2);
    gimbal_delay_ = declare_parameter("gimbal_delay", 0.1);
    max_move_yaw_ = declare_parameter("max_move_yaw", 0.0);
    fire_angle_threshold_ = declare_parameter("fire_angle_threshold", 1.0);
    z_gain_ = declare_parameter("z_gain", 0.0);
    y_gain_ = declare_parameter("y_gain", 0.0);
    x_gain_ = declare_parameter("x_gain", 0.0);
    pitch_gain_factor_ = declare_parameter("pitch_gain_factor", 1.0);
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    is_track_ = declare_parameter("is_track", true);
    is_pitch_gain_ = declare_parameter("is_pitch_gain", true);

    // 弹道解算器初始化, 定义参数
    int max_iter = declare_parameter("max_iter", 10);
    float stop_error = declare_parameter("stop_error", 0.001);
    int R_K_iter = declare_parameter("R_K_iter", 50);
    coord_solver_ = std::make_unique<CoordSolver>(max_iter, stop_error, R_K_iter);

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&GimbalControllerNode::TargetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gimbal Controller Init!");
}

void GimbalControllerNode::TargetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
    static rclcpp::Time last_time = this->now();
    static int fps_tmp = 0;
    static int fps = 0;
    auto start_time = this->now();

    if((start_time - last_time).seconds() < 1){
        fps_tmp++;
    }else{
        fps = fps_tmp;
        RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "Gimbal Serial update FPS: %d", fps);
        fps_tmp = 0;
        last_time = start_time;
    }

    double yaw = msg->yaw, r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
    double zc = za + za / 2;
    double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;
    double dz = msg->dz;
    double v_yaw = msg->v_yaw;
    double armor_witch = msg->type == "large" ? 0.225 : 0.135 ;
    size_t a_n = msg->armors_num;

    z_gain_ = get_parameter("z_gain").as_double();
    y_gain_ = get_parameter("y_gain").as_double();
    x_gain_ = get_parameter("x_gain").as_double();

    za += z_gain_;
    yc += y_gain_;
    xc += x_gain_;

    //整车中心坐标
    geometry_msgs::msg::Point point_c;
    point_c.x = xc;
    point_c.y = yc;
    point_c.z = za + dz / 2;

    // 整车速度
    geometry_msgs::msg::Point velocity_c;
    velocity_c.x = vx;
    velocity_c.y = vy;
    velocity_c.z = vz;

    // 整车角速度
    geometry_msgs::msg::Point angular_v_c;
    // 只是为了方便调试，在rviz2中显示，实际上只用到了 z 轴的角速度
    angular_v_c.x = xc;
    angular_v_c.y = yc;
    angular_v_c.z = v_yaw;

    //装甲板坐标
    bool is_current_pair = true;
    std::vector<geometry_msgs::msg::Point> points_a;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
        double tmp_yaw = yaw + i * (2 * M_PI / a_n);
        // Only 4 armors has 2 radius and height
        if (a_n == 4) {
            r = is_current_pair ? r1 : r2;
            p_a.z = za + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;
        } else {
            r = r1;
            p_a.z = za;
        }
        p_a.x = xc - r * cos(tmp_yaw);
        p_a.y = yc - r * sin(tmp_yaw);
        points_a.push_back(p_a);
    }

    // 子弹飞行速度为 15m/s, 发单延迟为 0.1s
    shoot_speed_ = get_parameter("shoot_speed").as_double();
    shoot_delay_ = get_parameter("shoot_delay").as_double();
    shoot_delay_spin_ = get_parameter("shoot_delay_spin_").as_double();

    // 子弹飞行时间加上发单延迟
    double delay_translation = shoot_delay_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;

    // 整车预测坐标
    geometry_msgs::msg::Point point_c_pre;
    point_c_pre.x = point_c.x + velocity_c.x * delay_translation;
    point_c_pre.y = point_c.y + velocity_c.y * delay_translation;
    point_c_pre.z = point_c.z + velocity_c.z * delay_translation;

    // 整车角度预测
    // TODO: 角度预测时间要短些
    double delay_spin = shoot_delay_spin_ + sqrt(xc*xc + yc*yc + zc*zc) / shoot_speed_;
    double yaw_pre = yaw + angular_v_c.z * delay_spin;

    //装甲板坐标预测
    is_current_pair = true;
    std::vector<geometry_msgs::msg::Point> points_a_pre;
    r = 0;
    for (size_t i = 0; i < a_n; i++) {
        double tmp_yaw = yaw_pre + i * (2 * M_PI / a_n);
        // Only 4 armors has 2 radius and height
        if (a_n == 4) {
            r = is_current_pair ? r1 : r2;
            p_a.z = za + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;
        } else {
            r = r1;
            p_a.z = za;
        }
        p_a.x = point_c_pre.x - r * cos(tmp_yaw);
        p_a.y = point_c_pre.y - r * sin(tmp_yaw);
        points_a_pre.push_back(p_a);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}