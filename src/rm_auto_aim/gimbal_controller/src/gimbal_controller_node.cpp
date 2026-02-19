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
    max_move_yaw_ = declare_parameter("max_move_yaw", 30.0);
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

    //subscription
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&GimbalControllerNode::TargetCallback, this, std::placeholders::_1));
    //publisher
    control_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalControl>("control/gimbal_control", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("debug/control_visuallize", 10);
    debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugController>("debug/controller", 10);

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
    if (a_n <= 0)
    {
        return;
    }
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

    // 匹配最优装甲板
    // 按照v_yaw，优先选择最接近面朝摄像头的装甲板，面朝摄像头的装甲板的yaw为0，但需要考虑一定的阈值
    // 如果最接近0的yaw大于另一个阈值，则认为没有最优装甲板，不进行射击
    double target_yaw = yaw;
    if (is_track_)
    {
        // 由于云台转动的延迟，进行最优装甲板筛选时，多预测一点，这里的delay应该比上面的delay大
        target_yaw += angular_v_c.z * (delay_spin + gimbal_delay_);
    }

    int index = 0;
    double min_yaw = 2 * M_PI;
    double c_yaw = atan2(point_c.y, point_c.x);

    for (size_t i = 0; i < a_n; i++)
    {
        double tmp_yaw = target_yaw + i * (2 * M_PI / a_n);
        //限制范围
        tmp_yaw = std::fmod(tmp_yaw + 2 * M_PI, 2 * M_PI);
        double delta_to_0 = std::fabs(tmp_yaw - c_yaw);
        double delta_to_2pi = std::fabs(tmp_yaw - (c_yaw + 2 * M_PI));
        double delta_to_zero = std::min(delta_to_0, delta_to_2pi);

        if (delta_to_zero < min_yaw)
        {
            min_yaw = delta_to_zero;
            index = i;
        }
    }

    //击打装甲板的世界坐标
    double x, y, z;

    if(is_track_){
        x = points_a_pre[index].x;
        y = points_a_pre[index].y;
        z = points_a_pre[index].z;
    }else{
        x = points_a[index].x;
        y = points_a[index].y;
        z = points_a[index].z;
    }

    // 计算需要击打的装甲板的云台姿态
    double send_pitch = atan2(z, sqrt(x * x + y * y));
    double send_yaw = -atan2(y, x);
    double send_is_fire = 0;

    auto_aim_interfaces::msg::DebugController debug_msg;
    debug_msg.send_pitch = send_pitch;
    debug_msg.armor_x = x;
    debug_msg.armor_y = y;
    debug_msg.armor_z = z;
    // 对抬枪角度进行增益
    if(is_pitch_gain_){
        pitch_gain_factor_ = get_parameter("pitch_gain_factor").as_double();
        coord_solver_->bullet_speed = shoot_speed_;
        Eigen::Vector3d xyz(x, y, z);
        double send_pitch_gain = coord_solver_->dynamicCalcPitchOffset(xyz);
        send_pitch_gain = send_pitch_gain * M_PI / 180.0;
        debug_msg.send_pitch_gain = send_pitch_gain;
        if(pitch_gain_factor_ > 10.0)
            send_pitch_gain *= xyz.norm() * (pitch_gain_factor_ - 10.0);
        else
            send_pitch_gain *= pitch_gain_factor_;
        send_pitch += send_pitch_gain;
    }

    debug_pub_->publish(debug_msg);

    //开火控制
    Eigen::Vector3d xyz(x, y, z);
    //装甲板尺寸内
    double shoot_diff = atan((armor_witch/2) / xyz.norm());
    RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver shoot_diff: %f", shoot_diff);
    static int loss_cnt = 0;
    //如果云台yaw、pitch与当前目标yaw、pitch的差值小于阈值，则认为云台已经对准目标，可以进行射击
    if( std::fabs(send_yaw - gimbal_yaw_) < fire_angle_threshold_ * shoot_diff &&
        std::fabs(send_pitch - gimbal_pitch_) < fire_angle_threshold_ * shoot_diff)
    {
        send_is_fire = 1.0;
        loss_cnt = 0;
    }else
    {
        loss_cnt ++;
        if(loss_cnt > 0)
            send_is_fire = 0.0;
        else
            send_is_fire = 1.0;
    }
    //开火窗口范围，max_move_yaw_单位度，只有在窗口内的装甲板才开火
    //TODO：是不是可以当超出窗口的时候直接去追下一个装甲板，而不是只不开火
    max_move_yaw_ = get_parameter("max_move_yaw").as_double();
    if(min_yaw > max_move_yaw_ * M_PI / 180){
        RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "No optimal armor, now min yaw: %f", min_yaw * 180 / M_PI);
        send_is_fire = 0.0;
    }

    auto_aim_interfaces::msg::GimbalControl control_msg;
    control_msg.is_fire = send_is_fire;
    control_msg.yaw = send_yaw;
    control_msg.pitch = send_pitch;
    control_pub_->publish(control_msg);

    //===================DEBUG=======================
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker shoot_point_marker;
    shoot_point_marker.header.frame_id = "odom";
    shoot_point_marker.header.stamp = now();
    shoot_point_marker.ns = "shoot_point";
    shoot_point_marker.id = 0;
    shoot_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
    shoot_point_marker.action = visualization_msgs::msg::Marker::ADD;
    shoot_point_marker.pose.position.x = points_a_pre[index].x;
    shoot_point_marker.pose.position.y = points_a_pre[index].y;
    shoot_point_marker.pose.position.z = points_a_pre[index].z;
    shoot_point_marker.scale.x = 0.04;
    shoot_point_marker.scale.y = 0.04;
    shoot_point_marker.scale.z = 0.04;
    shoot_point_marker.color.r = 0.0;
    shoot_point_marker.color.g = 0.1;
    shoot_point_marker.color.b = 0.5;
    shoot_point_marker.color.a = 0.4;
    markers.markers.push_back(shoot_point_marker);

    visualization_msgs::msg::Marker target_armor_marker;
    target_armor_marker.header.frame_id = "odom";
    target_armor_marker.header.stamp = now();
    target_armor_marker.ns = "target_point";
    target_armor_marker.id = 0;
    target_armor_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_armor_marker.action = visualization_msgs::msg::Marker::ADD;
    target_armor_marker.pose.position.x = points_a[index].x;
    target_armor_marker.pose.position.y = points_a[index].y;
    target_armor_marker.pose.position.z = points_a[index].z;
    target_armor_marker.scale.x = 0.02;
    target_armor_marker.scale.y = 0.02;
    target_armor_marker.scale.z = 0.02;
    target_armor_marker.color.r = 1.0;
    target_armor_marker.color.g = 0.6;
    target_armor_marker.color.b = 0.0;
    target_armor_marker.color.a = 0.9;
    markers.markers.push_back(target_armor_marker);

    visualization_msgs::msg::Marker yaw_marker;
    yaw_marker.header.frame_id = "odom";
    yaw_marker.header.stamp = now();
    yaw_marker.ns = "bullet_path";
    yaw_marker.id = 0;
    yaw_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    yaw_marker.action = visualization_msgs::msg::Marker::ADD;
    yaw_marker.scale.x = 0.01;
    yaw_marker.scale.y = 0.01;
    yaw_marker.scale.z = 0.01;
    yaw_marker.color.r = 1.0;
    yaw_marker.color.g = 0.5;
    yaw_marker.color.b = 0.5;
    yaw_marker.color.a = 0.2;
    geometry_msgs::msg::Point p0;
    p0.x = 0; p0.y = 0; p0.z = 0;
    geometry_msgs::msg::Point pa;
    pa.x = points_a[index].x;
    pa.y = points_a[index].y;
    pa.z = points_a[index].z;
    yaw_marker.points.push_back(p0);
    yaw_marker.points.push_back(point_c);
    yaw_marker.points.push_back(pa);
    markers.markers.push_back(yaw_marker);

    yaw_marker.id = 2;
    yaw_marker.color.r = 0.0;
    yaw_marker.color.g = 1.0;
    yaw_marker.color.b = 0.5;
    yaw_marker.color.a = 0.2;
    yaw_marker.points.clear();
    yaw_marker.points.push_back(point_c);
    pa.x = points_a_pre[index].x;
    pa.y = points_a_pre[index].y;
    pa.z = points_a_pre[index].z;
    yaw_marker.points.push_back(pa);
    markers.markers.push_back(yaw_marker);

    marker_array_pub_->publish(markers);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}