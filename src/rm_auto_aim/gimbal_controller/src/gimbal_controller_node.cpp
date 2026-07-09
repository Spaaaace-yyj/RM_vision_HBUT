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

    //mpc
    mpc_horizon_ = declare_parameter("mpc.horizon", 100);
    mpc_half_horizon_ = declare_parameter("mpc.half_horizon", 50);
    mpc_dt_ = declare_parameter("mpc.dt", 0.01);
    mpc_enable_ = declare_parameter("mpc.enable", true);

    if (mpc_horizon_ < 4)
    {
        mpc_horizon_ = 100;
    }
    if (mpc_half_horizon_ <= 0 || mpc_half_horizon_ >= mpc_horizon_)
    {
        mpc_half_horizon_ = mpc_horizon_ / 2;
    }

    AxisTinyMPC::Config yaw_cfg;
    yaw_cfg.dt = mpc_dt_;
    yaw_cfg.horizon = mpc_horizon_;
    yaw_cfg.output_index = mpc_half_horizon_;
    yaw_cfg.max_acc = declare_parameter("mpc.yaw.max_acc", 60.0);
    yaw_cfg.max_vel = declare_parameter("mpc.yaw.max_vel", 50.0);
    yaw_cfg.q_pos = declare_parameter("mpc.yaw.q_pos", 1e6);
    yaw_cfg.q_vel = declare_parameter("mpc.yaw.q_vel", 0.0);
    yaw_cfg.r_acc = declare_parameter("mpc.yaw.r_acc", 10.0);
    yaw_cfg.rho = declare_parameter("mpc.yaw.rho", 1.0);
    yaw_cfg.max_iter = declare_parameter("mpc.yaw.max_iter", 10);

    AxisTinyMPC::Config pitch_cfg;
    pitch_cfg.dt = mpc_dt_;
    pitch_cfg.horizon = mpc_horizon_;
    pitch_cfg.output_index = mpc_half_horizon_;
    pitch_cfg.max_acc = declare_parameter("mpc.pitch.max_acc", 40.0);
    pitch_cfg.max_vel = declare_parameter("mpc.pitch.max_vel", 20.0);
    pitch_cfg.q_pos = declare_parameter("mpc.pitch.q_pos", 9e6);
    pitch_cfg.q_vel = declare_parameter("mpc.pitch.q_vel", 0.0);
    pitch_cfg.r_acc = declare_parameter("mpc.pitch.r_acc", 1.0);
    pitch_cfg.rho = declare_parameter("mpc.pitch.rho", 1.0);
    pitch_cfg.max_iter = declare_parameter("mpc.pitch.max_iter", 10);

    if (mpc_enable_)
    {
        yaw_mpc_ = std::make_unique<AxisTinyMPC>(yaw_cfg);
        pitch_mpc_ = std::make_unique<AxisTinyMPC>(pitch_cfg);
    }

    //哨兵扫描寻敌
    sentray_mode_ = declare_parameter("sentray_mode", false);
    pitch_scan_range_ = declare_parameter("pitch_scan_range", 20.0);
    pitch_scan_f_ = declare_parameter("pitch_scan_f", 1.0);
    yaw_scan_speed_ = declare_parameter("yaw_scan_speed", 360.0);
    pitch_scanner_ = new SineScanner(pitch_scan_range_ * (M_PI / 180.0), pitch_scan_f_);

    //subscription
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS(),
        std::bind(&GimbalControllerNode::TargetCallback, this, std::placeholders::_1));
    gimbal_feed_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalFeed>(
        "/gimbal_feed", rclcpp::SensorDataQoS(),
        std::bind(&GimbalControllerNode::GimbalFeedCallback, this, std::placeholders::_1));
    //publisher
    control_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalControl>("control/gimbal_control", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("debug/control_visuallize", 10);
    debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugController>("debug/controller", 10);

    RCLCPP_INFO(this->get_logger(), "Gimbal Controller Init!");
}


static double shortestAngularDistance(double from, double to)
{
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

static double limitRad(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

GimbalControllerNode::AimReference GimbalControllerNode::calcAimReferenceAtTime(
    double t,
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
    bool apply_pitch_gain)
{
    AimReference ref;

    if (a_n == 0)
    {
        return ref;
    }

    double xc_pre = xc + vx * t;
    double yc_pre = yc + vy * t;
    double za_pre = za + vz * t;

    double yaw_pre = yaw + v_yaw * t;

    int best_index = 0;
    double min_yaw = 2.0 * M_PI;
    double c_yaw = std::atan2(yc_pre, xc_pre);

    for (size_t i = 0; i < a_n; ++i)
    {
        double tmp_yaw = yaw_pre + i * (2.0 * M_PI / static_cast<double>(a_n));
        tmp_yaw = std::fmod(tmp_yaw + 2.0 * M_PI, 2.0 * M_PI);

        double delta_to_0 = std::fabs(tmp_yaw - c_yaw);
        double delta_to_2pi = std::fabs(tmp_yaw - (c_yaw + 2.0 * M_PI));
        double delta_to_zero = std::min(delta_to_0, delta_to_2pi);

        if (delta_to_zero < min_yaw)
        {
            min_yaw = delta_to_zero;
            best_index = static_cast<int>(i);
        }
    }

    bool use_second_pair = false;
    if (a_n == 4)
    {
        use_second_pair = (best_index % 2 == 1);
    }

    double r = use_second_pair ? r2 : r1;
    double z = za_pre + (use_second_pair ? dz : 0.0);
    double armor_yaw = yaw_pre + best_index * (2.0 * M_PI / static_cast<double>(a_n));

    double x = xc_pre - r * std::cos(armor_yaw);
    double y = yc_pre - r * std::sin(armor_yaw);

    ref.yaw = -std::atan2(y, x);
    ref.pitch = std::atan2(z, std::sqrt(x * x + y * y));
    ref.x = x;
    ref.y = y;
    ref.z = z;
    ref.armor_index = best_index;
    ref.min_yaw = min_yaw;

    if (apply_pitch_gain && is_pitch_gain_)
    {
        pitch_gain_factor_ = get_parameter("pitch_gain_factor").as_double();
        coord_solver_->bullet_speed = shoot_speed_;
        Eigen::Vector3d xyz(x, y, z);
        double send_pitch_gain = coord_solver_->dynamicCalcPitchOffset(xyz);
        send_pitch_gain = send_pitch_gain * M_PI / 180.0;
        if (pitch_gain_factor_ > 10.0)
            send_pitch_gain *= xyz.norm() * (pitch_gain_factor_ - 10.0);
        else
            send_pitch_gain *= pitch_gain_factor_;
        ref.pitch += send_pitch_gain;
        ref.pitch_gain = send_pitch_gain;
    }

    return ref;
}

bool GimbalControllerNode::buildOpenSourceStyleMpcTrajectory(
    double center_time,
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
    AimReference& center_ref)
{
    if (a_n == 0 || mpc_horizon_ <= 1 || mpc_half_horizon_ <= 0 || mpc_half_horizon_ >= mpc_horizon_)
    {
        return false;
    }

    std::vector<double> yaw_ref_abs;
    yaw_ref_abs.reserve(mpc_horizon_);
    pitch_ref.reserve(mpc_horizon_);
    yaw_ref_relative.reserve(mpc_horizon_);

    for (int i = 0; i < mpc_horizon_; ++i)
    {
        double t = center_time + (static_cast<double>(i - mpc_half_horizon_) * mpc_dt_);
        auto ref = calcAimReferenceAtTime(
            t, yaw, r1, r2, xc, yc, za, vx, vy, vz, dz, v_yaw, a_n, true);

        if (i == mpc_half_horizon_)
        {
            center_ref = ref;
        }

        yaw_ref_abs.push_back(ref.yaw);
        pitch_ref.push_back(ref.pitch);
    }

    yaw0 = yaw_ref_abs[mpc_half_horizon_];
    for (int i = 0; i < mpc_horizon_; ++i)
    {
        yaw_ref_relative.push_back(shortestAngularDistance(yaw0, yaw_ref_abs[i]));
    }

    if (mpc_horizon_ >= 2)
    {
        yaw_start_vel = shortestAngularDistance(yaw_ref_relative[0], yaw_ref_relative[1]) / mpc_dt_;
        pitch_start_vel = (pitch_ref[1] - pitch_ref[0]) / mpc_dt_;
    }
    else
    {
        yaw_start_vel = 0.0;
        pitch_start_vel = 0.0;
    }

    return true;
}

void GimbalControllerNode::TargetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
    static bool yaw_swap_time_init = false;
    static rclcpp::Time last_time = this->now();
    static int fps_tmp = 0;
    static int fps = 0;
    auto start_time = this->now();

    if ((start_time - last_time).seconds() < 1)
    {
        fps_tmp++;
    }
    else
    {
        fps = fps_tmp;
        RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "Gimbal Serial update FPS: %d", fps);
        fps_tmp = 0;
        last_time = start_time;
    }

    if (!yaw_swap_time_init)
    {
        lost_time = this->now();
        yaw_swap_time_init = true;
    }

    auto_aim_interfaces::msg::GimbalControl control_msg;

    double yaw = msg->yaw, r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
    double zc = za + za / 2;
    double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;
    double dz = msg->dz;
    double v_yaw = msg->v_yaw;
    double armor_witch = msg->type == "large" ? 0.225 : 0.135;
    size_t a_n = msg->armors_num;
    // if (!msg->tracking) lock_armor_idx_ = -1;

    sentray_mode_ = get_parameter("sentray_mode").as_bool();
    if (a_n > 0)
    {
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
        for (size_t i = 0; i < a_n; i++)
        {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4)
            {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            }
            else
            {
                r = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);
            points_a.push_back(p_a);
        }

        // 子弹飞行速度为, 发单延迟为 0.1s
        shoot_speed_ = get_parameter("shoot_speed").as_double();
        shoot_delay_ = get_parameter("shoot_delay").as_double();
        shoot_delay_spin_ = get_parameter("shoot_delay_spin_").as_double();

        // 子弹飞行时间加上发单延迟
        double delay_translation = shoot_delay_ + sqrt(xc * xc + yc * yc + zc * zc) / shoot_speed_;

        // 整车预测坐标
        geometry_msgs::msg::Point point_c_pre;
        point_c_pre.x = point_c.x + velocity_c.x * delay_translation;
        point_c_pre.y = point_c.y + velocity_c.y * delay_translation;
        point_c_pre.z = point_c.z + velocity_c.z * delay_translation;

        // 整车角度预测
        // TODO: 角度预测时间要短些
        double delay_spin = shoot_delay_spin_ + sqrt(xc * xc + yc * yc + zc * zc) / shoot_speed_;
        double yaw_pre = yaw + angular_v_c.z * delay_spin;

        //装甲板坐标预测
        is_current_pair = true;
        std::vector<geometry_msgs::msg::Point> points_a_pre;
        r = 0;
        for (size_t i = 0; i < a_n; i++)
        {
            double tmp_yaw = yaw_pre + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4)
            {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            }
            else
            {
                r = r1;
                p_a.z = za;
            }
            p_a.x = point_c_pre.x - r * cos(tmp_yaw);
            p_a.y = point_c_pre.y - r * sin(tmp_yaw);
            points_a_pre.push_back(p_a);
        }

        // 匹配最优装甲板
        // 按照v_yaw，优先选择最接近面朝摄像头的装甲板，面朝摄像头的装甲板的yaw为0
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

        // double enter_angle = 40.0 * M_PI / 180.0;
        // double leave_angle = 70.0 * M_PI / 180.0;
        // if (lock_armor_idx_ == -1)
        // {
        //     // 未锁定
        //     if (min_yaw < enter_angle)
        //     {
        //         lock_armor_idx_ = index;
        //     }
        // }
        // else
        // {
        //     // 已锁定
        //     double lock_yaw = target_yaw + lock_armor_idx_ * (2 * M_PI / a_n);
        //
        //     double delta_to_0 = fabs(lock_yaw - c_yaw);
        //     double delta_to_2pi = fabs(lock_yaw - (c_yaw + 2 * M_PI));
        //     double lock_delta = std::min(delta_to_0, delta_to_2pi);
        //
        //     if (lock_delta > leave_angle)
        //     {
        //         lock_armor_idx_ = -1;
        //     }
        // }
        //
        // if (lock_armor_idx_ != -1)
        //     index = lock_armor_idx_;

        //击打装甲板的世界坐标
        double x, y, z;

        if (is_track_)
        {
            x = points_a_pre[index].x;
            y = points_a_pre[index].y;
            z = points_a_pre[index].z;
        }
        else
        {
            x = points_a[index].x;
            y = points_a[index].y;
            z = points_a[index].z;
        }

        auto_aim_interfaces::msg::DebugController debug_msg;
        // 计算需要击打的装甲板的云台姿态
        send_pitch = atan2(z, sqrt(x * x + y * y));
        send_yaw = -atan2(y, x);
        send_is_fire = 0;
        pitch_ref_ = send_pitch;
        yaw_ref_ = send_yaw;

        bool mpc_used = false;
        AimReference center_ref;
        std::vector<double> yaw_ref_relative;
        std::vector<double> pitch_ref;
        double yaw0 = send_yaw;
        double yaw_start_vel = 0.0;
        double pitch_start_vel = 0.0;

        double mpc_center_time = is_track_ ? delay_spin : 0.0;
        bool trajectory_ready = buildOpenSourceStyleMpcTrajectory(
            mpc_center_time,
            yaw,
            r1,
            r2,
            xc,
            yc,
            za,
            vx,
            vy,
            vz,
            dz,
            v_yaw,
            a_n,
            yaw_ref_relative,
            pitch_ref,
            yaw0,
            yaw_start_vel,
            pitch_start_vel,
            center_ref);

        if (trajectory_ready)
        {
            x = center_ref.x;
            y = center_ref.y;
            z = center_ref.z;
            index = center_ref.armor_index;
            min_yaw = center_ref.min_yaw;
            send_yaw = center_ref.yaw;
            send_pitch = center_ref.pitch;
        }

        if (trajectory_ready &&
            static_cast<int>(yaw_ref_relative.size()) > mpc_half_horizon_ &&
            static_cast<int>(pitch_ref.size()) > mpc_half_horizon_)
        {
            debug_msg.yaw_ref = limitRad(yaw_ref_relative[mpc_half_horizon_] + yaw0);
            debug_msg.pitch_ref = pitch_ref[mpc_half_horizon_];
        }
        else
        {
            debug_msg.yaw_ref = send_yaw;
            debug_msg.pitch_ref = send_pitch;
        }

        if (mpc_enable_ && yaw_mpc_ && pitch_mpc_ && trajectory_ready &&
            static_cast<int>(yaw_ref_relative.size()) >= mpc_horizon_ &&
            static_cast<int>(pitch_ref.size()) >= mpc_horizon_)
        {
            auto yaw_cmd = yaw_mpc_->solve(yaw_ref_relative[0], yaw_start_vel, yaw_ref_relative);
            auto pitch_cmd = pitch_mpc_->solve(pitch_ref[0], pitch_start_vel, pitch_ref);

            if (yaw_cmd.success)
            {
                send_yaw = limitRad(yaw_cmd.pos + yaw0);
                mpc_used = true;
            }
            // if (pitch_cmd.success)
            // {
            //     send_pitch = pitch_cmd.pos;
            //     mpc_used = true;
            // }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "[MPC SOLVE FAILED] yaw_success=%d pitch_success=%d yaw0=%.4f yaw_start_vel=%.4f pitch0=%.4f pitch_start_vel=%.4f yaw_ref0=%.4f yaw_ref_mid=%.4f yaw_ref_last=%.4f",
                    yaw_cmd.success,
                    pitch_cmd.success,
                    yaw0,
                    yaw_start_vel,
                    pitch_ref.empty() ? 0.0 : pitch_ref[0],
                    pitch_start_vel,
                    yaw_ref_relative.empty() ? 0.0 : yaw_ref_relative[0],
                    yaw_ref_relative.size() > static_cast<size_t>(mpc_half_horizon_) ? yaw_ref_relative[mpc_half_horizon_] : 0.0,
                    yaw_ref_relative.empty() ? 0.0 : yaw_ref_relative.back());
            }
        }else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "[MPC SKIP] enable=%d yaw_mpc=%d pitch_mpc=%d trajectory_ready=%d yaw_size=%ld pitch_size=%ld horizon=%d",
                mpc_enable_,
                static_cast<bool>(yaw_mpc_),
                static_cast<bool>(pitch_mpc_),
                trajectory_ready,
                yaw_ref_relative.size(),
                pitch_ref.size(),
                mpc_horizon_);
        }

        debug_msg.pitch_send = send_pitch;
        debug_msg.yaw_send = send_yaw;
        debug_msg.armor_x = x;
        debug_msg.armor_y = y;
        debug_msg.armor_z = z;
        if (mpc_used)
        {
            debug_msg.send_pitch_gain = center_ref.pitch_gain;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "[MPC] Failed enable=%d yaw_mpc=%d pitch_mpc=%d trajectory_ready=%d yaw_size=%ld pitch_size=%ld horizon=%d",
                mpc_enable_,
                static_cast<bool>(yaw_mpc_),
                static_cast<bool>(pitch_mpc_),
                trajectory_ready,
                yaw_ref_relative.size(),
                pitch_ref.size(),
                mpc_horizon_);
        }
        // 对抬枪角度进行增益
        if (is_pitch_gain_ && !mpc_used)
        {
            pitch_gain_factor_ = get_parameter("pitch_gain_factor").as_double();
            coord_solver_->bullet_speed = shoot_speed_;
            Eigen::Vector3d xyz(x, y, z);
            double send_pitch_gain = coord_solver_->dynamicCalcPitchOffset(xyz);
            send_pitch_gain = send_pitch_gain * M_PI / 180.0;
            debug_msg.send_pitch_gain = send_pitch_gain;
            if (pitch_gain_factor_ > 10.0)
                send_pitch_gain *= xyz.norm() * (pitch_gain_factor_ - 10.0);
            else
                send_pitch_gain *= pitch_gain_factor_;
            send_pitch += send_pitch_gain;
        }

        debug_pub_->publish(debug_msg);

        //开火控制
        Eigen::Vector3d xyz(x, y, z);
        //装甲板尺寸内
        double shoot_diff = atan((armor_witch / 2) / xyz.norm());
        RCLCPP_DEBUG(rclcpp::get_logger("lc_serial"), "SerialDriver shoot_diff: %f", shoot_diff);
        static int loss_cnt = 0;
        //如果云台yaw、pitch与当前目标yaw、pitch的差值小于阈值，则认为云台已经对准目标，可以进行射击
        if (std::fabs(yaw_ref_ - gimbal_yaw_) < fire_angle_threshold_ * shoot_diff ||
            std::fabs(pitch_ref_ - gimbal_pitch_) < fire_angle_threshold_ * shoot_diff)
        {
            send_is_fire = 1.0;
            loss_cnt = 0;
        }
        else
        {
            loss_cnt++;
            if (loss_cnt > 0)
                send_is_fire = 0.0;
            else
                send_is_fire = 1.0;
        }
        //开火窗口范围，max_move_yaw_单位度，只有在窗口内的装甲板才开火
        //TODO：是不是可以当超出窗口的时候直接去追下一个装甲板，而不是只不开火
        max_move_yaw_ = get_parameter("max_move_yaw").as_double();
        if (min_yaw > max_move_yaw_ * M_PI / 180)
        {
            RCLCPP_WARN(rclcpp::get_logger("lc_serial"), "No optimal armor, now min yaw: %f", min_yaw * 180 / M_PI);
            send_is_fire = 0.0;
        }
        control_msg.is_fire = send_is_fire;
        control_msg.yaw = send_yaw;
        control_msg.pitch = send_pitch;
        control_msg.tracing = 1.0;

        //===================DEBUG=======================
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker shoot_point_marker;
        shoot_point_marker.header.frame_id = "odom";
        shoot_point_marker.header.stamp = now();
        shoot_point_marker.ns = "shoot_point";
        shoot_point_marker.id = 0;
        shoot_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        shoot_point_marker.action = visualization_msgs::msg::Marker::ADD;
        shoot_point_marker.pose.position.x = x;
        shoot_point_marker.pose.position.y = y;
        shoot_point_marker.pose.position.z = z;
        shoot_point_marker.scale.x = 0.1;
        shoot_point_marker.scale.y = 0.1;
        shoot_point_marker.scale.z = 0.1;
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
        target_armor_marker.scale.x = 0.05;
        target_armor_marker.scale.y = 0.05;
        target_armor_marker.scale.z = 0.05;
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
        yaw_marker.color.a = 0.9;
        geometry_msgs::msg::Point p0;
        p0.x = 0;
        p0.y = 0;
        p0.z = 0;
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
        yaw_marker.color.a = 0.9;
        yaw_marker.points.clear();
        yaw_marker.points.push_back(point_c);
        pa.x = x;
        pa.y = y;
        pa.z = z;
        yaw_marker.points.push_back(pa);
        markers.markers.push_back(yaw_marker);
        marker_array_pub_->publish(markers);
        lost_time = this->now();
    }
    else
    {
        double dt = (lost_time - this->now()).seconds();
        control_msg.tracing = 0.0;
        if (sentray_mode_)
        {
            control_msg.is_fire = 0;
            control_msg.pitch = pitch_scanner_->getValue() + (-5.0 * (M_PI / 180.0));
            control_msg.yaw = send_yaw + yaw_scan_speed_ * (M_PI / 180.0) * dt;
            //TODO:这个IMU角度范围是360还是正负180？
            while (control_msg.yaw - gimbal_yaw_ > M_PI)
                control_msg.yaw -= 2.0 * M_PI;

            while (control_msg.yaw - gimbal_yaw_ < -M_PI)
                control_msg.yaw += 2.0 * M_PI;
        }
    }
    control_pub_->publish(control_msg);
}

void GimbalControllerNode::GimbalFeedCallback(auto_aim_interfaces::msg::GimbalFeed::SharedPtr msg)
{
    gimbal_yaw_ = msg->yaw;
    gimbal_pitch_ = msg->pitch;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
