#include "outpost_detector/outpost_target_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace outpost_auto_aim {

namespace {

constexpr double kPlateStep = 2.0 * M_PI / 3.0;  // 三块板 120° 分布
constexpr double kMaxDt = 1.0;                   // predict 最大步长，覆盖旋转空窗

// 角度差取最小回绕
double wrapAngle(double a) {
    return std::remainder(a, 2.0 * M_PI);
}

}  // namespace

OutpostTargetNode::OutpostTargetNode(const rclcpp::NodeOptions& options)
    : Node("outpost_target_node", options),
      tf2_buffer_(this->get_clock()),
      tf2_listener_(tf2_buffer_) {
    // ---- 参数 ----
    target_frame_ = this->declare_parameter("target_frame", "odom");
    camera_frame_ = this->declare_parameter("camera_frame", "camera_optical_frame");
    const std::vector<double> slot_z =
        this->declare_parameter("slot_z", std::vector<double>{1.10, 1.27, 1.45});
    for (int i = 0; i < 3; ++i) {
        slot_z_[i] = slot_z[i];
    }
    z_tol_ = this->declare_parameter("z_tol", 0.08);
    r_initial_ = this->declare_parameter("r_initial", 0.28);
    rotate_direction_ = this->declare_parameter("rotate_direction", 0);
    if (rotate_direction_ != 0) {
        v_yaw_eff_ = rotate_direction_ * 0.8 * M_PI;
    }
    gate_dist_ = this->declare_parameter("gate_dist", 1.0);
    temp_lost_time_ = this->declare_parameter("temp_lost_time", 1.5);
    lost_time_ = this->declare_parameter("lost_time", 5.0);
    tracking_thres_ = this->declare_parameter("tracking_thres", 5);
    s2q_xyz_ = this->declare_parameter("kf.sigma2_q_xyz", 100.0);
    s2q_yaw_ = this->declare_parameter("kf.sigma2_q_yaw", 1.0);
    r_meas_xy_ = this->declare_parameter("kf.r_meas_xy", 0.05);
    r_meas_z_ = this->declare_parameter("kf.r_meas_z", 0.03);
    outpost_x_ = this->declare_parameter("outpost_x", 0.0);
    outpost_y_ = this->declare_parameter("outpost_y", 0.0);
    outpost_radius_ = this->declare_parameter("outpost_radius", 0.0);
    accept_any_small_ = this->declare_parameter("accept_any_small", false);
    debug_ = this->declare_parameter("debug", false);

    // ---- EKF 模型，匀速圆周运动，参照 armor_tracker ----
    // 轴心静止没有速度状态，半径 r 和转速都用参数，不在状态里，
    // 正对窗口内观测对 r 不敏感，放进状态会被残差拉塌
    auto f = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd x_new = x;
        x_new(ST_YAW) = wrapAngle(x(ST_YAW) + v_yaw_eff_ * dt_);
        return x_new;
    };
    auto j_f = [](const Eigen::VectorXd&) {
        return Eigen::MatrixXd::Identity(ST_SIZE, ST_SIZE);
    };
    // 观测模型按板索引生成，这里只放占位
    auto h_dummy = [](const Eigen::VectorXd&) {
        return Eigen::VectorXd::Zero(3);
    };
    auto j_h_dummy = [](const Eigen::VectorXd&) {
        return Eigen::MatrixXd::Zero(3, ST_SIZE);
    };
    auto u_q = [this]() {
        Eigen::MatrixXd q = Eigen::MatrixXd::Zero(ST_SIZE, ST_SIZE);
        // 轴心静止、转速恒定，预测误差都来自模型偏差，给很小的噪声
        q(ST_XC, ST_XC) = 1e-3;
        q(ST_YC, ST_YC) = 1e-3;
        q(ST_YAW, ST_YAW) = 1e-3;
        // 高度是结构参数，几乎不变，给很小的过程噪声
        for (int j = 0; j < 3; ++j) {
            q(ST_Z0 + j, ST_Z0 + j) = 1e-4;
        }
        return q;
    };
    auto u_r = [](const Eigen::VectorXd&) {
        return Eigen::MatrixXd::Identity(3, 3);
    };
    Eigen::MatrixXd p0 = Eigen::MatrixXd::Identity(ST_SIZE, ST_SIZE);
    ekf_ = rm_auto_aim::ExtendedKalmanFilter{f, h_dummy, j_f, j_h_dummy, u_q, u_r, p0};

    // ---- 订阅 / 发布 ----
    armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](auto_aim_interfaces::msg::Armors::SharedPtr msg) {
            armorsCallback(msg);
        });
    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS());
    // 板转走时没有检测消息，用定时器持续外推发布，云台提前转到下一块正对板
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { timerTick(); });
    if (debug_) {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/outpost/marker", 10);
    }
}

void OutpostTargetNode::armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    // 回放没有时间戳时用节点时钟兜底
    double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const bool has_stamp = now_sec >= 1e-6;
    if (!has_stamp) {
        now_sec = this->now().seconds();
    }
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();

    // 1. 逐块变换到目标系，变换失败整帧丢弃
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(msg->armors.size());
    for (const auto& armor : msg->armors) {
        geometry_msgs::msg::Pose pose;
        if (!transformToTarget(armor.pose, pose, msg->header.frame_id, stamp)) {
            return;
        }
        poses.push_back(pose);
    }

    // 2. EKF 先按实际帧间隔推进
    dt_ = std::clamp(now_sec - last_predict_time_, 0.001, kMaxDt);
    last_predict_time_ = now_sec;
    if (ekf_inited_) {
        ekf_.predict();
    }

    // 3. 逐块观测更新
    bool matched = false;
    for (std::size_t i = 0; i < msg->armors.size(); ++i) {
        const auto& armor = msg->armors[i];
        // 只要识别成前哨站的板，识别不出来时可以兜底收小装甲板
        if (armor.number != "outpost") {
            if (!accept_any_small_ || armor.type != "small") {
                continue;
            }
        }

        const geometry_msgs::msg::Pose& pose = poses[i];
        // 前哨站位置门，半径 <= 0 表示关闭
        if (outpost_radius_ > 0.0) {
            const double dx = pose.position.x - outpost_x_;
            const double dy = pose.position.y - outpost_y_;
            if (std::hypot(dx, dy) > outpost_radius_) {
                continue;
            }
        }

        const int plate = classifyPlate(pose.position.z);
        if (plate < 0) {
            continue;
        }
        const Eigen::Vector3d pos(
            pose.position.x, pose.position.y, pose.position.z);

        if (!ekf_inited_) {
            initEKF(plate, pos);
            ekf_inited_ = true;
            matched = true;
            continue;
        }
        if (updateOnePlate(plate, pos, now_sec)) {
            matched = true;
            // 能被检测到说明正对相机，用它的相位记住相机方向
            c_yaw_ref_ = plateAngle(plate);
        }
    }
    if (matched) {
        last_match_time_ = now_sec;
    }

    // 4. 状态机 + 发布
    updateStateMachine(now_sec, matched);
    if (ekf_inited_) {
        if (track_state_ == TS_TRACKING || track_state_ == TS_TEMP_LOST) {
            publishTarget(now_sec);
        } else {
            auto_aim_interfaces::msg::Target target_msg;
            target_msg.header.stamp = msg->header.stamp;
            target_msg.header.frame_id = target_frame_;
            target_msg.tracking = false;
            target_msg.id = "outpost";
            // armors_num 置 0 表示丢失，云台看到 0 才知道目标丢了
            target_msg.armors_num = 0;
            target_msg.type = "small";
            target_pub_->publish(target_msg);
        }
    }

    if (debug_) {
        publishMarkers();
    }
}

void OutpostTargetNode::timerTick() {
    // 只在跟踪期间需要定时输出，其他状态等检测消息来驱动
    if (!ekf_inited_) {
        return;
    }
    const double now_sec = this->now().seconds();
    // 不推进 EKF 状态，输出时用外推 yaw 算位置，避免定时器和观测抢状态
    updateStateMachine(now_sec, false);
    if (track_state_ == TS_TRACKING || track_state_ == TS_TEMP_LOST) {
        publishTarget(now_sec);
    }
}

bool OutpostTargetNode::transformToTarget(
    const geometry_msgs::msg::Pose& in, geometry_msgs::msg::Pose& out,
    const std::string& in_frame, const rclcpp::Time& stamp) {
    // 目标系和输入系相同时直接透传，离线调试用
    if (in_frame == target_frame_) {
        out = in;
        return true;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = in_frame;
    ps.header.stamp = stamp;
    ps.pose = in;
    try {
        out = tf2_buffer_.transform(ps, target_frame_).pose;
        return true;
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "tf transform failed: %s", error.what());
        return false;
    }
}

int OutpostTargetNode::classifyPlate(double z) const {
    int best = -1;
    double best_dist = z_tol_;
    for (int i = 0; i < 3; ++i) {
        const double dist = std::abs(z - slot_z_[i]);
        if (dist <= best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

void OutpostTargetNode::initEKF(int plate, const Eigen::Vector3d& pos) {
    // 能检测到说明这块板正对相机，相位为 0，yaw 和轴心直接反推，
    // 不然 yaw 猜错 120° 轴心就偏几十厘米，窗口间预测会飞
    const double r = r_initial_;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(ST_SIZE);
    x0(ST_YAW) = wrapAngle(-plate * kPlateStep);
    x0(ST_XC) = pos.x() - r * std::cos(0.0);
    x0(ST_YC) = pos.y() - r * std::sin(0.0);
    for (int j = 0; j < 3; ++j) {
        x0(ST_Z0 + j) = slot_z_[j];
    }
    x0(ST_Z0 + plate) = pos.z();

    // 位置和高度都是结构参数，初始协方差给紧一点
    Eigen::VectorXd p0(ST_SIZE);
    p0 << 1e-2, 1e-2, 0.4, 1e-4, 1e-4, 1e-4;
    ekf_.setState(x0, p0.asDiagonal());

    output_plate_ = plate;
    c_yaw_ref_ = 0.0;  // 被观测板正对，相位 0，相机方向就是 0
    // 方向没手动指定时，用第一个窗口的相位差分定方向
    phase_dot_sum_ = 0.0;
    phase_dot_count_ = 0;
    stop_frames_ = 0;
    RCLCPP_INFO(
        get_logger(), "init EKF with plate %d, z=%.2f, v_yaw=%.2f",
        plate, pos.z(), v_yaw_eff_);
}

bool OutpostTargetNode::updateOnePlate(
    int plate, const Eigen::Vector3d& pos, double now_sec) {
    // 门控：预测位置和观测差太远就拒绝，防止误检污染状态
    const Eigen::VectorXd x_pri = ekf_.prioriState();
    const double phi = x_pri(ST_YAW) + plate * kPlateStep;
    const Eigen::Vector3d pred(
        x_pri(ST_XC) + r_initial_ * std::cos(phi),
        x_pri(ST_YC) + r_initial_ * std::sin(phi),
        x_pri(ST_Z0 + plate));
    if ((pred - pos).norm() > gate_dist_) {
        return false;
    }

    // 观测模型和雅可比按板索引生成
    auto h = [this, plate](const Eigen::VectorXd& x) {
        const double p = x(ST_YAW) + plate * kPlateStep;
        Eigen::Vector3d z;
        z << x(ST_XC) + r_initial_ * std::cos(p),
             x(ST_YC) + r_initial_ * std::sin(p),
             x(ST_Z0 + plate);
        return z;
    };
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, ST_SIZE);
    {
        const double p = x_pri(ST_YAW) + plate * kPlateStep;
        H(0, ST_XC) = 1.0;
        H(0, ST_YAW) = -r_initial_ * std::sin(p);
        H(1, ST_YC) = 1.0;
        H(1, ST_YAW) = r_initial_ * std::cos(p);
        H(2, ST_Z0 + plate) = 1.0;
    }
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = r_meas_xy_ * r_meas_xy_;
    R(1, 1) = r_meas_xy_ * r_meas_xy_;
    R(2, 2) = r_meas_z_ * r_meas_z_;
    auto z_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        return a - b;
    };
    const Eigen::Vector3d z(pos.x(), pos.y(), pos.z());
    ekf_.update(z, H, R, h, z_subtract);

    // 相位差分：窗口内帧（dt 小）才有意义，用于定方向和停转检测
    const Eigen::VectorXd x_post = ekf_.posteriorState();
    const double phi_obs = std::atan2(
        pos.y() - x_post(ST_YC), pos.x() - x_post(ST_XC)) - plate * kPlateStep;
    if (has_last_obs_) {
        const double dphi = wrapAngle(phi_obs - last_obs_phase_);
        const double dt = now_sec - last_obs_time_;
        if (dt > 0.01 && dt < 0.1) {
            // 自动定方向：第一个窗口内累计相位变化
            if (rotate_direction_ == 0 && phase_dot_count_ < 15) {
                phase_dot_sum_ += dphi;
                phase_dot_count_++;
                if (phase_dot_count_ == 15) {
                    if (phase_dot_sum_ > 0.3) {
                        v_yaw_eff_ = 0.8 * M_PI;
                    } else if (phase_dot_sum_ < -0.3) {
                        v_yaw_eff_ = -0.8 * M_PI;
                    }
                    RCLCPP_INFO(
                        get_logger(), "auto direction: sum=%.2f v_yaw=%.2f",
                        phase_dot_sum_, v_yaw_eff_);
                }
            }
            // 停转检测：相位不再变化就认为模块停了
            if (std::abs(dphi) < 0.01) {
                stop_frames_++;
                if (stop_frames_ > 30) {
                    v_yaw_eff_ = 0.0;
                    RCLCPP_INFO(get_logger(), "module stopped, v_yaw=0");
                }
            } else {
                stop_frames_ = 0;
            }
        }
    }
    last_obs_phase_ = phi_obs;
    last_obs_time_ = now_sec;
    has_last_obs_ = true;
    return true;
}

void OutpostTargetNode::updateStateMachine(double now_sec, bool matched) {
    switch (track_state_) {
        case TS_LOST:
            if (matched) {
                track_state_ = TS_DETECTING;
                detect_count_ = 1;
                RCLCPP_INFO(get_logger(), "outpost: lost -> detecting");
            }
            break;
        case TS_DETECTING:
            if (matched) {
                if (++detect_count_ >= tracking_thres_) {
                    track_state_ = TS_TRACKING;
                    RCLCPP_INFO(get_logger(), "outpost: detecting -> tracking");
                }
            } else if (now_sec - last_match_time_ > temp_lost_time_) {
                track_state_ = TS_LOST;
                ekf_inited_ = false;
                RCLCPP_INFO(get_logger(), "outpost: detecting -> lost");
            }
            break;
        case TS_TRACKING:
            if (!matched && now_sec - last_match_time_ > temp_lost_time_) {
                track_state_ = TS_TEMP_LOST;
                RCLCPP_INFO(get_logger(), "outpost: tracking -> temp_lost");
            }
            break;
        case TS_TEMP_LOST:
            if (matched) {
                track_state_ = TS_TRACKING;
                RCLCPP_INFO(get_logger(), "outpost: temp_lost -> tracking");
            } else if (now_sec - last_match_time_ > lost_time_) {
                track_state_ = TS_LOST;
                ekf_inited_ = false;
                RCLCPP_INFO(get_logger(), "outpost: temp_lost -> lost, reset");
            }
            break;
    }
}

double OutpostTargetNode::plateAngle(int i) const {
    const Eigen::VectorXd x = ekf_.posteriorState();
    return x(ST_YAW) + i * kPlateStep;
}

Eigen::Vector3d OutpostTargetNode::platePosition(int i) const {
    const Eigen::VectorXd x = ekf_.posteriorState();
    const double phi = x(ST_YAW) + i * kPlateStep;
    return Eigen::Vector3d(
        x(ST_XC) + r_initial_ * std::cos(phi),
        x(ST_YC) + r_initial_ * std::sin(phi),
        x(ST_Z0 + i));
}

int OutpostTargetNode::chooseOutputPlate() const {
    // 各板相位离相机方向的夹角，谁最接近正对就输出谁
    int best = 0;
    double best_delta = 1e9;
    for (int j = 0; j < 3; ++j) {
        const double delta = std::abs(wrapAngle(plateAngle(j) - c_yaw_ref_));
        if (delta < best_delta) {
            best_delta = delta;
            best = j;
        }
    }
    // 当前输出板只要还在正对方向附近就保持，防止 60° 边界来回跳
    const double cur_delta = std::abs(wrapAngle(plateAngle(output_plate_) - c_yaw_ref_));
    if (cur_delta < kPlateStep / 2.0 + 0.25) {
        return output_plate_;
    }
    return best;
}

void OutpostTargetNode::publishTarget(double now_sec) {
    const int plate = chooseOutputPlate();
    output_plate_ = plate;
    const Eigen::VectorXd x = ekf_.posteriorState();
    // 输出时按当前时间外推 yaw，板转到哪就算到哪，不修改 EKF 状态
    const double yaw_now = wrapAngle(
        x(ST_YAW) + v_yaw_eff_ * (now_sec - last_predict_time_));
    const double phi = yaw_now + plate * kPlateStep;

    auto_aim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = rclcpp::Time(
        static_cast<int64_t>(now_sec), static_cast<int64_t>((now_sec - std::floor(now_sec)) * 1e9));
    target_msg.header.frame_id = target_frame_;
    target_msg.tracking = true;
    target_msg.id = "outpost";
    // 板中心直接当打击点，radius 为 0，云台不需要再做偏移
    target_msg.armors_num = 1;
    target_msg.type = "small";
    target_msg.position.x = x(ST_XC) + r_initial_ * std::cos(phi);
    target_msg.position.y = x(ST_YC) + r_initial_ * std::sin(phi);
    target_msg.position.z = x(ST_Z0 + plate);
    // 切向速度，云台用它做提前量
    const double v_t = v_yaw_eff_ * r_initial_;
    target_msg.velocity.x = -v_t * std::sin(phi);
    target_msg.velocity.y = v_t * std::cos(phi);
    target_msg.velocity.z = 0.0;
    target_pub_->publish(target_msg);
}

void OutpostTargetNode::publishMarkers() {
    visualization_msgs::msg::MarkerArray markers;

    // 轴心
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = this->now();
        marker.ns = "outpost_axis";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = 0.08;
        marker.scale.z = 1.4;
        marker.color.a = 0.6;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        if (ekf_inited_) {
            const Eigen::VectorXd x = ekf_.posteriorState();
            marker.pose.position.x = x(ST_XC);
            marker.pose.position.y = x(ST_YC);
            marker.pose.position.z = 0.0;
        } else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        markers.markers.push_back(marker);
    }

    // 三块板的预测位置，输出板高亮
    for (int i = 0; i < 3; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = this->now();
        marker.ns = "outpost_plates";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.6f;
        marker.color.g = 0.6f;
        marker.color.b = 0.6f;
        if (ekf_inited_) {
            const Eigen::Vector3d p = platePosition(i);
            marker.pose.position.x = p.x();
            marker.pose.position.y = p.y();
            marker.pose.position.z = p.z();
            if (i == output_plate_) {
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            }
        } else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
}

}  // namespace outpost_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(outpost_auto_aim::OutpostTargetNode)
