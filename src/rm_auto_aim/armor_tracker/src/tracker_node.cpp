// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"
#include "armor_tracker/outpost_tracker.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // 前哨站专用跟踪器（三块不同高度装甲板绕同一竖直轴旋转）
  // 常驻创建：运行中可跟随仿真器靶标开关切换，无需重启节点。
  outpost_mode_ = this->declare_parameter("outpost_mode", false);
  outpost_tracker_ = std::make_unique<OutpostTracker>(
    max_match_distance, max_match_yaw_diff,
    tracker_->tracking_thres, static_cast<int>(lost_time_thres_ * 25));//让状态掉慢一点 从10改成25 只是先试试～
  if (outpost_mode_) {
    RCLCPP_INFO(this->get_logger(), "前哨站跟踪模式已启用（outpost_mode=true）");
  }

  // ===== 跟随仿真器靶标开关（与 detector 同一套规则）=====
  sim_armor_enabled_ = !outpost_mode_;
  sim_outpost_enabled_ = outpost_mode_;
  {
    const auto sim_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    sim_armor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sim/armor_enabled", sim_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        sim_armor_enabled_ = msg->data;
        applyTargetMode(sim_armor_enabled_, sim_outpost_enabled_);
      });
    sim_outpost_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sim/outpost_enabled", sim_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        sim_outpost_enabled_ = msg->data;
        applyTargetMode(sim_armor_enabled_, sim_outpost_enabled_);
      });
  }
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() != "outpost_mode") continue;
        const bool want = p.as_bool();
        if (want == outpost_mode_) continue;
        outpost_mode_ = want;
        RCLCPP_INFO(this->get_logger(), "跟踪模式手动切换为 %s",
                    want ? "OUTPOST" : "ARMOR");
      }
      return result;
    });

  // EKF
  // 11D whole-robot model
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r, l, h
  //   l = r2 - r1, h = z2 - z1
  // measurement is generated per observed armor in Tracker:
  //   z = [ypd_yaw, ypd_pitch, ypd_distance, armor_yaw]
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) = limitRad(x(6) + x(7) * dt_);
    return x_new;
  };

  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(11, 11);
    // clang-format off
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_, 0,   0,   0,   0,   0,   0,   0,
          0,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };

  // h / J_h are dummies. 11D EKF 的观测函数依赖装甲板 id，实际在 Tracker::updateOneArmor() 中传入。
  auto h = [](const Eigen::VectorXd &) {
    return Eigen::VectorXd::Zero(4);
  };
  auto j_h = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Zero(4, 11);
  };

  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 100.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 400.0);

  // 前哨站跟踪器同样使用 CV 过程噪声（之前漏了这步，
  // 导致圆心/yaw 估计抖动极大、云台按错误 yaw 追板越追越偏）
  if (outpost_tracker_) {
    outpost_tracker_->setProcessNoise(
      s2qxyz_, s2qyaw_, 0.02,
      declare_parameter("outpost_v_yaw", 1.0));
    // 前哨站专用更新门限。四板用的是 tracker.max_match_distance，是另一个参数，
    // 这里单独起一个名字，保证调前哨站不会连带改到四板。
    outpost_tracker_->setPositionGate(
      declare_parameter("outpost_match_distance", 0.1));
    // 检测态的入场门限故意放松，先让跟踪锁上，之后再用上面那个窄门限守住
    outpost_tracker_->setAcquireGate(
      declare_parameter("outpost_acquire_distance", 1.0));
    // 强制转速需要板朝向可信，我们那边朝向基本是噪声，默认先关着
    outpost_tracker_->setFixSpin(declare_parameter("outpost_fix_spin", false));
    // 转向符号。零表示自动判定，正负表示手动指定，实际场上方向两向都可能。
    outpost_tracker_->setSpinSign(declare_parameter("outpost_spin_sign", 1.0));
  }
  auto u_q = [this]() {
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(11, 11);
    const double t = dt_;
    const double v1 = s2qxyz_;
    const double v2 = s2qyaw_;

    const double a = std::pow(t, 4) / 4.0;
    const double b = std::pow(t, 3) / 2.0;
    const double c = std::pow(t, 2);

    auto fill_cv_noise = [&](int pos, int vel, double var) {
      q(pos, pos) = a * var;
      q(pos, vel) = b * var;
      q(vel, pos) = b * var;
      q(vel, vel) = c * var;
    };

    fill_cv_noise(0, 1, v1);  // x / vx
    fill_cv_noise(2, 3, v1);  // y / vy
    fill_cv_noise(4, 5, v1);  // z / vz
    fill_cv_noise(6, 7, v2);  // yaw / v_yaw

    // r, l, h 复刻开源：过程噪声为 0，主要靠观测修正。
    return q;
  };

  // update_R is dummy. 11D EKF 使用 Tracker 内部的 ypda 动态 R。
  auto u_r = [](const Eigen::VectorXd &) {
    return Eigen::MatrixXd::Identity(4, 4);
  };

  // P - error estimate covariance matrix. 真正初始化时 Tracker::initEKF() 会按目标类型重置 P。
  Eigen::DiagonalMatrix<double, 11> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  debug_tracker_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugTracker>(
    "/debug/tracker", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03; //厚度
  armor_marker_.scale.z = 0.125;  //0.125
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  // 前哨站的传播和发布走固定频率，不挂在检测回调上。
  // 检测输出间隔实测中位三十八毫秒、尾部五百多毫秒，跟着它走的话框会停一下再猛冲。
  // 这里按一百赫兹，用仿真时间算步长，仿真慢于实时也不会偏。
  outpost_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), [this]() { outpostTimerCallback(); });
}

// 跟随仿真器靶标开关：只有"恰好勾选一个"时才切换模式。
// 两个都勾（场景里同时有四板和前哨站）或都不勾时保持原模式。
void ArmorTrackerNode::applyTargetMode(bool armor_on, bool outpost_on)
{
  if (armor_on == outpost_on) {
    if (armor_on) {
      RCLCPP_WARN(this->get_logger(),
                  "ARMOR 与 OUTPOST 同时启用，跟踪模式保持不变（%s）",
                  outpost_mode_ ? "OUTPOST" : "ARMOR");
    }
    return;
  }
  const bool want_outpost = outpost_on;
  if (want_outpost == outpost_mode_) return;
  outpost_mode_ = want_outpost;
  // 同步刷新参数存储，这样 ros2 param get 看到的就是真实生效的模式
  this->set_parameter(rclcpp::Parameter("outpost_mode", want_outpost));
  RCLCPP_INFO(this->get_logger(), "跟踪模式已切换为 %s（跟随仿真器靶标开关）",
              want_outpost ? "OUTPOST" : "ARMOR");
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate

  //debug
  auto_aim_interfaces::msg::DebugTracker debug_msg;

  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  //排除Z轴过高的装甲板
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // ===== 前哨站路径：使用专用 OutpostTracker（三块不同高度板）=====
  if (outpost_mode_ && outpost_tracker_) {
    // 这里只做融合，发布交给固定频率的定时器。检测断帧的时候框照样按预测走。
    if (outpost_tracker_->tracker_state == Tracker::LOST) {
      // 重新初始化限频。原来每帧都调，检测抖一下就换一个观测重开，
      // 圆心和 yaw 跟着被反复重置，画面上就是跳变和方向反转。
      // 两百毫秒一次，既跟得上目标，又不会拿单帧噪声当基准。
      const rclcpp::Time now_time = this->now();
      if (last_outpost_init_.nanoseconds() == 0 ||
        (now_time - last_outpost_init_).seconds() > 0.2)
      {
        outpost_tracker_->init(armors_msg);
        last_outpost_init_ = now_time;
      }
    } else {
      outpost_tracker_->update(armors_msg);
    }
    last_time_ = time;
    return;
  }

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    //自适应半径过程噪声协方差
    // s2qr_ = computeQr(tracker_->target_state(7));

    tracker_->update(armors_msg);
    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = (tracker_->info_yaw_diff) * (180 / M_PI);
    info_msg.position.x = tracker_->tracked_armor.pose.position.x;
    info_msg.position.y = tracker_->tracked_armor.pose.position.y;
    info_msg.position.z = tracker_->tracked_armor.pose.position.z;
    info_msg.yaw = tracker_->measurement.size() >= 4 ? (tracker_->measurement(3)) * (180 / M_PI) : 0.0;
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = state(8) + state(9);
      target_msg.dz = state(10);
      // 四板是两大小交替，高度只有低高两组，按板序号摊开给成四个
      target_msg.dz_list = {0.0, target_msg.dz, 0.0, target_msg.dz};
      target_msg.type = tracker_->tracked_armor.type;
    }
  }

  last_time_ = time;

  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorTrackerNode::outpostTimerCallback()
{
  if (!outpost_mode_ || !outpost_tracker_) {
    return;
  }

  const rclcpp::Time now = this->now();
  double dt = 0.01;
  if (last_propagate_time_.nanoseconds() > 0) {
    dt = (now - last_propagate_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.01;  // 时间跳变或者刚启动，用一个保守值
    }
  }
  last_propagate_time_ = now;

  outpost_tracker_->propagate(dt);

  auto_aim_interfaces::msg::Target target_msg;
  target_msg.header.stamp = now;
  target_msg.header.frame_id = target_frame_;
  outpost_tracker_->fillTargetMsg(target_msg);
  if (outpost_tracker_->tracker_state == Tracker::LOST) {
    target_msg.tracking = false;  // 没锁定就给云台一个停火信号
  } else {
    target_msg.type = outpost_tracker_->tracked_armor.type;
  }

  target_pub_->publish(target_msg);
  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);


    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    //宽度
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23; //0.135
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
      }
      // 高度优先用逐板偏移。四板给的是两组交替，前哨站给的是三块各不同，
      // 老的单标量 dz 只够四板用，前哨站三块会被画成同一高度。
      if (i < target_msg.dz_list.size()) {
        p_a.z = za + target_msg.dz_list[i];
      } else {
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

double ArmorTrackerNode::computeQr(double vyaw)
{
  double w = std::abs(vyaw);

  const double w1 = 0.5;
  const double w2 = 3.0;

  const double Qmin = 0.1;
  const double Qmax = 100.0;

  if (w <= w1) return Qmin;
  if (w >= w2) return Qmax;

  double t = (w - w1) / (w2 - w1);
  return Qmin + t * (Qmax - Qmin);
}


}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)