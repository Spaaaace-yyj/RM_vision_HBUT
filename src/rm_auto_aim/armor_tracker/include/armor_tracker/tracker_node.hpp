// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/debug_tracker.hpp"


namespace rm_auto_aim
{
class OutpostTracker;
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // 前哨站的固定频率传播与发布。检测回调只负责融合，时序和检测解耦，
  // 免得检测断一下框就停一下。
  void outpostTimerCallback();

  double computeQr(double vyaw);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;

  // 前哨站专用：固定频率的传播定时器和上一次传播的时刻
  rclcpp::TimerBase::SharedPtr outpost_timer_;
  rclcpp::Time last_propagate_time_;
  rclcpp::Time last_outpost_init_;   // 上次重新初始化的时刻，用来限频
  int marker_tick_ = 0;              // 标记降频用的计数
  double dt_;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_, s2q_vyaw_;
  double p_coord_, p_yaw_, p_r_;

  double r_xyz_factor, r_yaw, r_v_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // 前哨站专用跟踪器（常驻创建，运行中可切换；outpost_mode=true 时使用）
  bool outpost_mode_ = false;
  std::unique_ptr<OutpostTracker> outpost_tracker_;

  // 跟随仿真器的靶标开关：只勾一个时切换跟踪模式，两个都勾/都不勾则保持原模式。
  void applyTargetMode(bool armor_on, bool outpost_on);
  bool sim_armor_enabled_ = true;
  bool sim_outpost_enabled_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_armor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_outpost_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  //debug
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugTracker>::SharedPtr debug_tracker_pub_;

};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_