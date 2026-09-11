// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/neural_detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  // 神经网络模式相关（detector_mode = neural 时才会用到）
  void initNeuralParams();
  bool setDetectorMode(const std::string & mode);
  bool ensureNeuralDetector();
  std::vector<Armor> detectArmorsByNeural(const cv::Mat & img, bool & traditional_ran);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();

  //tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Armor Detector（传统视觉模式，保持原有实现不动）
  std::unique_ptr<Detector> detector_;

  // 检测模式：traditional=传统识别（默认，行为与以前完全一致），neural=神经网络模式。
  // 只用一个原子标志控制，图像回调里读，参数回调里写，避免两个线程抢对象。
  std::atomic<bool> neural_mode_{false};
  std::string detector_mode_str_ = "traditional";
  std::shared_ptr<rclcpp::ParameterCallbackHandle> mode_cb_handle_;
  // 神经网络检测器，切到 neural 模式后第一次用到时才加载模型
  std::unique_ptr<NeuralDetector> neural_detector_;
  std::atomic<bool> neural_load_failed_{false};
  NeuralDetectorParams neural_params_;
  bool neural_refine_ = true;      // 用传统灯条微调 CNN 角点
  bool neural_fallback_ = true;    // CNN 没检出时退回传统流程
  std::string neural_model_path_;
  // 本帧是否跑过传统流程（决定 debug 里的二值图/灯条信息是不是这一帧的）
  bool traditional_ran_ = false;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  std::shared_ptr<image_transport::Subscriber> img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  // 录制视频
  bool is_record_;
  cv::VideoWriter video_writer_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
