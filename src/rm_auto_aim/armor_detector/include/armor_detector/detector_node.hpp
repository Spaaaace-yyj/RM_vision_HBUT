// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
  ~ArmorDetectorNode() override;

private:
  // ----------------------------- Frame scheduling -----------------------------
  // ROS 回调只负责把最新图像放进单槽位 mailbox，不执行任何检测算法。
  // 当 worker 还在处理上一帧时，新帧会覆盖旧的未处理帧，避免积压旧图像。
  struct FramePacket
  {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    std::chrono::steady_clock::time_point received_steady;
    float rx_age_ms = 0.0F;
  };

  struct FrameTiming
  {
    // ROS message timing
    float rx_age_ms = 0.0F;        // header.stamp -> imageCallback() 到达
    float mailbox_wait_ms = 0.0F;  // imageCallback() -> worker 真正取走本帧
    float work_age_ms = 0.0F;      // header.stamp -> worker 开始处理

    // Current-process timing
    float cv_bridge_ms = 0.0F;
    float record_ms = 0.0F;
    float detect_ms = 0.0F;
    float pnp_ms = 0.0F;
    float publish_ms = 0.0F;
    float core_ms = 0.0F;          // worker 开始 -> 检测结果发布完成，不含 debug 绘制
    float e2e_ms = 0.0F;           // header.stamp -> 检测结果发布完成

    std::size_t pnp_count = 0;
    bool neural_requested = false;
    bool neural_succeeded = false;
    bool traditional_fallback = false;
  };

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void processingLoop();
  void processFrame(const FramePacket & frame);

  // ----------------------------- Detection -----------------------------
  std::unique_ptr<Detector> initDetector();
  void initNeuralParams();
  bool setDetectorMode(const std::string & mode);
  bool ensureNeuralDetector();

  std::vector<Armor> detectArmors(
    const cv::Mat & img, bool use_neural, FrameTiming & timing);
  std::vector<Armor> detectArmorsTraditional(const cv::Mat & img);
  std::vector<Armor> detectArmorsNeural(
    const cv::Mat & img, FrameTiming & timing);

  // ----------------------------- PnP / publish -----------------------------
  void solvePnPAndPublish(
    std::vector<Armor> & armors,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    FrameTiming & timing,
    const std::shared_ptr<PnPSolver> & pnp_solver);
  void publishMarkers();

  // ----------------------------- Debug / performance -----------------------------
  void createDebugPublishers();
  void publishTraditionalDebugImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const std::vector<Armor> & armors,
    const FrameTiming & timing,
    const cv::Point2f & cam_center);
  void publishNeuralDebugImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const std::vector<Armor> & armors,
    const FrameTiming & timing,
    const cv::Point2f & cam_center);
  void drawPerformanceOverlay(
    cv::Mat & img,
    const FrameTiming & timing,
    const NeuralDetector::Timing * neural_timing) const;
  void logPerformance(const FrameTiming & timing);

  // ----------------------------- TF / camera -----------------------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex camera_state_mutex_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::shared_ptr<PnPSolver> pnp_solver_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  // ----------------------------- Detectors -----------------------------
  // 三个算法对象只在 processing_thread_ 内访问，因此算法内部不需要额外加锁。
  std::unique_ptr<Detector> detector_;

  std::atomic<bool> neural_mode_{false};
  std::string detector_mode_str_ = "traditional";
  std::shared_ptr<rclcpp::ParameterCallbackHandle> mode_cb_handle_;

  std::unique_ptr<NeuralDetector> neural_detector_;
  std::atomic<bool> neural_load_failed_{false};
  NeuralDetectorParams neural_params_;
  std::string neural_model_path_;

  // ----------------------------- Latest-frame mailbox -----------------------------
  std::shared_ptr<image_transport::Subscriber> img_sub_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  FramePacket latest_frame_;
  bool latest_frame_available_ = false;
  std::atomic<bool> worker_running_{true};
  std::thread processing_thread_;

  std::atomic<std::uint64_t> received_frames_{0};
  std::atomic<std::uint64_t> overwritten_frames_{0};
  std::uint64_t processed_frames_ = 0;  // worker 单线程访问
  float processing_fps_ = 0.0F;
  std::chrono::steady_clock::time_point fps_window_start_;
  std::uint64_t fps_window_frames_ = 0;

  // ----------------------------- Publishers -----------------------------
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Debug publishers 始终创建，debug 参数只控制是否实际构图/发布，避免运行时 reset 与 worker 竞争。
  std::atomic<bool> debug_{false};
  std::atomic<bool> performance_log_{true};
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> perf_log_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  // Video recording
  bool is_record_ = false;
  cv::VideoWriter video_writer_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
