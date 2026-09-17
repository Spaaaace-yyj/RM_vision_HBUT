#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

class ImageLatencyTestNode : public rclcpp::Node
{
public:
  ImageLatencyTestNode()
  : Node("image_latency_test_node")
  {
    auto qos = rmw_qos_profile_sensor_data;
    qos.depth = 1;

    sub_ = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
        this,
        "/image_raw",
        std::bind(
          &ImageLatencyTestNode::imageCallback,
          this,
          std::placeholders::_1),
        "raw",
        qos));

    last_report_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Subscribed to /image_raw with sensor-data QoS and depth=1");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const rclcpp::Time now = this->now();
    const rclcpp::Time image_stamp(msg->header.stamp);

    const double input_age_ms =
      (now - image_stamp).seconds() * 1000.0;

    if (frame_count_ == 0) {
      min_age_ms_ = input_age_ms;
      max_age_ms_ = input_age_ms;
    } else {
      min_age_ms_ = std::min(min_age_ms_, input_age_ms);
      max_age_ms_ = std::max(max_age_ms_, input_age_ms);
    }

    total_age_ms_ += input_age_ms;
    ++frame_count_;

    if (frame_count_ >= 100) {
      const auto now_steady = std::chrono::steady_clock::now();

      const double elapsed_s =
        std::chrono::duration<double>(
          now_steady - last_report_time_).count();

      const double fps =
        elapsed_s > 0.0
          ? static_cast<double>(frame_count_) / elapsed_s
          : 0.0;

      const double avg_age_ms =
        total_age_ms_ / static_cast<double>(frame_count_);

      RCLCPP_INFO(
        get_logger(),
        "Input age: current=%.2f ms | avg=%.2f ms | "
        "min=%.2f ms | max=%.2f ms | receive FPS=%.1f",
        input_age_ms,
        avg_age_ms,
        min_age_ms_,
        max_age_ms_,
        fps);

      frame_count_ = 0;
      total_age_ms_ = 0.0;
      min_age_ms_ = 0.0;
      max_age_ms_ = 0.0;
      last_report_time_ = now_steady;
    }
  }

  std::shared_ptr<image_transport::Subscriber> sub_;

  std::size_t frame_count_{0};
  double total_age_ms_{0.0};
  double min_age_ms_{0.0};
  double max_age_ms_{0.0};

  std::chrono::steady_clock::time_point last_report_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageLatencyTestNode>());
  rclcpp::shutdown();
  return 0;
}
