/**
 * @file video_sub.cpp
 * @brief 接收图像节点
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-01-18
 * 
 */
// c++
#include <functional>
#include <memory>
#include <string>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// opencv
#include <opencv2/opencv.hpp>

namespace video_pub
{
class VideoSub : public rclcpp::Node
{
public:
	VideoSub(const rclcpp::NodeOptions & options) : Node("video_sub", options)
	{
		RCLCPP_INFO(this->get_logger(), "Starting VideoSubNode!");
  		std::string transport_ = "raw";
		img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
			this, "/image_raw", std::bind(&VideoSub::sub_callback, this, std::placeholders::_1), transport_,
			rmw_qos_profile_sensor_data));

		// sub_ = this->create_subscription<sensor_msgs::msg::Image>("src_img", 10, std::bind(&VideoSub::sub_callback, this, std::placeholders::_1));
	}

private:
	/**
	 * @brief 订阅回调函数
	 */
	void sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
	{
		cv::Mat frame = cv_bridge::toCvShare(img_msg, "rgb8")->image;
		cv::imshow("video", frame);
		cv::waitKey(1);
	}
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
	std::shared_ptr<image_transport::Subscriber> img_sub_;
};
} // namespace video_pub

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(video_pub::VideoSub)
