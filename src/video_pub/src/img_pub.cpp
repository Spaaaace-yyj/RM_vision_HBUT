/**
 * @file video_pub.cpp
 * @brief opencv读取视频，发布图像 rclcpp
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-01-18
 *
 */
// c++
#include <chrono>
#include <functional>

// ros
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// opencv
#include <opencv2/opencv.hpp>

namespace video_pub
{
class ImgPub : public rclcpp::Node
{
public:
	ImgPub(const rclcpp::NodeOptions& options) : Node("img_pub", options)
	{
		RCLCPP_INFO(this->get_logger(), "Starting VideoPubNode!");
		src_img_pub_ = image_transport::create_publisher(this, "image_raw");
		auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
		src_img_ = cv::imread(pkg_path + "/video/blue_1.png");
		cv::cvtColor(src_img_, src_img_, cv::COLOR_BGR2RGB);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&ImgPub
	::timer_callback, this));
	}

private:
	/**
	 * @brief 定时器回调函数
	 */
	void timer_callback()
	{
		if (src_img_.empty())
		{
		RCLCPP_ERROR(this->get_logger(), "src_img_ is empty");
		auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
		src_img_ = cv::imread(pkg_path + "/video/blue_1.png");
		RCLCPP_INFO(this->get_logger(), (pkg_path + "/video/blue_1.png").c_str());
		// return;
		}
		auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", src_img_).toImageMsg();
		msg->header.stamp = this->now();
		src_img_pub_.publish(*msg);
		RCLCPP_INFO(this->get_logger(), "aleady publish a src_img_");
	}
	image_transport::Publisher src_img_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	// opencv打开图片
	cv::Mat src_img_;
};
}  // namespace video_pub

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(video_pub::ImgPub)