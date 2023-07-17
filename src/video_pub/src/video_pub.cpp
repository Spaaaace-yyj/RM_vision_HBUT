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
#include <camera_info_manager/camera_info_manager.hpp>

// opencv
#include <opencv2/opencv.hpp>

namespace video_pub
{
class VideoPub : public rclcpp::Node
{
public:
	VideoPub(const rclcpp::NodeOptions& options) : Node("video_pub", options)
	{
		RCLCPP_INFO(this->get_logger(), "Starting VideoPubNode!");

		// Create camera publisher
		// rqt_image_view can't subscribe image msg with sensor_data QoS
		// https://github.com/ros-visualization/rqt/issues/187
		bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
		auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
		camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

		// Load camera info
		camera_name_ = this->declare_parameter("camera_name", "mv_camera");
		camera_info_manager_ =
		std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
		auto camera_info_url = this->declare_parameter("camera_info_url", "package://video_pub/config/camera_info.yaml");
		if (camera_info_manager_->validateURL(camera_info_url)) {
			camera_info_manager_->loadCameraInfo(camera_info_url);
			camera_info_msg_ = camera_info_manager_->getCameraInfo();
		} else {
			RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
		}

		video_path_ = this->declare_parameter("video_path", "blue3.avi");
		auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
		cap_ = cv::VideoCapture(pkg_path + "/video/" + video_path_);
		int fps = this->declare_parameter("fps", 20);
		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)1000/fps), std::bind(&VideoPub::timer_callback, this));
	}

private:
	/**
	 * @brief 定时器回调函数
	 */
	void timer_callback()
	{
		cv::Mat frame;
		cap_ >> frame;
		if (frame.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "frame is empty");
			cap_.release();
			auto pkg_path = ament_index_cpp::get_package_share_directory("video_pub");
			cap_ = cv::VideoCapture(pkg_path + "/video/"  + video_path_);
			cap_ >> frame;
			// return;
		}
		cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
		auto image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
      	image_msg_->header.frame_id = "camera_optical_frame";
		image_msg_->header.stamp = this->now();
		camera_info_msg_.header = image_msg_->header;
		camera_pub_.publish(*image_msg_, camera_info_msg_);
	}
	std::string video_path_;
	rclcpp::TimerBase::SharedPtr timer_;

	// 虚拟相机
  	image_transport::CameraPublisher camera_pub_;
	std::string camera_name_;
	std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
	sensor_msgs::msg::CameraInfo camera_info_msg_;

	// opencv打开视频
	cv::VideoCapture cap_;
};
}  // namespace video_pub

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(video_pub::VideoPub)