//
// Created by spaaaaace on 2026/2/5.
//

#ifndef BUILD_ARUCO_DETECTOR_H
#define BUILD_ARUCO_DETECTOR_H

//ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>

//opencv
#include <opencv4/opencv2/opencv.hpp>
// #include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/aruco.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
//c++

//user
#include "auto_aim_interfaces/msg/armors.hpp"
#include "aruco.h"

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector();

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
private:
    bool debug_msg = true;

    cv::Mat image_raw_;

    cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                            505.454279, 0, 649.436297,
                            0, 502.363645, 357.912267,
                            0, 0, 1);
    cv::Mat dist_coeffs_ = (cv::Mat_<double>(5, 1) <<
                0.007606, 0.001375, -0.001245, -0.003013, 0.000000);

    std::vector<cv::Point3f> aruco_object_;

private:
    //sub
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    //pub
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arrau_pub_;
};

#endif //BUILD_ARUCO_DETECTOR_H