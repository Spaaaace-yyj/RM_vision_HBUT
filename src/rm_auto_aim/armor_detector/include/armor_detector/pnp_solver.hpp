// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

//Eigen
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//geometry_msgs::msg::PoseStamped

namespace rm_auto_aim
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  // Get 3d position
  bool solvePnP(Armor & armor, cv::Mat & rvec, cv::Mat & tvec, rclcpp::Time time);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

  //重投影误差
  double reprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec);
  //YAW优化
  void optimizeYaw(
    Armor & armor,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    cv::Mat& rvec,
    cv::Mat& tvec,
    rclcpp::Time time);

  double computeYawError(double yaw,
    const Eigen::Vector3d& t_world,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points);

  double computeDepthError(
    double depth,
    double yaw,
    const Eigen::Vector3d& dir_cam,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points);

  void updateTransform(const rclcpp::Time& time);

  geometry_msgs::msg::PoseStamped TransformToTargetFrame(geometry_msgs::msg::PoseStamped point, std::string target_frame);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  Eigen::Matrix3d R_camera2world_;
  Eigen::Vector3d t_camera2world_;

  Eigen::Matrix3d R_world2camera_;
  Eigen::Vector3d t_world2camera_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
