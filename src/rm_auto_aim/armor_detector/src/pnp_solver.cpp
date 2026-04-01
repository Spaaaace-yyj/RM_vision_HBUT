// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()),
  tf_buffer_(tf_buffer)
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(Armor & armor, cv::Mat & rvec, cv::Mat & tvec, rclcpp::Time time)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  //装甲板四个灯条的角点
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
  bool success = cv::solvePnP(
      object_points,
      image_armor_points,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_IPPE);

  if(!success)
    return false;

  optimizeYaw(armor, object_points, image_armor_points, rvec, tvec, time);
  return true;
}

geometry_msgs::msg::PoseStamped PnPSolver::TransformToTargetFrame(geometry_msgs::msg::PoseStamped point, std::string target_frame)
{
  geometry_msgs::msg::PoseStamped target_point;

  try
  {
    target_point = tf_buffer_->transform(point, target_frame);
  }
  catch (tf2::TransformException &ex)
  {
    std::cerr << "[TF ERROR] Transform failed: " << ex.what() << std::endl;
    target_point = point;
  }

  return target_point;
}

void PnPSolver::updateTransform(const rclcpp::Time& time)
{
    geometry_msgs::msg::TransformStamped tf;

    try
    {
        tf = tf_buffer_->lookupTransform(
            "odom",
            "camera_optical_frame",
            time);
    }
    catch (tf2::TransformException &ex)
    {
        std::cerr << ex.what() << std::endl;
        return;
    }

    // translation
    t_camera2world_ = Eigen::Vector3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

    // rotation
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);

    tf2::Matrix3x3 R(q);

    R_camera2world_ <<
        R[0][0], R[0][1], R[0][2],
        R[1][0], R[1][1], R[1][2],
        R[2][0], R[2][1], R[2][2];

    // inverse
    R_world2camera_ = R_camera2world_.transpose();
    t_world2camera_ = -R_world2camera_ * t_camera2world_;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::computeYawError(double yaw,
  const Eigen::Vector3d& t_world,
  const std::vector<cv::Point3f>& object_points,
  const std::vector<cv::Point2f>& image_points)
{
    double pitch = 15.0 * M_PI / 180.0;

    double cy = cos(yaw), sy = sin(yaw);
    double cp = cos(pitch), sp = sin(pitch);

    Eigen::Matrix3d R_aw;
    R_aw <<
        cy*cp, -sy, cy*sp,
        sy*cp,  cy, sy*sp,
        -sp,     0,  cp;

    Eigen::Matrix3d R_ac = R_world2camera_ * R_aw;
    Eigen::Vector3d t_ac = R_world2camera_ * t_world + t_world2camera_;

    cv::Mat R_cv;
    cv::eigen2cv(R_ac, R_cv);

    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);

    cv::Mat tvec = (cv::Mat_<double>(3,1) <<
        t_ac(0), t_ac(1), t_ac(2));

    return reprojectionError(object_points, image_points, rvec, tvec);
}

//yaw优化
void PnPSolver::optimizeYaw(
    Armor & armor,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    cv::Mat& rvec,
    cv::Mat& tvec,
    rclcpp::Time time)
{
    //更新tf
    updateTransform(time);

    //tvec to Eigen
    Eigen::Vector3d t_cam(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

    // camera to world
    Eigen::Vector3d t_world = R_camera2world_ * t_cam + t_camera2world_;

    // 初始yaw
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    Eigen::Matrix3d R_cam;
    cv::cv2eigen(R_cv, R_cam);

    Eigen::Matrix3d R_world = R_camera2world_ * R_cam;

    double yaw_init = std::atan2(R_world(1,0), R_world(0,0));

    //debug
    armor.yaw_raw = yaw_init;
    //三分法搜索
    constexpr double SEARCH_RANGE = 140.0 * M_PI / 180.0;
    constexpr int STEPS = 20;
    double left = yaw_init - SEARCH_RANGE / 2;
    double right = yaw_init + SEARCH_RANGE / 2;

    for (int i = 0; i < STEPS; i++)
    {
        double m1 = left + (right - left) / 3.0f;
        double m2 = right - (right - left) / 3.0f;

        double error1 = computeYawError(m1, t_world, object_points, image_points);
        double error2 = computeYawError(m2, t_world, object_points, image_points);

        if (error1 < error2)
        {
            right = m2;
        }else
        {
            left = m1;
        }
    }

    double best_yaw = (left + right) / 2.0;

    //用最优 yaw 重建
    double pitch = 15.0 * M_PI / 180.0;
    double cy = cos(best_yaw), sy = sin(best_yaw);
    double cp = cos(pitch), sp = sin(pitch);

    Eigen::Matrix3d R_aw;
    R_aw <<
        cy*cp, -sy, cy*sp,
        sy*cp,  cy, sy*sp,
        -sp,     0,  cp;

    Eigen::Matrix3d R_ac = R_world2camera_ * R_aw;
    Eigen::Vector3d t_ac = R_world2camera_ * t_world + t_world2camera_;

    cv::Mat R_cv_final;
    cv::eigen2cv(R_ac, R_cv_final);

    cv::Rodrigues(R_cv_final, rvec);

    tvec.at<double>(0) = t_ac(0);
    tvec.at<double>(1) = t_ac(1);
    tvec.at<double>(2) = t_ac(2);
    armor.best_yaw = best_yaw;
    // RCLCPP_INFO(rclcpp::Node("solver").get_logger(), "init_yaw = %f, best_yaw = %f", yaw_init, best_yaw);
}

  double PnPSolver::reprojectionError(
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& image_points,
      const cv::Mat& rvec,
      const cv::Mat& tvec)
{
  std::vector<cv::Point2f> projected_points;

  cv::projectPoints(
      object_points,
      rvec,
      tvec,
      camera_matrix_,
      dist_coeffs_,
      projected_points
  );

  double error = 0.0;
  for (int i = 0; i < 4; i++)
  {
    error += cv::norm(projected_points[i] - image_points[i]);
  }

  return error;
}

}  // namespace rm_auto_aim
