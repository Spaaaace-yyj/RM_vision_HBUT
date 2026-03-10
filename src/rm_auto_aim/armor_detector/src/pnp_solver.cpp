// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
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

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
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

  // optimizeYaw(object_points, image_armor_points, rvec, tvec);
  return true;

}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

  double PnPSolver::reprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec)
{
  std::vector<cv::Point2f> projected;

  cv::projectPoints(
      object_points,
      rvec,
      tvec,
      camera_matrix_,
      dist_coeffs_,
      projected);

  double error = 0;

  for(int i=0;i<4;i++)
  {
    error += cv::norm(projected[i] - image_points[i]);
  }

  return error;
}

  void PnPSolver::optimizeYaw(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    cv::Mat& rvec,
    cv::Mat& tvec)
{
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  double yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
  double pitch = atan2(-R.at<double>(2,0),
                  sqrt(R.at<double>(2,1)*R.at<double>(2,1) +
                       R.at<double>(2,2)*R.at<double>(2,2)));
  // double pitch = -15.0 * CV_PI / 180.0;
  double roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
  // double roll = -15.0 * CV_PI / 180.0;

  double best_yaw = yaw;
  double min_error = 1e9;

  const double search_range = 20.0 * CV_PI / 180.0;
  const double step = 1.0 * CV_PI / 180.0;

  for(double dy = -search_range; dy <= search_range; dy += step)
  {
    double test_yaw = yaw + dy;

    double cy = cos(test_yaw);
    double sy = sin(test_yaw);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cr = cos(roll);
    double sr = sin(roll);

    cv::Mat R_test = (cv::Mat_<double>(3,3) <<
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
        -sp,   cp*sr,            cp*cr
    );

    cv::Mat rvec_test;
    cv::Rodrigues(R_test, rvec_test);

    double error = reprojectionError(
        object_points,
        image_points,
        rvec_test,
        tvec);

    if(error < min_error)
    {
      min_error = error;
      best_yaw = test_yaw;
    }
  }
  // std::cout << "raw_yaw:" <<  yaw << "| now_yaw:" << best_yaw << "| distance:" << cv::norm(tvec) << std::endl;

  // 更新 rvec
  double cy = cos(best_yaw);
  double sy = sin(best_yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  cv::Mat R_best = (cv::Mat_<double>(3,3) <<
      cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
      -sp,   cp*sr,            cp*cr
  );

  cv::Rodrigues(R_best, rvec);
}

}  // namespace rm_auto_aim
