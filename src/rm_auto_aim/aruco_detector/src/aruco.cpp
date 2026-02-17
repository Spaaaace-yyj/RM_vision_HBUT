//
// Created by spaaaaace on 2026/2/6.
//

#include "../include/aruco_detector/aruco.h"

Aruco::Aruco(int id, const std::vector<cv::Point2f> image_point, const std::vector<cv::Point3f> world_def)
{
    this->id_ = id;
    this->image_point_ = image_point;
    this->world_def_ = world_def;
}

void Aruco::setArucoInfo(int id, const std::vector<cv::Point2f> image_point, const std::vector<cv::Point3f> world_def)
{
    this->id_ = id;
    this->image_point_ = image_point;
    this->world_def_ = world_def;
}

bool Aruco::solveArucoPnP(const cv::Mat camera_matrix, const cv::Mat dist_coeffs)
{
    if (image_point_.empty() || world_def_.empty())
    {
        return false;
    }
    bool can_solve = cv::solvePnP(world_def_, image_point_, camera_matrix, dist_coeffs, rvec_cv, tvec_cv, false, cv::SOLVEPNP_IPPE_SQUARE);

    if (can_solve)
    {
        //相机坐标系变换到Ros2的坐标系定义

        tvec_ros[0] = tvec_cv[2];
        tvec_ros[1] = -tvec_cv[0];
        tvec_ros[2] = -tvec_cv[1];
        world_location_ = cv::Point3f(tvec_ros[0], tvec_ros[1], tvec_ros[2]);

        cv::Mat R_cv;
        cv::Rodrigues(rvec_cv, R_cv);

        const tf2::Matrix3x3 tf_cv(
            R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
            R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
            R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
        );
        tf2::Quaternion q_cv;
        tf_cv.getRotation(q_cv);

        aruco_pos_cv_.position.x = world_location_.x;
        aruco_pos_cv_.position.y = world_location_.y;
        aruco_pos_cv_.position.z = world_location_.z;

        aruco_pos_cv_.orientation.x = q_cv.x();
        aruco_pos_cv_.orientation.y = q_cv.y();
        aruco_pos_cv_.orientation.z = q_cv.z();
        aruco_pos_cv_.orientation.w = q_cv.w();

        const cv::Mat T = (cv::Mat_<double>(3,3) <<
            0, 0, 1,
            -1, 0, 0,
            0,-1, 0);
        cv::Mat R_ros = T * R_cv;
        cv::Rodrigues(R_ros, rvec_ros);

        const tf2::Matrix3x3 tf_rot(
            R_ros.at<double>(0,0), R_ros.at<double>(0,1), R_ros.at<double>(0,2),
            R_ros.at<double>(1,0), R_ros.at<double>(1,1), R_ros.at<double>(1,2),
            R_ros.at<double>(2,0), R_ros.at<double>(2,1), R_ros.at<double>(2,2)
        );

        tf2::Quaternion q_ros;
        tf_rot.getRotation(q_ros);

        aruco_pos_ros_.position.x = world_location_.x;
        aruco_pos_ros_.position.y = world_location_.y;
        aruco_pos_ros_.position.z = world_location_.z;

        aruco_pos_ros_.orientation.x = q_ros.x();
        aruco_pos_ros_.orientation.y = q_ros.y();
        aruco_pos_ros_.orientation.z = q_ros.z();
        aruco_pos_ros_.orientation.w = q_ros.w();
    }

    return can_solve;
}
