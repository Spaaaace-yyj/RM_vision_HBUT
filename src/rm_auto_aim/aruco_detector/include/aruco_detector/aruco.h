//
// Created by spaaaaace on 2026/2/6.
//

#ifndef BUILD_ARUCO_H
#define BUILD_ARUCO_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>

class Aruco
{
public:
    Aruco(int id, const std::vector<cv::Point2f> image_point, const std::vector<cv::Point3f> world_def);
    Aruco() = default;

    void setArucoInfo(int id, const std::vector<cv::Point2f> image_point, const std::vector<cv::Point3f> world_def);

    bool solveArucoPnP(
        const cv::Mat camera_matrix,
        const cv::Mat dist_coeffs);
public:
    int id_ = 0;
    std::vector<cv::Point2f> image_point_;  //Aruco在图像坐标系下的坐标
    cv::Point3f world_location_;   //Aruco在世界坐标系下坐标
    cv::Vec3d rvec_ros, tvec_ros;   //Aruco姿态
    cv::Vec3d rvec_cv, tvec_cv;   //Aruco姿态

    geometry_msgs::msg::Pose aruco_pos_ros_;    //Aruco姿态
    geometry_msgs::msg::Pose aruco_pos_cv_;

private:
    std::vector<cv::Point3f> world_def_;    //世界坐标系定义
};


#endif //BUILD_ARUCO_H