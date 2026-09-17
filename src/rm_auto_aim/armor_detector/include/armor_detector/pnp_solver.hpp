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
    struct DynamicRParams
    {
        // 角度标准差，单位 rad
        double std_dev_azi_angle = 0.006; // 水平方位角
        double std_dev_ele_angle = 0.006; // 俯仰角

        // 深度标准差系数：sigma_depth = std_dev_dist_coeff * distance
        double std_dev_dist_coeff = 0.08;

        // 装甲板 yaw 标准差，单位 rad
        double std_dev_yaw = 0.15;

        // 防止近距离时 R 过小
        double min_tangent_std = 0.005; // 横向最小标准差，单位 m
        double min_depth_std = 0.02; // 深度最小标准差，单位 m
        double min_yaw_std = 0.05; // yaw 最小标准差，单位 rad

        // 防止极远距离时 R 过大，可以按需要调整
        double max_tangent_std = 0.50; // 单位 m
        double max_depth_std = 1.50; // 单位 m
        double max_yaw_std = 0.80; // 单位 rad

        // 用于质量调节。默认 1.0。
        // 如果角点质量差、重投影误差大，可以传入 > 1.0。
        double quality_scale = 1.0;
    };

    class PnPSolver
    {
    public:
        PnPSolver(
            const std::array<double, 9>& camera_matrix,
            const std::vector<double>& distortion_coefficients,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer);

        // Get 3d position
        bool solvePnP(Armor& armor, cv::Mat& rvec, cv::Mat& tvec, rclcpp::Time time);

        // Calculate the distance between armor center and image center
        float calculateDistanceToCenter(const cv::Point2f& image_point);

        //重投影误差
        double reprojectionError(
            const std::vector<cv::Point3f>& object_points,
            const std::vector<cv::Point2f>& image_points,
            const cv::Mat& rvec,
            const cv::Mat& tvec);
        //YAW优化
        void optimizeYaw(
            Armor& armor,
            const std::vector<cv::Point3f>& object_points,
            const std::vector<cv::Point2f>& image_points,
            cv::Mat& rvec,
            cv::Mat& tvec,
            rclcpp::Time time);

        //删除
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

        double fastReprojectionError(
            const Eigen::Matrix3d& R_ac,
            const Eigen::Vector3d& t_ac,
            const std::vector<cv::Point3f>& object_points,
            const std::vector<cv::Point2f>& image_points) const;

        double computeYawErrorFast(
            double yaw,
            const Eigen::Vector3d& t_cam,
            const std::vector<cv::Point3f>& object_points,
            const std::vector<cv::Point2f>& image_points) const;

        geometry_msgs::msg::PoseStamped TransformToTargetFrame(geometry_msgs::msg::PoseStamped point,
                                                               std::string target_frame);

    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        DynamicRParams R_params_;

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
} // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
