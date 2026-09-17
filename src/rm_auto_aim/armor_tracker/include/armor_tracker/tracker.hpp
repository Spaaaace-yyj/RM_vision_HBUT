// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
    struct MeasurementNoiseParams
    {
        // 对应开源 ypda 观测: [azimuth, pitch, distance, armor_yaw]
        // 注意：这里都是“方差”，不是标准差。
        double r_azimuth = 4e-3;
        double r_pitch = 4e-3;
        double r_distance_base = 1.0;
        double r_yaw_base = 9e-2;
        double yaw_distance_log_div = 200.0;

        double distance_scale = 1.0;
        double yaw_scale = 1.0;
    };

    enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

    inline double limitRad(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    inline double square(double x) { return x * x; }

    /**
     * @brief EKF坐标系下 xyz -> ypd
     * ypd = [azimuth, pitch/elevation, distance]
     */
    inline Eigen::Vector3d xyzToYpd(const Eigen::Vector3d& xyz)
    {
        const double x = xyz.x();
        const double y = xyz.y();
        const double z = xyz.z();

        const double xy_norm = std::max(std::hypot(x, y), 1e-6);
        const double distance = std::max(xyz.norm(), 1e-6);

        const double azimuth = std::atan2(y, x);
        const double pitch = std::atan2(z, xy_norm);

        return Eigen::Vector3d(azimuth, pitch, distance);
    }

    /**
     * @brief 计算 ypd = f(xyz) 的雅可比 ∂[azimuth,pitch,distance]/∂[x,y,z]
     */
    inline Eigen::Matrix3d xyzToYpdJacobian(const Eigen::Vector3d& xyz)
    {
        const double x = xyz.x();
        const double y = xyz.y();
        const double z = xyz.z();

        const double xy2 = std::max(x * x + y * y, 1e-9);
        const double xy = std::sqrt(xy2);
        const double d2 = std::max(xy2 + z * z, 1e-9);
        const double d = std::sqrt(d2);

        Eigen::Matrix3d J;
        // azimuth = atan2(y, x)
        J(0, 0) = -y / xy2;
        J(0, 1) = x / xy2;
        J(0, 2) = 0.0;

        // pitch = atan2(z, sqrt(x^2+y^2))
        J(1, 0) = -x * z / (xy * d2);
        J(1, 1) = -y * z / (xy * d2);
        J(1, 2) = xy / d2;

        // distance = sqrt(x^2+y^2+z^2)
        J(2, 0) = x / d;
        J(2, 1) = y / d;
        J(2, 2) = z / d;

        return J;
    }

    /**
     * @brief 根据开源逻辑构造 ypda 观测噪声 R。
     * measurement = [ypd_yaw, ypd_pitch, ypd_distance, armor_yaw]
     */
    inline Eigen::Matrix4d makeYpdaMeasurementNoiseR(
        const Eigen::Vector3d& armor_xyz,
        double armor_yaw,
        const MeasurementNoiseParams& params = MeasurementNoiseParams{})
    {
        Eigen::Matrix4d R = Eigen::Matrix4d::Zero();

        const Eigen::Vector3d ypd = xyzToYpd(armor_xyz);
        const double center_yaw = ypd[0];
        const double distance = ypd[2];
        const double delta_angle = limitRad(armor_yaw - center_yaw);

        R(0, 0) = params.r_azimuth;
        R(1, 1) = params.r_pitch;
        R(2, 2) = params.distance_scale * (std::log(std::abs(delta_angle) + 1.0) + params.r_distance_base);
        R(3, 3) = params.yaw_scale * (
            std::log(std::abs(distance) + 1.0) / params.yaw_distance_log_div + params.r_yaw_base);

        return R;
    }

    class Tracker
    {
    public:
        Tracker(double max_match_distance, double max_match_yaw_diff);

        using Armors = auto_aim_interfaces::msg::Armors;
        using Armor = auto_aim_interfaces::msg::Armor;

        void init(const Armors::SharedPtr& armors_msg);

        void update(const Armors::SharedPtr& armors_msg);

        ExtendedKalmanFilter ekf;

        int tracking_thres;
        int lost_thres;

        enum State
        {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        } tracker_state;

        std::string tracked_id;
        Armor tracked_armor;
        ArmorsNum tracked_armors_num;

        double info_position_diff = 0.0;
        double info_yaw_diff = 0.0;
        int last_model_id = 0;

        // measurement = [ypd_yaw, ypd_pitch, ypd_distance, armor_yaw]
        Eigen::VectorXd measurement;
        Eigen::Matrix4d R_meas_;

        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r, l, h
        // l: r2 - r1, h: z2 - z1
        Eigen::VectorXd target_state;

        // For compatibility with old Target msg mapping
        double dz = 0.0;
        double another_r = 0.0;

        std::vector<Eigen::Vector4d> armorXYZAList(const Eigen::VectorXd& x) const;

    private:
        void initEKF(const Armor& a);

        void updateArmorsNum(const Armor& a);

        bool updateOneArmor(const Armor& a);

        int matchArmorId(const Armor& a, const Eigen::VectorXd& x) const;

        Eigen::Vector4d makeArmorYpdaMeasurement(const Armor& a) const;

        Eigen::Vector3d hArmorXYZ(const Eigen::VectorXd& x, int id) const;

        Eigen::Vector4d hArmorYPDA(const Eigen::VectorXd& x, int id) const;

        Eigen::MatrixXd hJacobian(const Eigen::VectorXd& x, int id) const;

        void syncCompatibilityFields();

        void clampGeometryState();

        double initialRadius(const Armor& armor) const;

        Eigen::MatrixXd initialCovariance(const Armor& armor) const;

        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);

        double getRawYaw(const geometry_msgs::msg::Quaternion & q) const;

        double continuousYaw(double raw_yaw, double reference_yaw) const;

        int armorsNum() const { return static_cast<int>(tracked_armors_num); }

        double max_match_distance_;
        double max_match_yaw_diff_;

        int detect_count_ = 0;
        int lost_count_ = 0;

        double last_yaw_ = 0.0;

        MeasurementNoiseParams r_params;
    };

} // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_