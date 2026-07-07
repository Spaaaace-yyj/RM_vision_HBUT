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
#include <memory>
#include <string>
#include <iostream>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
    struct MeasurementNoiseParams
    {
        // 下面 4 个参数尽量保持和开源代码一致。
        // 注意：这里都是“方差”，不是标准差。
        double r_azimuth = 4e-3; // ypd yaw 方差
        double r_pitch = 4e-3; // ypd pitch 方差
        double r_distance_base = 1.0; // distance 方差基础值
        double r_yaw_base = 9e-2; // armor yaw 方差基础值

        // 开源代码：log(distance + 1) / 200 + 9e-2
        double yaw_distance_log_div = 200.0;

        // 整体缩放，默认 1.0 表示先尽量保持开源量级
        double xyz_scale = 1.0;
        double yaw_scale = 1.0;

        // 可选：额外模型噪声。
        // 先设为 0，若近距离旋转时速度仍然跳，可设为 0.03~0.08。
        double model_xy_std = 0.0;
        double model_z_std = 0.0;
    };

    enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

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

        double info_position_diff;
        double info_yaw_diff;

        Eigen::VectorXd measurement;
        Eigen::Matrix4d R_meas_;

        Eigen::VectorXd target_state;

        // To store another pair of armors message
        double dz, another_r;

    private:
        void initEKF(const Armor& a);

        void updateArmorsNum(const Armor& a);

        void handleArmorJump(const Armor& a);

        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);

        double getRawYaw(const geometry_msgs::msg::Quaternion & q);

        double continuousYaw(double raw_yaw, double reference_yaw);

        double max_match_distance_;
        double max_match_yaw_diff_;

        int detect_count_;
        int lost_count_;

        double last_yaw_;

        MeasurementNoiseParams r_params;
    };

    inline double limitRad(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    inline double square(double x)
    {
        return x * x;
    }

    /**
     * @brief EKF坐标系下 xyz -> ypd
     *
     * 坐标约定：
     *   azimuth = atan2(y, x)
     *   pitch   = atan2(z, sqrt(x^2 + y^2))
     *   distance = sqrt(x^2 + y^2 + z^2)
     *
     * 这个约定和你给的开源代码 center_yaw = atan2(y, x) 对齐。
     */
    inline Eigen::Vector3d xyzToYpd(const Eigen::Vector3d& xyz)
    {
        const double x = xyz.x();
        const double y = xyz.y();
        const double z = xyz.z();

        const double xy_norm = std::hypot(x, y);
        const double distance = std::max(xyz.norm(), 1e-6);

        const double azimuth = std::atan2(y, x);
        const double pitch = std::atan2(z, xy_norm);

        return Eigen::Vector3d(azimuth, pitch, distance);
    }

    /**
     * @brief 计算 xyz = f(azimuth, pitch, distance) 对 ypd 的雅可比
     *
     * ypd = [azimuth, pitch, distance]
     *
     * x = d * cos(pitch) * cos(azimuth)
     * y = d * cos(pitch) * sin(azimuth)
     * z = d * sin(pitch)
     *
     * J = ∂xyz / ∂[azimuth, pitch, distance]
     */
    inline Eigen::Matrix3d xyzWrtYpdJacobian(
        double azimuth,
        double pitch,
        double distance)
    {
        const double ca = std::cos(azimuth);
        const double sa = std::sin(azimuth);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);

        Eigen::Matrix3d J;

        //             d/d azimuth        d/d pitch          d/d distance
        J << -distance * cp * sa, -distance * sp * ca, cp * ca,
            distance * cp * ca, -distance * sp * sa, cp * sa,
            0.0, distance * cp, sp;

        return J;
    }

    /**
     * @brief 根据 EKF 测量值 [x,y,z,yaw] 动态计算 R
     *
     * 输入的 armor_xyz 和 armor_yaw 必须和 EKF measurement 使用同一个坐标系。
     *
     * @param armor_xyz EKF测量坐标系下的装甲板位置 [x,y,z]
     * @param armor_yaw EKF测量坐标系下的装甲板yaw
     * @param params 噪声参数
     * @return 4x4 R，对应 measurement = [x, y, z, yaw]
     */
    inline Eigen::Matrix4d makeMeasurementNoiseR(
        const Eigen::Vector3d& armor_xyz,
        double armor_yaw,
        const MeasurementNoiseParams& params = MeasurementNoiseParams{})
    {
        Eigen::Matrix4d R = Eigen::Matrix4d::Zero();

        //用odom坐标系下的 xyz 计算 ypd
        const Eigen::Vector3d ypd = xyzToYpd(armor_xyz);

        const double azimuth = ypd[0];
        const double pitch = ypd[1];
        const double distance = ypd[2];

        // center_yaw = atan2(y, x)
        // delta_angle = limit_rad(armor_yaw - center_yaw)
        const double center_yaw = azimuth;
        const double delta_angle = limitRad(armor_yaw - center_yaw);

        //先在 ypd 空间下构造 R_ypd
        Eigen::Matrix3d R_ypd = Eigen::Matrix3d::Zero();

        R_ypd(0, 0) = params.r_azimuth;
        R_ypd(1, 1) = params.r_pitch;
        R_ypd(2, 2) =
            std::log(std::abs(delta_angle) + 1.0) + params.r_distance_base;

        const double R_yaw =
            std::log(std::abs(distance) + 1.0) / params.yaw_distance_log_div +
            params.r_yaw_base;

        //因为你们 EKF 更新用的是 [x,y,z,yaw]，
        //所以把 R_ypd 通过雅可比转换到 xyz 空间
        const Eigen::Matrix3d J_xyz_ypd =
            xyzWrtYpdJacobian(azimuth, pitch, distance);

        Eigen::Matrix3d R_xyz =
            J_xyz_ypd * R_ypd * J_xyz_ypd.transpose();

        //整体缩放，默认不缩放
        R_xyz *= params.xyz_scale;

        //可选模型噪声下限。
        //这不是开源原始代码的一部分，但对你们 XYZ 观测模型很有用。
        R_xyz(0, 0) += square(params.model_xy_std);
        R_xyz(1, 1) += square(params.model_xy_std);
        R_xyz(2, 2) += square(params.model_z_std);

        //组装成 [x,y,z,yaw] 的 4x4 R
        R.block<3, 3>(0, 0) = R_xyz;
        R(3, 3) = params.yaw_scale * R_yaw;

        //强制对称，避免数值误差
        R = 0.5 * (R + R.transpose());

        return R;
    }

    /**
     * @brief 调试用：打印 R 的标准差量级
     */
    inline void printMeasurementNoiseR(const Eigen::Matrix4d& R)
    {
        std::cout << "R diag std = "
            << std::sqrt(std::max(0.0, R(0, 0))) << ", "
            << std::sqrt(std::max(0.0, R(1, 1))) << ", "
            << std::sqrt(std::max(0.0, R(2, 2))) << ", "
            << std::sqrt(std::max(0.0, R(3, 3))) << std::endl;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(R.block<3, 3>(0, 0));
        std::cout << "R xyz eig std = "
            << solver.eigenvalues().cwiseMax(0.0).cwiseSqrt().transpose()
            << std::endl;
    }
} // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
