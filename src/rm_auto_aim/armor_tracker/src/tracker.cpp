// Copyright 2022 Chen Jun

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace rm_auto_aim
{
    Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
        : tracker_state(LOST),
          tracked_id(std::string("")),
          measurement(Eigen::VectorXd::Zero(4)),
          target_state(Eigen::VectorXd::Zero(11)),
          max_match_distance_(max_match_distance),
          max_match_yaw_diff_(max_match_yaw_diff)
    {
        r_params.r_azimuth = 4e-3;
        r_params.r_pitch = 4e-3;
        r_params.r_distance_base = 1.0;
        r_params.r_yaw_base = 9e-2;
        r_params.yaw_distance_log_div = 200.0;
        r_params.distance_scale = 1.0;
        r_params.yaw_scale = 1.0;
    }

    void Tracker::init(const Armors::SharedPtr& armors_msg)
    {
        if (armors_msg->armors.empty())
        {
            return;
        }

        // Simply choose the armor that is closest to image center
        double min_distance = DBL_MAX;
        tracked_armor = armors_msg->armors[0];
        for (const auto& armor : armors_msg->armors)
        {
            if (armor.distance_to_image_center < min_distance)
            {
                min_distance = armor.distance_to_image_center;
                tracked_armor = armor;
            }
        }

        tracked_id = tracked_armor.number;
        updateArmorsNum(tracked_armor);
        initEKF(tracked_armor);
        RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init 11D EKF!");

        tracker_state = DETECTING;
    }

    void Tracker::update(const Armors::SharedPtr& armors_msg)
    {
        // EKF predict
        Eigen::VectorXd ekf_prediction = ekf.predict();
        bool matched = false;
        target_state = ekf_prediction;

        int same_id_armors_count = 0;

        if (!armors_msg->armors.empty())
        {
            for (const auto& armor : armors_msg->armors)
            {
                if (armor.number != tracked_id)
                {
                    continue;
                }

                // 尽量保持目标类型一致。若某一帧大小分类抖动，可以先只按 number 匹配。
                if (!tracked_armor.type.empty() && !armor.type.empty() && armor.type != tracked_armor.type)
                {
                    continue;
                }

                same_id_armors_count++;
                if (updateOneArmor(armor))
                {
                    matched = true;
                    tracked_armor = armor;
                    updateArmorsNum(tracked_armor);
                }
            }
        }

        if (!matched && same_id_armors_count == 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found! Same armor num = 0");
        }

        clampGeometryState();
        syncCompatibilityFields();

        // Tracking state machine
        if (tracker_state == DETECTING)
        {
            if (matched)
            {
                detect_count_++;
                if (detect_count_ > tracking_thres)
                {
                    detect_count_ = 0;
                    tracker_state = TRACKING;
                }
            }
            else
            {
                detect_count_ = 0;
                tracker_state = LOST;
            }
        }
        else if (tracker_state == TRACKING)
        {
            if (!matched)
            {
                tracker_state = TEMP_LOST;
                lost_count_++;
            }
        }
        else if (tracker_state == TEMP_LOST)
        {
            if (!matched)
            {
                lost_count_++;
                if (lost_count_ > lost_thres)
                {
                    lost_count_ = 0;
                    tracker_state = LOST;
                }
            }
            else
            {
                tracker_state = TRACKING;
                lost_count_ = 0;
            }
        }
    }

    void Tracker::initEKF(const Armor& a)
    {
        const double xa = a.pose.position.x;
        const double ya = a.pose.position.y;
        const double za = a.pose.position.z;

        last_yaw_ = 0.0;
        const double raw_yaw = getRawYaw(a.pose.orientation);
        const double yaw = continuousYaw(raw_yaw, 0.0);
        last_yaw_ = yaw;

        const double r = initialRadius(a);
        const double xc = xa + r * std::cos(yaw);
        const double yc = ya + r * std::sin(yaw);

        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r, l, h
        // l = r2 - r1, h = z2 - z1
        target_state = Eigen::VectorXd::Zero(11);
        target_state << xc, 0.0, yc, 0.0, za, 0.0, yaw, 0.0, r, 0.0, 0.0;

        ekf.setState(target_state, initialCovariance(a));
        syncCompatibilityFields();
    }

    void Tracker::updateArmorsNum(const Armor& armor)
    {
        if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
        {
            tracked_armors_num = ArmorsNum::BALANCE_2;
        }
        else if (tracked_id == "outpost")
        {
            tracked_armors_num = ArmorsNum::OUTPOST_3;
        }
        else
        {
            tracked_armors_num = ArmorsNum::NORMAL_4;
        }
    }

    bool Tracker::updateOneArmor(const Armor& a)
    {
        if (target_state.size() != 11)
        {
            return false;
        }

        const int id = matchArmorId(a, target_state);
        last_model_id = id;

        const Eigen::Vector4d z = makeArmorYpdaMeasurement(a);
        const Eigen::Vector3d armor_xyz(a.pose.position.x, a.pose.position.y, a.pose.position.z);
        const Eigen::Matrix4d R = makeYpdaMeasurementNoiseR(armor_xyz, z[3], r_params);
        const Eigen::MatrixXd H = hJacobian(target_state, id);

        auto h = [this, id](const Eigen::VectorXd& x) -> Eigen::VectorXd {
            return hArmorYPDA(x, id);
        };

        auto z_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
            Eigen::VectorXd c = a - b;
            if (c.size() >= 4)
            {
                c[0] = limitRad(c[0]);
                c[1] = limitRad(c[1]);
                c[3] = limitRad(c[3]);
            }
            return c;
        };

        // 调试信息：当前观测和选中的模型装甲板之间的差异
        const Eigen::Vector4d z_pred = hArmorYPDA(target_state, id);
        const Eigen::Vector3d pred_xyz = hArmorXYZ(target_state, id);
        info_position_diff = (pred_xyz - armor_xyz).norm();
        info_yaw_diff = std::abs(angles::shortest_angular_distance(z_pred[3], z[3]));

        measurement = z;
        R_meas_ = R;
        target_state = ekf.update(z, H, R, h, z_subtract);
        target_state(6) = limitRad(target_state(6));
        ekf.setState(target_state);

        return true;
    }

    int Tracker::matchArmorId(const Armor& a, const Eigen::VectorXd& x) const
    {
        const auto xyza_list = armorXYZAList(x);
        if (xyza_list.empty())
        {
            return 0;
        }

        std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
        xyza_i_list.reserve(xyza_list.size());
        for (int i = 0; i < static_cast<int>(xyza_list.size()); ++i)
        {
            xyza_i_list.push_back({xyza_list[i], i});
        }

        std::sort(
            xyza_i_list.begin(), xyza_i_list.end(),
            [](const auto& lhs, const auto& rhs) {
                const Eigen::Vector3d lhs_xyz(lhs.first(0), lhs.first(1), lhs.first(2));
                const Eigen::Vector3d rhs_xyz(rhs.first(0), rhs.first(1), rhs.first(2));

                return lhs_xyz.norm() < rhs_xyz.norm();
            });

        const Eigen::Vector3d armor_xyz(a.pose.position.x, a.pose.position.y, a.pose.position.z);
        const Eigen::Vector3d armor_ypd = xyzToYpd(armor_xyz);
        const double raw_yaw = getRawYaw(a.pose.orientation);

        int best_id = xyza_i_list.front().second;
        double min_angle_error = DBL_MAX;
        const int check_num = std::min<int>(3, xyza_i_list.size());

        for (int i = 0; i < check_num; ++i)
        {
            const auto& xyza = xyza_i_list[i].first;
            const Eigen::Vector3d pred_ypd = xyzToYpd(xyza.head<3>());

            const double armor_yaw_error = std::abs(angles::shortest_angular_distance(xyza[3], raw_yaw));
            const double view_yaw_error = std::abs(angles::shortest_angular_distance(pred_ypd[0], armor_ypd[0]));
            const double angle_error = armor_yaw_error + view_yaw_error;

            if (angle_error < min_angle_error)
            {
                min_angle_error = angle_error;
                best_id = xyza_i_list[i].second;
            }
        }

        return best_id;
    }

    Eigen::Vector4d Tracker::makeArmorYpdaMeasurement(const Armor& a) const
    {
        const Eigen::Vector3d xyz(a.pose.position.x, a.pose.position.y, a.pose.position.z);
        const Eigen::Vector3d ypd = xyzToYpd(xyz);
        const double raw_yaw = getRawYaw(a.pose.orientation);

        return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], raw_yaw);
    }

    Eigen::Vector3d Tracker::hArmorXYZ(const Eigen::VectorXd& x, int id) const
    {
        const int n = std::max(1, armorsNum());
        const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(n));
        const bool use_l_h = (tracked_armors_num == ArmorsNum::NORMAL_4) && (id == 1 || id == 3);

        const double r = use_l_h ? x[8] + x[9] : x[8];
        const double armor_x = x[0] - r * std::cos(angle);
        const double armor_y = x[2] - r * std::sin(angle);
        const double armor_z = use_l_h ? x[4] + x[10] : x[4];

        return Eigen::Vector3d(armor_x, armor_y, armor_z);
    }

    Eigen::Vector4d Tracker::hArmorYPDA(const Eigen::VectorXd& x, int id) const
    {
        const int n = std::max(1, armorsNum());
        const Eigen::Vector3d xyz = hArmorXYZ(x, id);
        const Eigen::Vector3d ypd = xyzToYpd(xyz);
        const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(n));

        return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);
    }

    Eigen::MatrixXd Tracker::hJacobian(const Eigen::VectorXd& x, int id) const
    {
        const int n = std::max(1, armorsNum());
        const double angle = limitRad(x[6] + id * 2.0 * M_PI / static_cast<double>(n));
        const bool use_l_h = (tracked_armors_num == ArmorsNum::NORMAL_4) && (id == 1 || id == 3);

        const double r = use_l_h ? x[8] + x[9] : x[8];
        const double dx_da = r * std::sin(angle);
        const double dy_da = -r * std::cos(angle);

        const double dx_dr = -std::cos(angle);
        const double dy_dr = -std::sin(angle);
        const double dx_dl = use_l_h ? -std::cos(angle) : 0.0;
        const double dy_dl = use_l_h ? -std::sin(angle) : 0.0;

        const double dz_dh = use_l_h ? 1.0 : 0.0;

        // ∂[armor_x, armor_y, armor_z, armor_yaw] / ∂state
        Eigen::MatrixXd H_armor_xyza(4, 11);
        // clang-format off
        H_armor_xyza <<
            1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0,
            0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0,
            0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh,
            0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0;
        // clang-format on

        const Eigen::Vector3d armor_xyz = hArmorXYZ(x, id);
        const Eigen::Matrix3d H_ypd_xyz = xyzToYpdJacobian(armor_xyz);

        Eigen::MatrixXd H_armor_ypda(4, 4);
        // clang-format off
        H_armor_ypda <<
            H_ypd_xyz(0, 0), H_ypd_xyz(0, 1), H_ypd_xyz(0, 2), 0,
            H_ypd_xyz(1, 0), H_ypd_xyz(1, 1), H_ypd_xyz(1, 2), 0,
            H_ypd_xyz(2, 0), H_ypd_xyz(2, 1), H_ypd_xyz(2, 2), 0,
                         0,              0,              0, 1;
        // clang-format on

        return H_armor_ypda * H_armor_xyza;
    }

    std::vector<Eigen::Vector4d> Tracker::armorXYZAList(const Eigen::VectorXd& x) const
    {
        std::vector<Eigen::Vector4d> list;
        const int n = std::max(1, armorsNum());
        list.reserve(n);

        for (int i = 0; i < n; ++i)
        {
            const double angle = limitRad(x[6] + i * 2.0 * M_PI / static_cast<double>(n));
            const Eigen::Vector3d xyz = hArmorXYZ(x, i);
            list.emplace_back(xyz[0], xyz[1], xyz[2], angle);
        }
        return list;
    }

    void Tracker::syncCompatibilityFields()
    {
        if (target_state.size() < 11)
        {
            return;
        }
        target_state(6) = limitRad(target_state(6));
        another_r = target_state(8) + target_state(9);
        dz = target_state(10);
    }

    void Tracker::clampGeometryState()
    {
        if (target_state.size() < 11)
        {
            return;
        }

        double r1 = target_state(8);
        double r2 = target_state(8) + target_state(9);

        r1 = std::clamp(r1, 0.05, 0.50);
        r2 = std::clamp(r2, 0.05, 0.50);

        target_state(8) = r1;
        target_state(9) = r2 - r1;
        target_state(10) = std::clamp(target_state(10), -0.50, 0.50);
        target_state(6) = limitRad(target_state(6));

        ekf.setState(target_state);
    }

    double Tracker::initialRadius(const Armor& armor) const
    {
        if (armor.number == "outpost")
        {
            return 0.2765;
        }
        if (armor.number == "base")
        {
            return 0.3205;
        }
        return 0.20;
    }

    Eigen::MatrixXd Tracker::initialCovariance(const Armor& armor) const
    {
        Eigen::VectorXd p0(11);
        const bool is_balance = armor.type == "large" &&
            (armor.number == "3" || armor.number == "4" || armor.number == "5");

        if (is_balance)
        {
            p0 << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
        }
        else if (armor.number == "outpost")
        {
            p0 << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
        }
        else if (armor.number == "base")
        {
            p0 << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
        }
        else
        {
            p0 << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
        }

        return p0.asDiagonal();
    }

    double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        const double raw_yaw = getRawYaw(q);
        const double yaw = continuousYaw(raw_yaw, last_yaw_);
        last_yaw_ = yaw;
        return yaw;
    }

    double Tracker::getRawYaw(const geometry_msgs::msg::Quaternion& q) const
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        return yaw;
    }

    double Tracker::continuousYaw(double raw_yaw, double reference_yaw) const
    {
        return reference_yaw + angles::shortest_angular_distance(reference_yaw, raw_yaw);
    }
} // namespace rm_auto_aim