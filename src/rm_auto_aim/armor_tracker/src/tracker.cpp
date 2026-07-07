// Copyright 2022 Chen Jun

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
    Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
        : tracker_state(LOST),
          tracked_id(std::string("")),
          measurement(Eigen::VectorXd::Zero(4)),
          target_state(Eigen::VectorXd::Zero(9)),
          max_match_distance_(max_match_distance),
          max_match_yaw_diff_(max_match_yaw_diff)
    {
        r_params.r_azimuth = 4e-3;
        r_params.r_pitch = 4e-3;
        r_params.r_distance_base = 1.0;
        r_params.r_yaw_base = 9e-2;
        r_params.yaw_distance_log_div = 200.0;
        r_params.xyz_scale = 1.0;
        r_params.yaw_scale = 1.0;

        // 先关掉额外模型噪声，尽量复刻开源
        r_params.model_xy_std = 0.0;
        r_params.model_z_std = 0.0;
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

        initEKF(tracked_armor);
        RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

        tracked_id = tracked_armor.number;
        tracker_state = DETECTING;

        updateArmorsNum(tracked_armor);
    }

    void Tracker::update(const Armors::SharedPtr& armors_msg)
    {
        // KF predict
        Eigen::VectorXd ekf_prediction = ekf.predict();
        // RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        target_state = ekf_prediction;

        //todo:做成参数
        double max_match_maha_distance_ = 13.28;
        //todo:修改按照马式距离判断当前追踪的是哪个装甲板
        //寻找距离EKF预测的装甲板最近的观测装甲板作为目标进行更新
        if (!armors_msg->armors.empty())
        {
            Armor best_armor;
            Armor same_id_armor;

            bool has_best_armor = false;
            int same_id_armors_count = 0;

            double best_maha = DBL_MAX;
            double best_position_diff = DBL_MAX;
            double best_yaw_diff = DBL_MAX;
            double best_measured_yaw = 0.0;
            Eigen::Matrix4d best_R = Eigen::Matrix4d::Identity();

            auto predicted_position = getArmorPositionFromState(ekf_prediction);

            for (const auto& armor : armors_msg->armors)
            {
                if (armor.number != tracked_id)
                {
                    continue;
                }

                same_id_armor = armor;
                same_id_armors_count++;

                auto p = armor.pose.position;
                Eigen::Vector3d armor_xyz(p.x, p.y, p.z);

                // 不要在候选循环里调用会修改 last_yaw_ 的 orientationToYaw()
                double raw_yaw = getRawYaw(armor.pose.orientation);
                double measured_yaw = continuousYaw(raw_yaw, ekf_prediction(6));

                Eigen::Vector4d z;
                z << p.x, p.y, p.z, measured_yaw;

                Eigen::Matrix4d R_meas = makeMeasurementNoiseR(
                    armor_xyz,
                    measured_yaw,
                    r_params);

                double maha = ekf.mahalanobisDistance(z, R_meas);

                double position_diff = (predicted_position - armor_xyz).norm();
                double yaw_diff = std::abs(
                    angles::shortest_angular_distance(ekf_prediction(6), measured_yaw));

                if (maha < best_maha)
                {
                    best_maha = maha;
                    best_position_diff = position_diff;
                    best_yaw_diff = yaw_diff;
                    best_armor = armor;
                    best_measured_yaw = measured_yaw;
                    best_R = R_meas;
                    has_best_armor = true;
                }
            }

            info_position_diff = best_position_diff;
            info_yaw_diff = best_yaw_diff;

            if (has_best_armor && best_maha < max_match_maha_distance_ && best_yaw_diff < max_match_yaw_diff_)
            {
                matched = true;
                tracked_armor = best_armor;

                auto p = tracked_armor.pose.position;
                measurement = Eigen::Vector4d(p.x, p.y, p.z, best_measured_yaw);

                target_state = ekf.update(measurement, best_R);

                // 只有真正 update 的 yaw 才更新 last_yaw_
                last_yaw_ = best_measured_yaw;

                // RCLCPP_DEBUG(
                //     rclcpp::get_logger("armor_tracker"),
                //     "[MAHA MATCH] d2=%.3f, pos_diff=%.3f, yaw_diff=%.3f deg",
                //     best_maha,
                //     best_position_diff,
                //     best_yaw_diff * 180.0 / M_PI);
            }
            else if (same_id_armors_count == 1 && best_yaw_diff > max_match_yaw_diff_)
            {
                // Matched armor not found, but there is only one armor with the same id
                // and yaw has jumped, take this case as the target is spinning and armor jumped
                handleArmorJump(same_id_armor);
            }
            else if (same_id_armors_count == 1)
            {
                auto p = same_id_armor.pose.position;
                Eigen::Vector3d armor_xyz(p.x, p.y, p.z);

                double raw_yaw = getRawYaw(same_id_armor.pose.orientation);
                double measured_yaw = continuousYaw(raw_yaw, ekf_prediction(6));

                measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

                Eigen::Matrix4d R_meas = makeMeasurementNoiseR(
                    armor_xyz,
                    measured_yaw,
                    r_params);

                // force update 这里建议加一个更宽的 maha 阈值
                double maha = ekf.mahalanobisDistance(measurement, R_meas);

                if (maha < max_match_maha_distance_ * 2.0)
                {
                    target_state = ekf.update(measurement, R_meas);
                    matched = true;
                    tracked_armor = same_id_armor;
                    last_yaw_ = measured_yaw;

                    RCLCPP_DEBUG(
                        rclcpp::get_logger("armor_tracker"),
                        "[FORCE MAHA UPDATE] d2=%.3f",
                        maha);

                    handleArmorJump(same_id_armor);
                }
                else
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("armor_tracker"),
                        "[REJECT SINGLE ARMOR] maha d2=%.3f",
                        maha);
                }
            }
            else
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("armor_tracker"),
                    "No matched armor found! Same armor num = %d",
                    same_id_armors_count);
            }
        }

        // Prevent radius from spreading
        if (target_state(8) < 0.12)
        {
            target_state(8) = 0.12;
            ekf.setState(target_state);
        }
        else if (target_state(8) > 0.4)
        {
            target_state(8) = 0.4;
            ekf.setState(target_state);
        }

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
        double xa = a.pose.position.x;
        double ya = a.pose.position.y;
        double za = a.pose.position.z;
        last_yaw_ = 0;
        double yaw = orientationToYaw(a.pose.orientation);

        // Set initial position at 0.2m behind the target
        target_state = Eigen::VectorXd::Zero(9);
        double r = 0.26; //0.26
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        dz = 0, another_r = r;
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
        target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

        ekf.setState(target_state);
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

    void Tracker::handleArmorJump(const Armor& current_armor)
    {
        double yaw = orientationToYaw(current_armor.pose.orientation);
        target_state(6) = yaw;
        updateArmorsNum(current_armor);
        // Only 4 armors has 2 radius and height
        if (tracked_armors_num == ArmorsNum::NORMAL_4)
        {
            dz = target_state(4) - current_armor.pose.position.z;
            target_state(4) = current_armor.pose.position.z;
            std::swap(target_state(8), another_r);
        }
        RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Armor jump!, max_match_distance = [%f]",
                     max_match_distance_);

        // If position difference is larger than max_match_distance_,
        // take this case as the ekf diverged, reset the state
        auto p = current_armor.pose.position;
        Eigen::Vector3d current_p(p.x, p.y, p.z);
        Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
        if ((current_p - infer_p).norm() > max_match_distance_)
        {
            double r = target_state(8);
            target_state(0) = p.x + r * cos(yaw); // xc
            target_state(1) = 0; // vxc
            target_state(2) = p.y + r * sin(yaw); // yc
            target_state(3) = 0; // vyc
            target_state(4) = p.z; // za
            target_state(5) = 0; // vza
            RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State! Current dis to EKF = [%f]",
                         (current_p - infer_p).norm());
        }

        ekf.setState(target_state);
    }

    double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        // Get armor yaw
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        // Make yaw change continuous (-pi~pi to -inf~inf)
        yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
        last_yaw_ = yaw;
        return yaw;
    }

    double Tracker::getRawYaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        return yaw;
    }

    double Tracker::continuousYaw(double raw_yaw, double reference_yaw)
    {
        return reference_yaw + angles::shortest_angular_distance(reference_yaw, raw_yaw);
    }

    Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd& x)
    {
        // Calculate predicted position of the current armor
        double xc = x(0), yc = x(2), za = x(4);
        double yaw = x(6), r = x(8);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        return Eigen::Vector3d(xa, ya, za);
    }
} // namespace rm_auto_aim
