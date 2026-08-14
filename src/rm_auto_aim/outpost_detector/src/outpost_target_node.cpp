#include "outpost_detector/outpost_target_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace outpost_auto_aim {

OutpostTargetNode::OutpostTargetNode(const rclcpp::NodeOptions& options)
    : Node("outpost_target_node", options),
      tf2_buffer_(this->get_clock()),
      tf2_listener_(tf2_buffer_) {
    // ---- 参数 ----
    target_frame_ = this->declare_parameter("target_frame", "odom");
    camera_frame_ = this->declare_parameter("camera_frame", "camera_optical_frame");
    const std::vector<double> slot_z =
        this->declare_parameter("slot_z", std::vector<double>{1.10, 1.27, 1.45});
    for (int i = 0; i < SLOT_NUM; ++i) {
        slot_z_[i] = slot_z[i];
    }
    z_tol_ = this->declare_parameter("z_tol", 0.08);
    const std::vector<int64_t> priority = this->declare_parameter(
        "slot_priority", std::vector<int64_t>{SLOT_LOW, SLOT_MID, SLOT_HIGH});
    slot_priority_.assign(priority.begin(), priority.end());
    hold_time_ = this->declare_parameter("hold_time", 3.0);
    lost_time_ = this->declare_parameter("lost_time", 5.0);
    min_sightings_ = this->declare_parameter("min_sightings", 2);
    pos_alpha_ = this->declare_parameter("pos_ema_alpha", 0.5);
    vel_alpha_ = this->declare_parameter("vel_ema_alpha", 0.3);
    outpost_x_ = this->declare_parameter("outpost_x", 0.0);
    outpost_y_ = this->declare_parameter("outpost_y", 0.0);
    outpost_radius_ = this->declare_parameter("outpost_radius", 0.0);
    accept_any_small_ = this->declare_parameter("accept_any_small", false);
    debug_ = this->declare_parameter("debug", false);

    // ---- 订阅 / 发布 ----
    armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](auto_aim_interfaces::msg::Armors::SharedPtr msg) {
            armorsCallback(msg);
        });
    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS());
    if (debug_) {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/outpost/marker", 10);
    }
}

void OutpostTargetNode::armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    // 回放没有时间戳时用节点时钟兜底
    double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const bool has_stamp = now_sec >= 1e-6;
    if (!has_stamp) {
        now_sec = this->now().seconds();
    }
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();

    // 1. 逐块变换到目标系，变换失败整帧丢弃
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(msg->armors.size());
    for (const auto& armor : msg->armors) {
        geometry_msgs::msg::Pose pose;
        if (!transformToTarget(armor.pose, pose, msg->header.frame_id, stamp)) {
            return;
        }
        poses.push_back(pose);
    }

    // 2. 更新槽位状态
    updateSlots(msg, poses, stamp, now_sec);

    // 3. 决定锁定哪块板，并发布目标
    const int slot = decideSlot(now_sec);
    if (slot >= 0 && slots_[slot].has_position) {
        publishTarget(slots_[slot], msg->header.stamp, now_sec);
    } else {
        auto_aim_interfaces::msg::Target target_msg;
        target_msg.header.stamp = msg->header.stamp;
        target_msg.header.frame_id = target_frame_;
        target_msg.tracking = false;
        target_msg.id = "outpost";
        // armors_num 置 0，云台看到 0 才知道目标丢了；
        // 发 1 加零位置会让云台去瞄原点
        target_msg.armors_num = 0;
        target_msg.type = "small";
        target_pub_->publish(target_msg);
    }

    if (debug_) {
        publishMarkers();
    }
}

bool OutpostTargetNode::transformToTarget(
    const geometry_msgs::msg::Pose& in, geometry_msgs::msg::Pose& out,
    const std::string& in_frame, const rclcpp::Time& stamp) {
    // 目标系和输入系相同时直接透传，离线调试用
    if (in_frame == target_frame_) {
        out = in;
        return true;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = in_frame;
    ps.header.stamp = stamp;
    ps.pose = in;
    try {
        out = tf2_buffer_.transform(ps, target_frame_).pose;
        return true;
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "tf transform failed: %s", error.what());
        return false;
    }
}

void OutpostTargetNode::updateSlots(
    const auto_aim_interfaces::msg::Armors::SharedPtr msg,
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const rclcpp::Time& seen_time,
    double now_sec) {
    // 每个槽位本帧只收一个检测，多个时取离带中心最近的
    std::array<bool, SLOT_NUM> slot_seen{false, false, false};

    for (std::size_t i = 0; i < msg->armors.size(); ++i) {
        const auto& armor = msg->armors[i];
        // 只要识别成前哨站的板，识别不出来时可以兜底收小装甲板
        if (armor.number != "outpost") {
            if (!accept_any_small_ || armor.type != "small") {
                continue;
            }
        }

        const geometry_msgs::msg::Pose& pose = poses[i];
        // 前哨站位置门，半径 <= 0 表示关闭
        if (outpost_radius_ > 0.0) {
            const double dx = pose.position.x - outpost_x_;
            const double dy = pose.position.y - outpost_y_;
            if (std::hypot(dx, dy) > outpost_radius_) {
                continue;
            }
        }

        // 按 odom 系 z 分槽位，odom 原点在地面
        const double z = pose.position.z;
        int best_slot = -1;
        double best_dist = 1e9;
        for (int s = 0; s < SLOT_NUM; ++s) {
            const double dist = std::abs(z - slot_z_[s]);
            if (dist <= z_tol_ && dist < best_dist) {
                best_dist = dist;
                best_slot = s;
            }
        }
        if (best_slot < 0 || slot_seen[best_slot]) {
            continue;
        }
        slot_seen[best_slot] = true;

        SlotState& slot = slots_[best_slot];
        const Eigen::Vector3f raw(
            pose.position.x, pose.position.y, pose.position.z);
        const double dt = now_sec - slot.last_raw_time;
        if (slot.has_position) {
            if (dt > 0.25) {
                // 隔太久没见到，旧速度方向已经不对，清零
                slot.vel_ema.setZero();
            } else if (dt > 0.01) {
                const Eigen::Vector3f v = (raw - slot.last_raw) / dt;
                slot.vel_ema = (1.0 - vel_alpha_) * slot.vel_ema + vel_alpha_ * v;
            }
            slot.pos_ema = (1.0 - pos_alpha_) * slot.pos_ema + pos_alpha_ * raw;
        } else {
            slot.pos_ema = raw;
            slot.has_position = true;
        }
        slot.last_raw = raw;
        slot.last_raw_time = now_sec;
        slot.last_seen = seen_time;
        slot.sightings++;
    }
}

int OutpostTargetNode::decideSlot(double now_sec) {
    const auto seen_ago = [&](int s) {
        return now_sec - slots_[s].last_seen.seconds();
    };

    // 锁定中的槽位还在正常出现，继续打它，不来回换
    if (active_slot_ >= 0) {
        const double ago = seen_ago(active_slot_);
        if (ago <= hold_time_) {
            return active_slot_;
        }
        if (ago > lost_time_) {
            RCLCPP_INFO(get_logger(), "outpost slot %d lost, reset", active_slot_);
            active_slot_ = -1;
            // 全丢这么久，槽位里的位置和速度都是过期的，清掉重新攒
            for (auto& slot : slots_) {
                slot.has_position = false;
                slot.sightings = 0;
                slot.vel_ema.setZero();
            }
        } else {
            // 超过 hold_time 还没出现，看看别的槽位有没有在出
            int best = -1;
            for (const int s : slot_priority_) {
                if (s == active_slot_) {
                    continue;
                }
                if (slots_[s].sightings >= min_sightings_ && seen_ago(s) <= hold_time_) {
                    best = s;
                    break;
                }
            }
            if (best >= 0) {
                RCLCPP_INFO(get_logger(), "switch outpost slot %d -> %d", active_slot_, best);
                active_slot_ = best;
                active_since_ = this->now();
            }
        }
    }

    // 还没锁定，等某个槽位攒够次数，按优先级挑
    if (active_slot_ < 0) {
        for (const int s : slot_priority_) {
            if (slots_[s].sightings >= min_sightings_ && seen_ago(s) <= lost_time_) {
                active_slot_ = s;
                active_since_ = this->now();
                RCLCPP_INFO(
                    get_logger(), "lock outpost slot %d, z=%.2f",
                    s, slot_z_[s]);
                break;
            }
        }
    }
    return active_slot_;
}

void OutpostTargetNode::publishTarget(
    const SlotState& slot, const rclcpp::Time& stamp, double now_sec) {
    auto_aim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = stamp;
    target_msg.header.frame_id = target_frame_;
    target_msg.tracking = true;
    target_msg.id = "outpost";
    // 板中心直接当打击点，radius 为 0，云台不需要再做偏移
    target_msg.armors_num = 1;
    target_msg.type = "small";
    target_msg.position.x = slot.pos_ema.x();
    target_msg.position.y = slot.pos_ema.y();
    target_msg.position.z = slot.pos_ema.z();
    // 板正对时速度有效，旋转期间用它做提前量；板转过去后清零
    const double ago = now_sec - (slot.last_raw_time);
    if (ago < 0.25) {
        target_msg.velocity.x = slot.vel_ema.x();
        target_msg.velocity.y = slot.vel_ema.y();
        target_msg.velocity.z = slot.vel_ema.z();
    }
    target_pub_->publish(target_msg);
}

void OutpostTargetNode::publishMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    const std::array<std::array<float, 3>, SLOT_NUM> colors{{
        {0.6f, 0.6f, 0.6f},
        {0.6f, 0.6f, 0.6f},
        {0.6f, 0.6f, 0.6f},
    }};

    for (int s = 0; s < SLOT_NUM; ++s) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = this->now();
        marker.ns = "outpost_slots";
        marker.id = s;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = colors[s][0];
        marker.color.g = colors[s][1];
        marker.color.b = colors[s][2];
        if (slots_[s].has_position) {
            marker.pose.position.x = slots_[s].pos_ema.x();
            marker.pose.position.y = slots_[s].pos_ema.y();
            marker.pose.position.z = slots_[s].pos_ema.z();
            if (s == active_slot_) {
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            }
        } else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
}

}  // namespace outpost_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(outpost_auto_aim::OutpostTargetNode)
