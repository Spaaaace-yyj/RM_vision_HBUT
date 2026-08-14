#pragma once

// OutpostTargetNode：前哨站目标选择节点。
// 2026 新规前哨站是旋转模块上高低排布的三块小装甲板，绕竖直轴匀速转，
// 打哪一块都扣同一个前哨站的血。节点订阅装甲板检测结果，
// 按高度把板分到低/中/高三个槽位，锁定其中一块持续输出给云台。

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

namespace outpost_auto_aim {

// 三个槽位：低/中/高，对应三块板的固定高度
enum SlotId : int {
    SLOT_LOW = 0,
    SLOT_MID = 1,
    SLOT_HIGH = 2,
    SLOT_NUM = 3,
};

class OutpostTargetNode : public rclcpp::Node {
public:
    explicit OutpostTargetNode(const rclcpp::NodeOptions& options);

private:
    // ---- 槽位状态 ----
    struct SlotState {
        bool has_position = false;
        Eigen::Vector3f pos_ema;    // 平滑后的板中心
        Eigen::Vector3f last_raw;   // 上一帧原始位置，用于差分速度
        double last_raw_time = 0.0;
        Eigen::Vector3f vel_ema;    // 平滑切向速度
        rclcpp::Time last_seen;     // 最近一次见到的时间
        int sightings = 0;          // 累计检测次数
    };
    std::array<SlotState, SLOT_NUM> slots_;

    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);

    // 相机系板位姿变换到 target_frame_，离线调试时两个 frame 相同则跳过
    bool transformToTarget(
        const geometry_msgs::msg::Pose& in, geometry_msgs::msg::Pose& out,
        const std::string& in_frame, const rclcpp::Time& stamp);

    // 按高度把检测分到槽位，更新槽位的平滑位置和速度
    void updateSlots(
        const auto_aim_interfaces::msg::Armors::SharedPtr msg,
        const std::vector<geometry_msgs::msg::Pose>& poses,
        const rclcpp::Time& seen_time,
        double now_sec);

    // 决定当前锁定哪个槽位
    int decideSlot(double now_sec);

    void publishTarget(const SlotState& slot, const rclcpp::Time& stamp, double now_sec);
    void publishMarkers();

    // ---- 订阅 / 发布 ----
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // ---- tf ----
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    int active_slot_ = -1;          // 当前锁定槽位，-1 表示还没锁定
    rclcpp::Time active_since_;     // 锁定开始时间，打印状态用

    // ---- 参数 ----
    std::string target_frame_;
    std::string camera_frame_;
    std::array<double, SLOT_NUM> slot_z_ = {1.10, 1.27, 1.45};  // 三块板离地高度
    double z_tol_ = 0.08;           // 高度带半宽
    std::vector<int> slot_priority_ = {SLOT_LOW, SLOT_MID, SLOT_HIGH};
    double hold_time_ = 3.0;        // 锁定后这么久没再见到就考虑换槽位
    double lost_time_ = 5.0;        // 全丢这么久就清零输出
    int min_sightings_ = 2;         // 锁定一个槽位至少要见到几次
    double pos_alpha_ = 0.5;        // 位置 EMA 系数
    double vel_alpha_ = 0.3;        // 速度 EMA 系数
    double outpost_x_ = 0.0;        // 前哨站位置门，半径 <= 0 表示关闭
    double outpost_y_ = 0.0;
    double outpost_radius_ = 0.0;
    bool accept_any_small_ = false; // 没识别成 outpost 时兜底收小装甲板
    bool debug_ = false;
};

}  // namespace outpost_auto_aim
