#pragma once

// OutpostTargetNode：前哨站目标节点。
// 2026 新规前哨站是三块小装甲板绕竖直轴 120° 均匀分布、高低排布，整体匀速旋转，
// 打哪一块都扣同一个前哨站的血。三块板共轴共半径，只有高度不同，
// 所以用一个共享的圆周运动 EKF 建模（参照 armor_tracker 的 EKF 写法）：
//   状态 = [轴心x, 轴心vx, 轴心y, 轴心vy, 半径, 相位, 角速度, 板0高, 板1高, 板2高]
// 观测 = 任意一块板的位置，按高度归类到对应板索引后顺序更新。
// 板转到背面时 EKF 继续外推，能预测每块板什么时候转回正对位置，
// 空窗期用定时器持续发布目标，云台提前转到下一块正对板的位置。

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

#include "armor_tracker/extended_kalman_filter.hpp"

namespace outpost_auto_aim {

// EKF 状态索引，6 维。前哨站轴心是静止的，不需要速度状态，
// 半径 r 是机械固定值不进状态，用参数；转速也不进状态：
// 规则书给定匀速 0.8π rad/s，方向第一窗口自动测，停转检测到就把转速置 0
enum StateIdx : int {
    ST_XC = 0,    // 旋转轴心 x
    ST_YC = 1,    // 旋转轴心 y
    ST_YAW = 2,   // 模块相位，板 0 相对 x 轴的方位角
    ST_Z0 = 3,    // 板 0 高度
    ST_Z1 = 4,
    ST_Z2 = 5,
    ST_SIZE = 6,
};

// 跟踪状态，和 armor_tracker 一致
enum TrackState : int {
    TS_LOST = 0,
    TS_DETECTING = 1,
    TS_TRACKING = 2,
    TS_TEMP_LOST = 3,
};

class OutpostTargetNode : public rclcpp::Node {
public:
    explicit OutpostTargetNode(const rclcpp::NodeOptions& options);

private:
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
    void timerTick();

    // 相机系板位姿变换到 target_frame_，离线调试时两个 frame 相同则跳过
    bool transformToTarget(
        const geometry_msgs::msg::Pose& in, geometry_msgs::msg::Pose& out,
        const std::string& in_frame, const rclcpp::Time& stamp);

    // 按高度把检测归到板索引，-1 表示不在任何高度带
    int classifyPlate(double z) const;

    // 首次观测初始化 EKF，之后每帧 predict + 观测更新
    void initEKF(int plate, const Eigen::Vector3d& pos);
    bool updateOnePlate(int plate, const Eigen::Vector3d& pos, double now_sec);

    // 状态机，matched 表示本帧有通过门控的观测
    void updateStateMachine(double now_sec, bool matched);

    // 选当前最接近正对相机的板，带滞回防止边界抖动
    int chooseOutputPlate() const;

    double plateAngle(int i) const;
    Eigen::Vector3d platePosition(int i) const;

    void publishTarget(double now_sec);
    void publishMarkers();

    // ---- EKF ----
    rm_auto_aim::ExtendedKalmanFilter ekf_;
    bool ekf_inited_ = false;
    double dt_ = 0.02;             // 帧间隔，predict 用
    double last_predict_time_ = 0.0;

    // ---- 状态机 ----
    int track_state_ = TS_LOST;
    int detect_count_ = 0;
    double last_match_time_ = 0.0;  // 最近一次成功观测的时间
    double c_yaw_ref_ = 0.0;        // 相机方向角，每次观测时用正对板相位更新
    int output_plate_ = 0;          // 当前输出板，滞回用

    // ---- 转速 ----
    // 规则转速 0.8π rad/s，方向当局固定：rotate_direction 0=第一窗口自动测，
    // 1/-1=手动指定；停转检测到相位不再变化就把转速置 0
    double v_yaw_eff_ = 0.8 * M_PI;
    double phase_dot_sum_ = 0.0;
    int phase_dot_count_ = 0;
    int stop_frames_ = 0;
    bool has_last_obs_ = false;     // 相位差分用的上一帧观测
    double last_obs_phase_ = 0.0;
    double last_obs_time_ = 0.0;

    // ---- 订阅 / 发布 ----
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- tf ----
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    // ---- 参数 ----
    std::string target_frame_;
    std::string camera_frame_;
    std::array<double, 3> slot_z_ = {1.10, 1.27, 1.45};  // 三块板离地高度
    double z_tol_ = 0.08;           // 高度带半宽
    double r_initial_ = 0.28;       // 旋转半径，机械固定值，只进观测模型不更新
    int rotate_direction_ = 0;      // 旋转方向，0=第一窗口自动测，1/-1=手动指定
    double gate_dist_ = 1.0;        // 观测门控，预测位置离观测太远就拒绝
    double temp_lost_time_ = 1.5;   // 观测中断这么久进 TEMP_LOST
    double lost_time_ = 5.0;        // 观测中断这么久回 LOST
    int tracking_thres_ = 5;        // 连续观测这么多帧进 TRACKING
    double s2q_xyz_ = 100.0;        // EKF 过程噪声，参照 armor_tracker
    double s2q_yaw_ = 1.0;
    double r_meas_xy_ = 0.05;       // 观测噪声
    double r_meas_z_ = 0.03;
    double outpost_x_ = 0.0;        // 前哨站位置门，半径 <= 0 表示关闭
    double outpost_y_ = 0.0;
    double outpost_radius_ = 0.0;
    bool accept_any_small_ = false; // 没识别成 outpost 时兜底收小装甲板
    bool debug_ = false;
};

}  // namespace outpost_auto_aim
