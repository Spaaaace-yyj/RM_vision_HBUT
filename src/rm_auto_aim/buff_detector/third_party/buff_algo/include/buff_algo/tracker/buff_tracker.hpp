#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "buff_algo/common/types.hpp"
#include "buff_algo/common/ema_filter.hpp"

#include "buff_algo/tracker/kalman_filter.hpp"
#include "buff_algo/tracker/ransac_filter.hpp"
#include "buff_algo/tracker/tracker_status.hpp"

namespace buff_algo {

// 单帧能量机关扇叶的观测量：从 PnP 解算得到的位姿计算出 R 字中心位置、roll 角等。
struct Buff {
    explicit Buff(const Pose3f& buff_pose);

    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    Eigen::Vector3f R_center;
    float yaw, roll;
};

// 小符观测器：小符扇叶转速固定，只需简单的匀速卡尔曼滤波跟踪 roll/yaw。
class SmallBuffObserver {
public:
    explicit SmallBuffObserver(const cv::FileNode& fn);
    void reset();
    void initialize(const std::vector<Buff>& buffs);
    void predict(const float time_elapsed);
    void update(const std::vector<Buff>& buffs);

    void print_colored_status_info() const;
    BuffState get_state() const;

private:
    const unsigned BUFFS_COUNT = 5;
    float SWITCH_BUFF_ANGLE;

    std::unique_ptr<KF<1>> kf_roll_;
    std::unique_ptr<KF<1>> kf_yaw_;
    std::unique_ptr<EMAFilter<Eigen::Vector3f>> R_center_;
};

// 大符观测器：大符转速服从正弦模型，使用 RANSAC 拟合正弦参数，
// 拟合达到最小内点数之前使用简易差分速度跟踪 roll。
class BigBuffObserver {
public:
    explicit BigBuffObserver(const cv::FileNode& fn);
    void reset();
    void initialize(const std::vector<Buff>& buffs);
    void predict(const float time_elapsed, const double timestamp);
    void update(const std::vector<Buff>& buffs, const double timestamp);

    void print_colored_status_info() const;
    BuffState get_state() const;
    bool ransac_ready() const;

private:
    const unsigned BUFFS_COUNT = 5;
    float SWITCH_BUFF_ANGLE;

    std::unique_ptr<KF<1>> kf_yaw_;
    std::unique_ptr<EMAFilter<Eigen::Vector3f>> R_center_;

    // RANSAC 初始化阶段
    int ransac_min_inliers_;
    double ransac_max_abs_speed_;
    std::unique_ptr<RansacSineFitter> ransac_;

    // RANSAC 不是每帧都重拟合，隔几帧跑一次能省不少 CPU
    int ransac_fit_interval_ = 5;
    int fit_frames_since_last_ = 0;

    // 差分速度计算
    float prev_roll_ = 0.0f;
    double prev_timestamp_ = 0.0;
    bool has_prev_roll_ = false;

    // RANSAC 阶段的简易 roll 跟踪
    float current_roll_ = 0.0f;
    float current_roll_velocity_ = 0.0f;
};

// 能量机关跟踪器：根据模式（小符/大符）切换观测器，并驱动跟踪状态机。
class BuffTracker {
public:
    explicit BuffTracker(const cv::FileNode& fn);
    void push(const Buff& buff);
    void update(const double timestamp);
    void reset();
    void set_mode(BuffMode mode);
    StatusType status() const;

    void print_colored_status_info() const;

    BuffState get_state() const;

private:
    void status_change_handler(StatusType from, StatusType to);
    void status_remain_handler(StatusType current);

    std::unique_ptr<TrackerStatus> small_buff_status_;
    std::unique_ptr<TrackerStatus> big_buff_status_;
    std::unique_ptr<SmallBuffObserver> small_buff_observer_;
    std::unique_ptr<BigBuffObserver> big_buff_observer_;

    BuffMode mode_ = BuffMode::NONE;
    double current_update_time_ = 0.0;
    double prev_update_time_ = 0.0;
    std::vector<Buff> pushed_buffs_;

    int TEMP_LOST_RETURN_FRAMES;
    int ERR_QUEUE_SIZE_;
    int APPROXIMATE_FRAMERATE_;
};

} // namespace buff_algo
