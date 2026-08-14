#include <cmath>

#include "buff_algo/common/math_utils.hpp"
#include "buff_algo/tracker/buff_tracker.hpp"

namespace buff_algo {

using namespace Eigen;
using Scalarf = Vector<float, 1>;

namespace {

constexpr float kBuffRollSingularityThreshold = 0.10f;
// 去掉90度附近的值
inline bool is_roll_near_singularity(const Buff& buff) {
    return std::abs(buff.R_center.y() - buff.translation.y()) < kBuffRollSingularityThreshold;
}

}  // namespace

/**********************************************************************************
***********************************    Buff    ************************************
***********************************************************************************/
Buff::Buff(const Pose3f& buff_pose) {
    translation = buff_pose.translation;
    rotation = buff_pose.rotation;
    R_center = rotation * Vector3f(0, 0, -consts::BUFF_RADIUS) + translation;
    yaw = 0;
    roll = std::atan2(translation.z() - R_center.z(), (R_center.y() - translation.y()));
}

/**********************************************************************************
******************************  SmallBuffObserver  ********************************
***********************************************************************************/

SmallBuffObserver::SmallBuffObserver(const cv::FileNode& fn) {
    SWITCH_BUFF_ANGLE = d2r(static_cast<float>(fn["switch_buff_angle"]));
    R_center_ = std::make_unique<EMAFilter<Eigen::Vector3f>>(fn["R_center_filter_ratio"]);
    kf_roll_ = std::make_unique<KF<1>>(fn["kf_roll"]);
    kf_yaw_ = std::make_unique<KF<1>>(fn["kf_yaw"]);
    reset();
}

void SmallBuffObserver::reset() {
    R_center_->reset();
    kf_roll_->reset();
    kf_yaw_->reset();
}

void SmallBuffObserver::initialize(const std::vector<Buff>& buffs) {
    reset();
    kf_roll_->initialize(Scalarf(buffs[0].roll));
    kf_yaw_->initialize(Scalarf(buffs[0].yaw));
    R_center_->initialize(buffs[0].R_center);
}

void SmallBuffObserver::predict(const float time_elapsed) {
    kf_roll_->predict(time_elapsed);
    kf_yaw_->predict(time_elapsed);
    if (kf_roll_->value().value() > M_PI) {
        kf_roll_->force_change_value(Scalarf(kf_roll_->value().value() - M_PI * 2));
    } else if (kf_roll_->value().value() < -M_PI) {
        kf_roll_->force_change_value(Scalarf(kf_roll_->value().value() + M_PI * 2));
    }
}

void SmallBuffObserver::update(const std::vector<Buff>& buffs) {
    const bool near_roll_singularity = is_roll_near_singularity(buffs[0]);
    const float buff_roll = static_cast<float>(rad_period_correction(buffs[0].roll));
    if (near_roll_singularity || std::abs(buff_roll - kf_roll_->value().value()) > SWITCH_BUFF_ANGLE) {
        kf_roll_->force_change_value(Scalarf(buff_roll));
    } else {
        kf_roll_->update(Scalarf(buff_roll));
    }
    kf_yaw_->update(Scalarf(buffs[0].yaw));
    R_center_->update(buffs[0].R_center);
}

void SmallBuffObserver::print_colored_status_info() const {
    const auto print_vec = [](const char* format, Vector3f vec) {
        std::printf(format, vec.x(), vec.y(), vec.z());
    };
    std::cout << termcolor::bold << "SmallBuff.RCenter   " << termcolor::reset;
    print_vec("[% 4.0f, % 4.0f, % 4.0f]\n", R_center_->value() * 100);
    std::cout << termcolor::bold << "SmallBuff.Theta     " << termcolor::reset;
    std::printf(
        "[% 4.0f] += [% 4.0f] (%s)\n",
        r2d(kf_roll_->value().value()),
        r2d(kf_roll_->derivative().value()),
        kf_roll_->derivative().value() > 0 ? "counterclockwise" : "clockwise"
    );
    std::cout << termcolor::bold << "kf_yaw     " << termcolor::reset;
    std::printf(
        "[% 4.0f] += [% 4.0f]\n",
        r2d(kf_yaw_->value().value()),
        r2d(kf_yaw_->derivative().value())
    );
    std::cout << std::endl;
}

BuffState SmallBuffObserver::get_state() const {
    BuffState state;
    state.r_center = R_center_->value();

    state.yaw = kf_yaw_->value().value();
    state.yaw_velocity = kf_yaw_->derivative().value();
    state.roll = kf_roll_->value().value();
    state.roll_velocity = kf_roll_->derivative().value();
    state.mode = 1; // Small Buff
    return state;
}

/**********************************************************************************
******************************  BigBuffObserver  ********************************
***********************************************************************************/

BigBuffObserver::BigBuffObserver(const cv::FileNode& fn) {
    SWITCH_BUFF_ANGLE = d2r(static_cast<float>(fn["switch_buff_angle"]));
    R_center_ = std::make_unique<EMAFilter<Eigen::Vector3f>>(fn["R_center_filter_ratio"]);
    kf_yaw_ = std::make_unique<KF<1>>(fn["kf_yaw"]);
    const auto ransac_fn = fn["ransac"];
    ransac_min_inliers_ = static_cast<int>(ransac_fn["min_inliers"]);
    ransac_max_abs_speed_ = static_cast<double>(ransac_fn["max_abs_speed"]);
    const int max_iterations = static_cast<int>(ransac_fn["max_iterations"]);
    const double threshold = static_cast<double>(ransac_fn["threshold"]);
    const double min_omega = static_cast<double>(ransac_fn["min_omega"]);
    const double max_omega = static_cast<double>(ransac_fn["max_omega"]);
    const double min_amplitude = static_cast<double>(ransac_fn["min_amplitude"]);
    const double max_amplitude = static_cast<double>(ransac_fn["max_amplitude"]);
    // 没配 fit_interval 时用默认 5
    if (!ransac_fn["fit_interval"].empty()) {
        ransac_fit_interval_ = static_cast<int>(ransac_fn["fit_interval"]);
    }
    ransac_ = std::make_unique<RansacSineFitter>(
        max_iterations,
        threshold,
        min_omega,
        max_omega,
        min_amplitude,
        max_amplitude
    );
    reset();
}

void BigBuffObserver::reset() {
    R_center_->reset();
    kf_yaw_->reset();
    ransac_->reset();
    prev_roll_ = 0.0f;
    prev_timestamp_ = 0.0;
    has_prev_roll_ = false;
    current_roll_ = 0.0f;
    current_roll_velocity_ = 0.0f;
    fit_frames_since_last_ = 0;
}

void BigBuffObserver::initialize(const std::vector<Buff>& buffs) {
    reset();
    kf_yaw_->initialize(Scalarf(buffs[0].yaw));
    R_center_->initialize(buffs[0].R_center);
    current_roll_ = buffs[0].roll;
}
bool BigBuffObserver::ransac_ready() const {
    return ransac_->best_result_.inliers >= ransac_min_inliers_;
}

void BigBuffObserver::predict(const float time_elapsed, const double timestamp) {
    kf_yaw_->predict(time_elapsed);

    if (ransac_ready()) {
        const double t_rel = ransac_->relative_time(timestamp);
        const double phi = ransac_->best_result_.omega * t_rel + ransac_->best_result_.phi;
        current_roll_velocity_ = static_cast<float>(
            ransac_->best_result_.A * std::sin(phi) + ransac_->best_result_.C
        );
        current_roll_ += current_roll_velocity_ * time_elapsed;
        if (current_roll_ > M_PI) current_roll_ -= 2.0f * M_PI;
        else if (current_roll_ < -M_PI) current_roll_ += 2.0f * M_PI;
    } else {
        current_roll_ += current_roll_velocity_ * time_elapsed;
        if (current_roll_ > M_PI) current_roll_ -= 2.0f * M_PI;
        else if (current_roll_ < -M_PI) current_roll_ += 2.0f * M_PI;
    }
}

void BigBuffObserver::update(const std::vector<Buff>& buffs, const double timestamp) {
    const bool near_roll_singularity = is_roll_near_singularity(buffs[0]);
    const float buff_roll = static_cast<float>(rad_period_correction(buffs[0].roll));
    kf_yaw_->update(Scalarf(buffs[0].yaw));
    R_center_->update(buffs[0].R_center);

    bool jumped = false;
    if (near_roll_singularity) {
        has_prev_roll_ = false;
    } else if (has_prev_roll_) {
        float roll_diff = buff_roll - prev_roll_;
        if (roll_diff > M_PI) roll_diff -= 2.0f * M_PI;
        else if (roll_diff < -M_PI) roll_diff += 2.0f * M_PI;

        if (std::abs(roll_diff) > SWITCH_BUFF_ANGLE) {
            jumped = true;
        } else {
            float dt = static_cast<float>(timestamp - prev_timestamp_);
            if (dt > 1e-6f) {
                current_roll_velocity_ = roll_diff / dt;
                if (std::abs(current_roll_velocity_) <= ransac_max_abs_speed_) {
                    ransac_->add_data(timestamp, current_roll_velocity_);
                    // 正弦参数变化很慢，隔几帧拟合一次，不用每帧都跑 200 次迭代
                    if (++fit_frames_since_last_ >= ransac_fit_interval_) {
                        fit_frames_since_last_ = 0;
                        ransac_->fit();
                    }
                }
            }
        }
    }
    current_roll_ = buff_roll;
    prev_roll_ = buff_roll;
    prev_timestamp_ = timestamp;
    has_prev_roll_ = !jumped;
}

void BigBuffObserver::print_colored_status_info() const {
    const auto print_vec = [](const char* format, Vector3f vec) {
        std::printf(format, vec.x(), vec.y(), vec.z());
    };
    std::cout << termcolor::bold << "BigBuff.RCenter   " << termcolor::reset;
    print_vec("[% 4.0f, % 4.0f, % 4.0f]\n", R_center_->value() * 100);
    std::cout << termcolor::bold << "BigBuff.Theta     " << termcolor::reset;
    std::printf(
        "[% 4.0f] += [% 4.0f] (%s) [RANSAC: %d inliers]\n",
        r2d(current_roll_),
        r2d(current_roll_velocity_),
        current_roll_velocity_ > 0 ? "counterclockwise" : "clockwise",
        ransac_->best_result_.inliers
    );
    std::cout << termcolor::bold << "kf_yaw     " << termcolor::reset;
    std::printf(
        "[% 4.0f] += [% 4.0f]",
        r2d(kf_yaw_->value().value()),
        r2d(kf_yaw_->derivative().value())
    );
    std::cout << std::endl;
}

BuffState BigBuffObserver::get_state() const {
    BuffState state;
    state.r_center = R_center_->value();

    state.yaw = kf_yaw_->value().value();
    state.yaw_velocity = kf_yaw_->derivative().value();

    state.roll = current_roll_;
    state.roll_velocity = current_roll_velocity_;
    if (ransac_ready()) {
        const double t_rel = ransac_->relative_time(prev_timestamp_);
        state.phi = static_cast<float>(ransac_->best_result_.omega * t_rel + ransac_->best_result_.phi);
        state.a = static_cast<float>(ransac_->best_result_.A);
        state.w = static_cast<float>(ransac_->best_result_.omega);
        state.b = static_cast<float>(ransac_->best_result_.C);
        state.roll_velocity = state.a * std::sin(state.phi) + state.b;
    } else {
        state.phi = 0.0f;
        state.a = 0.0f;
        state.w = 0.0f;
        state.b = 0.0f;
    }
    state.mode = 2; // Big Buff
    return state;
}

/**********************************************************************************
*********************************  BuffTracker  ***********************************
***********************************************************************************/

BuffTracker::BuffTracker(const cv::FileNode& fn) {
    TEMP_LOST_RETURN_FRAMES = static_cast<int>(fn["temp_lost_return_frames"]);
    ERR_QUEUE_SIZE_ = static_cast<int>(fn["err_queue_size"]);
    APPROXIMATE_FRAMERATE_ = static_cast<int>(fn["approximate_framerate"]);
    small_buff_status_ = std::make_unique<TrackerStatus>(
        fn["small_buff_status"],
        [this](StatusType from, StatusType to) { status_change_handler(from, to); },
        [this](StatusType curr) { status_remain_handler(curr); }
    );

    big_buff_status_ = std::make_unique<TrackerStatus>(
        fn["big_buff_status"],
        [this](StatusType from, StatusType to) { status_change_handler(from, to); },
        [this](StatusType curr) { status_remain_handler(curr); }
    );
    small_buff_observer_ = std::make_unique<SmallBuffObserver>(fn["small_buff_observer"]);
    big_buff_observer_ = std::make_unique<BigBuffObserver>(fn["big_buff_observer"]);

}

StatusType BuffTracker::status() const {
    if (mode_ == BuffMode::SMALL_BUFF) {
        return small_buff_status_->status();
    } else if (mode_ == BuffMode::BIG_BUFF) {
        return big_buff_status_->status();
    }
    return StatusType::LOST;
}

void BuffTracker::push(const Buff& buff) {
    pushed_buffs_.push_back(buff);
}

void BuffTracker::set_mode(const BuffMode mode) {
    if (mode_ != mode) {
        mode_ = mode;
        reset();
    }
}

void BuffTracker::reset() {
    pushed_buffs_.clear();
    small_buff_status_->reset();
    small_buff_observer_->reset();
    big_buff_status_->reset();
    big_buff_observer_->reset();
}

void BuffTracker::update(const double timestamp) {
    current_update_time_ = timestamp;
    bool is_valid = (pushed_buffs_.size() == 1);
    if (mode_ == BuffMode::SMALL_BUFF) {
        small_buff_status_->update(is_valid);
    } else if (mode_ == BuffMode::BIG_BUFF) {
        // 大符始终走 RANSAC 路线：
        // 1. 拟合达到最小内点数时立即进入 TRACKING；
        // 2. 否则在 CONVERGING 超过 300 帧后由状态机兜底进入 TRACKING。
        const bool ready_to_track = big_buff_observer_->ransac_ready();
        big_buff_status_->update(is_valid, ready_to_track);
    }
    prev_update_time_ = current_update_time_;
    pushed_buffs_.clear();
}

void BuffTracker::status_change_handler(StatusType from, StatusType to) {
    if (from == StatusType::LOST && to == StatusType::CONVERGING) {
        if (mode_ == BuffMode::SMALL_BUFF) {
            small_buff_observer_->initialize(pushed_buffs_);
        } else if (mode_ == BuffMode::BIG_BUFF) {
            big_buff_observer_->initialize(pushed_buffs_);
        }
    } else if ((from == StatusType::TEMP_LOST && to == StatusType::TRACKING) ||
               (from == StatusType::CONVERGING && to == StatusType::TRACKING)) {
        const float time_elapsed = static_cast<float>(current_update_time_ - prev_update_time_);
        if (mode_ == BuffMode::SMALL_BUFF) {
            small_buff_observer_->predict(time_elapsed);
            small_buff_observer_->update(pushed_buffs_);
        } else if (mode_ == BuffMode::BIG_BUFF) {
            big_buff_observer_->predict(time_elapsed, current_update_time_);
            big_buff_observer_->update(pushed_buffs_, current_update_time_);
        }
    }
}

void BuffTracker::status_remain_handler(StatusType current) {
    if (current != StatusType::LOST) {
        const float time_elapsed = static_cast<float>(current_update_time_ - prev_update_time_);
        if (mode_ == BuffMode::SMALL_BUFF) {
            small_buff_observer_->predict(time_elapsed);
        } else {
            big_buff_observer_->predict(time_elapsed, current_update_time_);

        }
    }
    if (current == StatusType::CONVERGING || current == StatusType::TRACKING) {
        if (mode_ == BuffMode::SMALL_BUFF) {
            small_buff_observer_->update(pushed_buffs_);
        } else if (mode_ == BuffMode::BIG_BUFF) {
            big_buff_observer_->update(pushed_buffs_, current_update_time_);
        }
    }
}

void BuffTracker::print_colored_status_info() const {
    if (mode_ == BuffMode::SMALL_BUFF) {
        small_buff_status_->print_colored_status_info();
        small_buff_observer_->print_colored_status_info();
        return;
    }
    if (mode_ == BuffMode::BIG_BUFF) {
        big_buff_status_->print_colored_status_info();
        big_buff_observer_->print_colored_status_info();
        return;
    }

    std::cout << termcolor::red << termcolor::bold << "NO_MODE     " << termcolor::reset << std::endl;

}

BuffState BuffTracker::get_state() const {
    if (mode_ == BuffMode::SMALL_BUFF) {
        return small_buff_observer_->get_state();
    } else if (mode_ == BuffMode::BIG_BUFF) {
        return big_buff_observer_->get_state();
    }
    return BuffState();
}

} // namespace buff_algo
