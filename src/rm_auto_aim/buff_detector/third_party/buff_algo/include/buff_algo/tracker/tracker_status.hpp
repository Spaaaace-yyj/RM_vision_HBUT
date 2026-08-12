#pragma once

#include <opencv2/opencv.hpp>
#include "buff_algo/common/termcolor/termcolor.hpp"
#include <functional>

namespace buff_algo {

enum class StatusType : int {
    CONVERGING = 0,
    TRACKING = 1,
    TEMP_LOST = 2,
    LOST = 3
};

// 通用的跟踪状态机：LOST -> CONVERGING -> TRACKING -> TEMP_LOST -> (TRACKING | LOST)
class TrackerStatus {
public:
    explicit TrackerStatus(
        const cv::FileNode& fn,
        std::function<void(StatusType from, StatusType to)> status_change_handler =
            [](StatusType from, StatusType to) { (void)from, (void)to; },
        std::function<void(StatusType current)> status_remain_handler =
            [](StatusType current) { (void)current; }
    );
    explicit TrackerStatus(
        const unsigned max_temp_lost_frames,
        const unsigned max_converging_frames,
        std::function<void(StatusType from, StatusType to)> status_change_handler =
            [](StatusType from, StatusType to) { (void)from, (void)to; },
        std::function<void(StatusType current)> status_remain_handler =
            [](StatusType current) { (void)current; }
    );

    void reset();
    void update(bool is_valid);
    void update(bool is_valid, bool ready_to_track);
    StatusType status() const;
    unsigned current_status_frames() const;
    void print_colored_status_info() const;

private:
    void set_next_status(StatusType status);

    // 两个handler会且仅会被调用其中一个
    const std::function<void(StatusType from, StatusType to)> status_change_handler; // 状态转移时调用
    const std::function<void(StatusType current)> status_remain_handler; // 状态持续时调用
    unsigned MAX_TEMP_LOST_FRAMES, MAX_CONVERGING_FRAMES;

    StatusType status_ = StatusType::LOST;
    unsigned current_status_frames_ = 0;
};

} // namespace buff_algo
