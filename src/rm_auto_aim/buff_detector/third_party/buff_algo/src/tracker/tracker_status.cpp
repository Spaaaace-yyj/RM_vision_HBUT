#include "buff_algo/tracker/tracker_status.hpp"

namespace buff_algo {

TrackerStatus::TrackerStatus(
    const cv::FileNode& fn,
    std::function<void(StatusType from, StatusType to)> status_change_handler,
    std::function<void(StatusType current)> status_remain_handler
): status_change_handler(status_change_handler), status_remain_handler(status_remain_handler) {
    MAX_TEMP_LOST_FRAMES = static_cast<int>(fn["max_temp_lost_frames"]);
    MAX_CONVERGING_FRAMES = static_cast<int>(fn["max_converging_frames"]);
    reset();
}

TrackerStatus::TrackerStatus(
    const unsigned max_temp_lost_frames,
    const unsigned max_converging_frames,
    std::function<void(StatusType from, StatusType to)> status_change_handler,
    std::function<void(StatusType current)> status_remain_handler
):
    status_change_handler(status_change_handler),
    status_remain_handler(status_remain_handler) {
    MAX_TEMP_LOST_FRAMES = max_temp_lost_frames;
    MAX_CONVERGING_FRAMES = max_converging_frames;
    reset();
}

void TrackerStatus::set_next_status(StatusType status) {
    if (status_ != status) {
        status_change_handler(status_, status);
        status_ = status;
        current_status_frames_ = 0;
    } else {
        status_remain_handler(status_);
        current_status_frames_++;
    }
}

void TrackerStatus::reset() { status_ = StatusType::LOST; current_status_frames_ = 0; }
StatusType TrackerStatus::status() const { return status_; }
unsigned TrackerStatus::current_status_frames() const { return current_status_frames_; }

void TrackerStatus::update(bool is_valid) {
    update(is_valid, true);
}

void TrackerStatus::update(bool is_valid, bool ready_to_track) {
    using TS = StatusType;
    TS next_status = TS::LOST;
    if (is_valid) {
        switch (status_) {
            case TS::LOST: next_status = TS::CONVERGING; break;
            case TS::TEMP_LOST: next_status = TS::TRACKING; break;
            case TS::CONVERGING: {
                if (ready_to_track || current_status_frames_ > MAX_CONVERGING_FRAMES) {
                    next_status = TS::TRACKING;
                }
                else next_status = TS::CONVERGING;
                break;
            }
            case TS::TRACKING: next_status = TS::TRACKING; break;
        }
    } else {
        switch (status_) {
            case TS::LOST: next_status = TS::LOST; break;
            case TS::TEMP_LOST: {
                if (current_status_frames_ > MAX_TEMP_LOST_FRAMES) next_status = TS::LOST;
                else next_status = TS::TEMP_LOST;
                break;
            }
            case TS::CONVERGING: next_status = TS::LOST; break;
            case TS::TRACKING: next_status = TS::TEMP_LOST; break;
        }
    }
    set_next_status(next_status);
}

void TrackerStatus::print_colored_status_info() const {
    switch(status_) {
        case StatusType::TRACKING: {
            std::cout << termcolor::green << termcolor::bold << "TRACKING    " << termcolor::reset;
            break;
        }
        case StatusType::CONVERGING: {
            std::cout << termcolor::white << termcolor::bold << "CONVERGING  " << termcolor::reset;
            break;
        }
        case StatusType::TEMP_LOST: {
            std::cout << termcolor::red << termcolor::bold << "TEMP_LOST   " << termcolor::reset;
            break;
        }
        case StatusType::LOST: {
            std::cout << termcolor::red << termcolor::bold << "LOST        " << termcolor::reset;
            break;
        }
    }
    std::cout << current_status_frames_ << std::endl;
}

} // namespace buff_algo
