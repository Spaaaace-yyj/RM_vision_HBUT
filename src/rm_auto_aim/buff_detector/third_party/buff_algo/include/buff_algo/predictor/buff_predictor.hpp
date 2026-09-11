#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "buff_algo/common/types.hpp"

namespace buff_algo {

// 根据能量机关的运动状态估计结果外推未来位置（用于弹道解算的迭代收敛）。
class BuffPredictor {
public:
    explicit BuffPredictor(const cv::FileNode& fn);

    void set_state(
        const BuffState& state,
        double state_timestamp,
        double predict_base_timestamp
    );

    // 预测从"预测基准时刻"起 dt 秒后的目标位置。
    // 内部会自动补偿状态数据的滞后时间(state_age_)。
    Eigen::Vector3f predict_position(float dt);

private:
    float predict_roll(float dt) const;

    BuffState state_;
    bool has_state_ = false;
    float state_age_ = 0.0f;
};

} // namespace buff_algo
