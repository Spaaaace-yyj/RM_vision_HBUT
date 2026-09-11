#include "buff_algo/predictor/buff_predictor.hpp"
#include <algorithm>
#include <cmath>
#include "buff_algo/common/math_utils.hpp"

namespace buff_algo {

using namespace Eigen;

namespace {

constexpr float kModelEps = 1e-6f;

bool has_big_buff_model(const BuffState& state) {
    return state.mode == 2 && (std::abs(state.w) > kModelEps || std::abs(state.a) > kModelEps);
}

}  // namespace

BuffPredictor::BuffPredictor(const cv::FileNode& fn) { (void)fn; }


void BuffPredictor::set_state(
    const BuffState& state,
    const double state_timestamp,
    const double predict_base_timestamp
) {
    state_ = state;
    has_state_ = true;
    state_age_ = static_cast<float>(std::max(0.0, predict_base_timestamp - state_timestamp));
}

float BuffPredictor::predict_roll(const float dt) const {
    const float pred_dt = std::max(0.0f, dt);
    double pred_roll = static_cast<double>(state_.roll) +
        static_cast<double>(state_.roll_velocity) * pred_dt;

    if (has_big_buff_model(state_)) {
        const double phi0 = static_cast<double>(state_.phi);
        const double a = static_cast<double>(state_.a);
        const double omega = static_cast<double>(state_.w);
        const double b = static_cast<double>(state_.b);
        if (std::abs(omega) > kModelEps) {
            pred_roll = static_cast<double>(state_.roll) +
                b * pred_dt +
                a * (std::cos(phi0) - std::cos(phi0 + omega * pred_dt)) / omega;
        } else {
            pred_roll = static_cast<double>(state_.roll) +
                (a * std::sin(phi0) + b) * pred_dt;
        }
    }

    return static_cast<float>(rad_period_correction(pred_roll));
}

Eigen::Vector3f BuffPredictor::predict_position(float dt) {
    if (!has_state_) return Vector3f::Zero();

    const float prediction_horizon = state_age_ + std::max(0.0f, dt);

    Vector3f r_center = state_.r_center;
    const float pred_roll = predict_roll(prediction_horizon);

    Vector3f leaf_position;
    leaf_position.x() = r_center.x();
    leaf_position.y() = r_center.y() - consts::BUFF_RADIUS * std::cos(pred_roll);
    leaf_position.z() = r_center.z() + consts::BUFF_RADIUS * std::sin(pred_roll);

    return leaf_position;
}

} // namespace buff_algo
