#pragma once

#include <Eigen/Dense>
#include <type_traits>

namespace buff_algo {

// 指数滑动平均滤波器，支持 Eigen 向量与四元数。
template<typename T>
class EMAFilter {
public:
    explicit EMAFilter(const double filter_ratio): filter_ratio_(filter_ratio) { reset(); }
    void initialize(const T& val) { value_ = val; }
    void force_change_value(const T& val) { value_ = val; }
    T value() const { return value_; };
    void reset() {
        if constexpr (std::is_base_of_v<Eigen::MatrixBase<std::decay_t<T>>, std::decay_t<T>>) { // 向量
            value_ = T::Zero();
        } else if constexpr (std::is_base_of_v<Eigen::QuaternionBase<std::decay_t<T>>, std::decay_t<T>>) { // 旋转
            value_ = T::Identity();
        } else {
            static_assert(sizeof(T) == 0, "unsupported type");
        }
    }
    void update(const T& val) {
        if constexpr (std::is_base_of_v<Eigen::MatrixBase<std::decay_t<T>>, std::decay_t<T>>) { // 向量
            value_ = filter_ratio_ * value_ + (1 - filter_ratio_) * val;
        } else if constexpr (std::is_base_of_v<Eigen::QuaternionBase<std::decay_t<T>>, std::decay_t<T>>) { // 旋转
            value_ = value_.slerp(1 - filter_ratio_, val);
        } else {
            static_assert(sizeof(T) == 0, "unsupported type");
        }
    }

private:
    T value_;
    const double filter_ratio_ = 0; // 介于0-1之间，越大越稳定
};

} // namespace buff_algo
