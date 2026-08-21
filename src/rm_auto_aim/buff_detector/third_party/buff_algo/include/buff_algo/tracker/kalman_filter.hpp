#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace buff_algo {

// 简单的匀速度模型卡尔曼滤波器。
// 状态向量为 [位置; 速度]，测量向量为位置，维度由模板参数指定。
template <unsigned MEASUREMENT_DIM>
class KF {
public:
    static constexpr unsigned STATE_DIM = 2 * MEASUREMENT_DIM;  // 状态维度：位置+速度

    explicit KF(const cv::FileNode& fn);
    void reset();
    void initialize(const Eigen::Vector<float, MEASUREMENT_DIM>& meas);
    void predict(float delta_t);
    void update(const Eigen::Vector<float, MEASUREMENT_DIM>& meas);
    Eigen::Vector<float, MEASUREMENT_DIM> value() const;
    Eigen::Vector<float, MEASUREMENT_DIM> derivative() const;
    void force_change_value(const Eigen::Vector<float, MEASUREMENT_DIM>& meas);

private:
    Eigen::Matrix<float, STATE_DIM, 1> x_; // 状态向量 [位置; 速度]
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> P_; // 状态协方差矩阵
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F_; // 状态转移矩阵
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q_; // 过程噪声协方差
    Eigen::Matrix<float, MEASUREMENT_DIM, MEASUREMENT_DIM> R_; // 观测噪声协方差
    Eigen::Matrix<float, MEASUREMENT_DIM, STATE_DIM> H_; // 观测矩阵
};

} // namespace buff_algo
