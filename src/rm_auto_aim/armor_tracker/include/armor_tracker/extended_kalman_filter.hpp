// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <angles/angles.h>

namespace rm_auto_aim
{
    class ExtendedKalmanFilter
    {
    public:
        ExtendedKalmanFilter() = default;

        using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
        using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
        using VoidMatFunc = std::function<Eigen::MatrixXd()>;
        using ResidualFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

        explicit ExtendedKalmanFilter(
            const VecVecFunc& f, const VecVecFunc& h, const VecMatFunc& j_f, const VecMatFunc& j_h,
            const VoidMatFunc& u_q, const VecMatFunc& u_r, const Eigen::MatrixXd& P0);

        // Set the initial state
        void setState(const Eigen::VectorXd& x0);

        // Set the initial state and covariance. Re-init EKF 时建议用这个接口。
        void setState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

        // Compute a predicted state
        Eigen::MatrixXd predict();

        // Update the estimated state based on the default measurement model
        Eigen::MatrixXd update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R_meas);

        // Update with a per-measurement nonlinear model. 11D 整车 EKF 使用这个接口。
        Eigen::MatrixXd update(
            const Eigen::VectorXd& z,
            const Eigen::MatrixXd& H_custom,
            const Eigen::MatrixXd& R_meas,
            const VecVecFunc& h_custom,
            const ResidualFunc& z_subtract);

        double mahalanobisDistance(
            const Eigen::VectorXd& z,
            const Eigen::MatrixXd& R_meas);

        double mahalanobisDistance(
            const Eigen::VectorXd& z,
            const Eigen::MatrixXd& H_custom,
            const Eigen::MatrixXd& R_meas,
            const VecVecFunc& h_custom,
            const ResidualFunc& z_subtract) const;

        Eigen::VectorXd innovation(
            const Eigen::VectorXd& z) const;

        Eigen::MatrixXd innovationCovariance(
            const Eigen::MatrixXd& R_meas);

        const Eigen::VectorXd& prioriState() const { return x_pri; }
        const Eigen::VectorXd& posteriorState() const { return x_post; }

    private:
        // Process nonlinear vector function
        VecVecFunc f; //状态转移函数
        // Observation nonlinear vector function
        VecVecFunc h; //观测函数
        // Jacobian of f()
        VecMatFunc jacobian_f;
        Eigen::MatrixXd F; //状态函数雅可比矩阵
        // Jacobian of h()
        VecMatFunc jacobian_h;
        Eigen::MatrixXd H;
        // Process noise covariance matrix
        VoidMatFunc update_Q;
        Eigen::MatrixXd Q;
        // Measurement noise covariance matrix
        VecMatFunc update_R;
        Eigen::MatrixXd R;

        // Priori error estimate covariance matrix
        Eigen::MatrixXd P_pri;
        // Posteriori error estimate covariance matrix
        Eigen::MatrixXd P_post;

        // Kalman gain
        Eigen::MatrixXd K;

        // System dimensions
        int n = 0;

        // N-size identity
        Eigen::MatrixXd I;

        // Priori state
        Eigen::VectorXd x_pri;
        // Posteriori state
        Eigen::VectorXd x_post;
    };
} // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_