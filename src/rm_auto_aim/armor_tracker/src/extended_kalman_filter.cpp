// // Copyright 2022 Chen Jun

// #include "armor_tracker/extended_kalman_filter.hpp"

// namespace rm_auto_aim
// {
// ExtendedKalmanFilter::ExtendedKalmanFilter(
//   const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
//   const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
// : f(f),
//   h(h),
//   jacobian_f(j_f),
//   jacobian_h(j_h),
//   update_Q(u_q),
//   update_R(u_r),
//   P_post(P0),
//   n(P0.rows()),
//   I(Eigen::MatrixXd::Identity(n, n)),
//   x_pri(n),
//   x_post(n)
// {
// }

// void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

// Eigen::MatrixXd ExtendedKalmanFilter::predict()
// {
//   F = jacobian_f(x_post), Q = update_Q();

//   x_pri = f(x_post);
//   P_pri = F * P_post * F.transpose() + Q;

//   // handle the case when there will be no measurement before the next predict
//   x_post = x_pri;
//   P_post = P_pri;

//   return x_pri;
// }

// Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
// {
//   H = jacobian_h(x_pri), R = update_R(z);

//   K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
//   x_post = x_pri + K * (z - h(x_pri));
//   P_post = (I - K * H) * P_pri;

//   return x_post;
// }

// }  // namespace rm_auto_aim

#include "armor_tracker/extended_kalman_filter.hpp"
#include "angles/angles.h"

namespace rm_auto_aim
{
    double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    ExtendedKalmanFilter::ExtendedKalmanFilter(
        const VecVecFunc& f, const VecVecFunc& h, const VecMatFunc& j_f, const VecMatFunc& j_h,
        const VoidMatFunc& u_q, const VecMatFunc& u_r, const Eigen::MatrixXd& P0)
        : f(f),
          h(h),
          jacobian_f(j_f),
          jacobian_h(j_h),
          update_Q(u_q),
          update_R(u_r),
          P_post(P0),
          n(P0.rows()),
          I(Eigen::MatrixXd::Identity(n, n)),
          x_pri(n),
          x_post(n)
    {
    }

    void ExtendedKalmanFilter::setState(const Eigen::VectorXd& x0) { x_post = x0; }

    Eigen::MatrixXd ExtendedKalmanFilter::predict()
    {
        F = jacobian_f(x_post), Q = update_Q();

        x_pri = f(x_post);
        P_pri = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict
        x_post = x_pri;
        P_post = P_pri;

        return x_pri;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R_meas)
    {
        H = jacobian_h(x_pri);
        // R = update_R(z);
        R = R_meas;

        Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(S);
        Eigen::MatrixXd K = P_pri * H.transpose() * qr.solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));

        Eigen::VectorXd innovation = z - h(x_pri);
        // innovation(3) = angles::shortest_angular_distance(
        //     h(x_pri)(3), z(3)
        // );


        Eigen::VectorXd update_term = K * innovation;
        x_post = x_pri + update_term;

        Eigen::MatrixXd KH = K * H;
        P_post = (I - KH) * P_pri;

        return x_post;
    }

    Eigen::VectorXd ExtendedKalmanFilter::innovation(
        const Eigen::VectorXd& z) const
    {
        Eigen::VectorXd z_pred = h(x_pri);
        Eigen::VectorXd v = z - z_pred;

        // 当前观测是 [x, y, z, yaw]，所以 yaw 在第 3 维
        if (v.size() >= 4)
        {
            v(3) = angles::shortest_angular_distance(z_pred(3), z(3));
        }

        return v;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::innovationCovariance(
        const Eigen::MatrixXd& R_meas)
    {
        Eigen::MatrixXd H_tmp = jacobian_h(x_pri);
        return H_tmp * P_pri * H_tmp.transpose() + R_meas;
    }

    double ExtendedKalmanFilter::mahalanobisDistance(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& R_meas)
    {
        Eigen::MatrixXd H_tmp = jacobian_h(x_pri);
        Eigen::VectorXd z_pred = h(x_pri);

        Eigen::VectorXd v = z - z_pred;
        if (v.size() >= 4)
        {
            v(3) = angles::shortest_angular_distance(z_pred(3), z(3));
        }

        Eigen::MatrixXd S = H_tmp * P_pri * H_tmp.transpose() + R_meas;

        // 不要直接 inverse()，用分解更稳定
        Eigen::LDLT<Eigen::MatrixXd> ldlt(S);

        if (ldlt.info() != Eigen::Success)
        {
            return std::numeric_limits<double>::infinity();
        }

        double d2 = v.transpose() * ldlt.solve(v);

        if (!std::isfinite(d2))
        {
            return std::numeric_limits<double>::infinity();
        }

        return d2;
    }
} // namespace rm_auto_aim
