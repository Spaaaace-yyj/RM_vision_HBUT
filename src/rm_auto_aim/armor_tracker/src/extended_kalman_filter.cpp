#include "armor_tracker/extended_kalman_filter.hpp"
#include "angles/angles.h"

namespace rm_auto_aim
{
    static double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
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
          x_pri(Eigen::VectorXd::Zero(n)),
          x_post(Eigen::VectorXd::Zero(n))
    {
    }

    void ExtendedKalmanFilter::setState(const Eigen::VectorXd& x0)
    {
        x_post = x0;
        if (n == 0)
        {
            n = static_cast<int>(x0.rows());
            I = Eigen::MatrixXd::Identity(n, n);
            P_post = Eigen::MatrixXd::Identity(n, n);
            x_pri = Eigen::VectorXd::Zero(n);
        }
        if (x_post.size() > 6)
        {
            x_post(6) = wrapAngle(x_post(6));
        }
    }

    void ExtendedKalmanFilter::setState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
    {
        n = static_cast<int>(x0.rows());
        I = Eigen::MatrixXd::Identity(n, n);
        x_pri = Eigen::VectorXd::Zero(n);
        x_post = x0;
        P_pri = P0;
        P_post = P0;
        if (x_post.size() > 6)
        {
            x_post(6) = wrapAngle(x_post(6));
        }
    }

    Eigen::MatrixXd ExtendedKalmanFilter::predict()
    {
        F = jacobian_f(x_post);
        Q = update_Q();

        x_pri = f(x_post);
        if (x_pri.size() > 6)
        {
            x_pri(6) = wrapAngle(x_pri(6));
        }
        P_pri = F * P_post * F.transpose() + Q;
        P_pri = 0.5 * (P_pri + P_pri.transpose());

        // handle the case when there will be no measurement before the next predict
        x_post = x_pri;
        P_post = P_pri;

        return x_pri;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R_meas)
    {
        auto default_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            Eigen::VectorXd c = a - b;
            if (c.size() >= 4)
            {
                c(3) = angles::shortest_angular_distance(b(3), a(3));
            }
            return c;
        };
        return update(z, jacobian_h(x_pri), R_meas, h, default_subtract);
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H_custom,
        const Eigen::MatrixXd& R_meas,
        const VecVecFunc& h_custom,
        const ResidualFunc& z_subtract)
    {
        H = H_custom;
        R = R_meas;

        Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
        if (ldlt.info() != Eigen::Success)
        {
            return x_post;
        }

        K = P_pri * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.rows()));

        Eigen::VectorXd innovation = z_subtract(z, h_custom(x_pri));
        Eigen::VectorXd update_term = K * innovation;
        x_post = x_pri + update_term;
        if (x_post.size() > 6)
        {
            x_post(6) = wrapAngle(x_post(6));
        }

        // Joseph form，比 (I-KH)P 数值稳定一些。
        Eigen::MatrixXd IKH = I - K * H;
        P_post = IKH * P_pri * IKH.transpose() + K * R * K.transpose();
        P_post = 0.5 * (P_post + P_post.transpose());

        // 允许同一帧内对多块同目标装甲板做顺序更新。
        x_pri = x_post;
        P_pri = P_post;

        return x_post;
    }

    Eigen::VectorXd ExtendedKalmanFilter::innovation(const Eigen::VectorXd& z) const
    {
        Eigen::VectorXd z_pred = h(x_pri);
        Eigen::VectorXd v = z - z_pred;

        if (v.size() >= 4)
        {
            v(3) = angles::shortest_angular_distance(z_pred(3), z(3));
        }

        return v;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::innovationCovariance(const Eigen::MatrixXd& R_meas)
    {
        Eigen::MatrixXd H_tmp = jacobian_h(x_pri);
        return H_tmp * P_pri * H_tmp.transpose() + R_meas;
    }

    double ExtendedKalmanFilter::mahalanobisDistance(const Eigen::VectorXd& z, const Eigen::MatrixXd& R_meas)
    {
        auto default_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            Eigen::VectorXd c = a - b;
            if (c.size() >= 4)
            {
                c(3) = angles::shortest_angular_distance(b(3), a(3));
            }
            return c;
        };
        return mahalanobisDistance(z, jacobian_h(x_pri), R_meas, h, default_subtract);
    }

    double ExtendedKalmanFilter::mahalanobisDistance(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H_custom,
        const Eigen::MatrixXd& R_meas,
        const VecVecFunc& h_custom,
        const ResidualFunc& z_subtract) const
    {
        Eigen::VectorXd v = z_subtract(z, h_custom(x_pri));
        Eigen::MatrixXd S = H_custom * P_pri * H_custom.transpose() + R_meas;

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