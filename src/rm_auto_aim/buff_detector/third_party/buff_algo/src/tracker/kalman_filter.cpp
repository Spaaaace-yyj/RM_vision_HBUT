#include "buff_algo/tracker/kalman_filter.hpp"

namespace buff_algo {

template <unsigned MEASUREMENT_DIM>
KF<MEASUREMENT_DIM>::KF(const cv::FileNode& fn) :
    x_(STATE_DIM),
    P_(STATE_DIM, STATE_DIM),
    F_(STATE_DIM, STATE_DIM),
    Q_(STATE_DIM, STATE_DIM),
    R_(MEASUREMENT_DIM, MEASUREMENT_DIM),
    H_(MEASUREMENT_DIM, STATE_DIM) {
    cv::Mat Q_cv, R_cv;
    fn["process_noise"] >> Q_cv;
    fn["measurement_noise"] >> R_cv;
    if (!(Q_cv.cols == Q_cv.rows && Q_cv.cols == static_cast<int>(STATE_DIM))) {
        throw std::invalid_argument("invalid size of process noise");
    }
    if (!(R_cv.cols == R_cv.rows && R_cv.cols == static_cast<int>(MEASUREMENT_DIM))) {
        throw std::invalid_argument("invalid size of measurement noise");
    }
    cv::cv2eigen(Q_cv, Q_);
    cv::cv2eigen(R_cv, R_);
    reset();
}

template <unsigned MEASUREMENT_DIM>
void KF<MEASUREMENT_DIM>::reset() {
    initialize(Eigen::Vector<float, MEASUREMENT_DIM>::Zero());
}

template <unsigned MEASUREMENT_DIM>
void KF<MEASUREMENT_DIM>::initialize(const Eigen::Vector<float, MEASUREMENT_DIM>& meas) {
    x_.setZero();
    x_.template head<MEASUREMENT_DIM>() = meas;
    P_.setIdentity();
    P_ *= 1.0f;
    H_.setZero();
    H_.template topLeftCorner<MEASUREMENT_DIM, MEASUREMENT_DIM>().setIdentity();
}

template <unsigned MEASUREMENT_DIM>
void KF<MEASUREMENT_DIM>::predict(float delta_t) {
    F_.setIdentity();
    for (unsigned i = 0; i < MEASUREMENT_DIM; i++) {
        F_(i, MEASUREMENT_DIM + i) = delta_t;
    }
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

template <unsigned MEASUREMENT_DIM>
void KF<MEASUREMENT_DIM>::update(const Eigen::Vector<float, MEASUREMENT_DIM>& meas) {
    Eigen::Vector<float, MEASUREMENT_DIM> y = meas - H_ * x_;
    Eigen::Matrix<float, MEASUREMENT_DIM, MEASUREMENT_DIM> S =
        H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<float, STATE_DIM, MEASUREMENT_DIM> K =
        P_ * H_.transpose() * S.inverse();
    x_ += K * y;
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> I = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> KH = K * H_;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose();
}

template <unsigned MEASUREMENT_DIM>
Eigen::Vector<float, MEASUREMENT_DIM> KF<MEASUREMENT_DIM>::value() const {
    return x_.template head<MEASUREMENT_DIM>();
}

template <unsigned MEASUREMENT_DIM>
Eigen::Vector<float, MEASUREMENT_DIM> KF<MEASUREMENT_DIM>::derivative() const {
    return x_.template tail<MEASUREMENT_DIM>();
}

template <unsigned MEASUREMENT_DIM>
void KF<MEASUREMENT_DIM>::force_change_value(const Eigen::Vector<float, MEASUREMENT_DIM>& meas) {
    x_.template head<MEASUREMENT_DIM>() = meas;
}

template class KF<1>;
template class KF<2>;
template class KF<3>;

} // namespace buff_algo
