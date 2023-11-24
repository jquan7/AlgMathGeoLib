#include "kf_eigen.h"

namespace trm {

/*
 * @brief        Create a Kalman filter
 * @param        [IN] {const Eigen::MatrixXd&} A - System dynamics matrix
 * @param        [IN] {const Eigen::MatrixXd&} C - Output matrix
 * @param        [IN] {const Eigen::MatrixXd&} Q - Process noise covariance
 * @param        [IN] {const Eigen::MatrixXd&} R - Measurement noise covariance
 * @param        [IN] {const Eigen::MatrixXd&} P - Estimate error covariance
 * @return       {*}
 */
TRM_kalman_filter_c::TRM_kalman_filter_c(const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, const Eigen::MatrixXd& P) :
    A(A),
    C(C),
    Q(Q),
    R(R),
    P(P),
    m(C.rows()),
    n(A.rows()),
    I(n, n),
    x_hat(n),
    x_hat_new(n) {
    I.setIdentity();
}

/*
 * @brief        Initialize the filter
 * @param        [IN] {const Eigen::VectorXd&} x0 - initial state
 * @return       {*}
 */
void TRM_kalman_filter_c::init(const Eigen::VectorXd& x0) {
    I.setIdentity();
    x_hat = x0;
}

/*
 * @brief        Predict.
 * @return       {*}
 */
void TRM_kalman_filter_c::predict() {
    x_hat_new = A * x_hat;
    P = A * P * A.transpose() + Q;
    x_hat = x_hat_new;
}

/*
 * @brief        Update the estimated state based on measured values.
 * @param        [IN] {const Eigen::VectorXd&} y - measured values
 * @return       {*}
 */
void TRM_kalman_filter_c::update(const Eigen::VectorXd& y) {
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    x_hat = x_hat_new;
}

/*
 * @brief        Update the estimated state based on measured values.
 * @param        [IN] {const Eigen::VectorXd&} y - measured values
 * @param        [IN] {const Eigen::MatrixXd&} Q - Given process noise covariance
 * @return       {*}
 */
void TRM_kalman_filter_c::update(const Eigen::VectorXd& y, const Eigen::MatrixXd Q) {
    this->Q = Q;
    update(y);
}

}  // namespace trm
