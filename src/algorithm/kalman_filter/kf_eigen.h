// #pragma once
#ifndef GEOMETRY_KALMAN_EIGEN_H_
#define GEOMETRY_KALMAN_EIGEN_H_

#include <Eigen/Dense>

namespace trm {

class TRM_kalman_filter_c {
   public:
    /*
    * @brief        Create a Kalman filter
    * @param        [IN] {const Eigen::MatrixXd&} A - System dynamics matrix
    * @param        [IN] {const Eigen::MatrixXd&} C - Output matrix
    * @param        [IN] {const Eigen::MatrixXd&} Q - Process noise covariance
    * @param        [IN] {const Eigen::MatrixXd&} R - Measurement noise covariance
    * @param        [IN] {const Eigen::MatrixXd&} P - Estimate error covariance
    * @return       {*}
    */
    TRM_kalman_filter_c(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P);

    /*
    * @brief        Initialize the filter
    * @param        [IN] {const Eigen::VectorXd&} x0 - initial state
    * @return       {*}
    */
    void init(const Eigen::VectorXd& x0);

    /*
    * @brief        Predict.
    * @return       {*}
    */
    void predict();

    /*
    * @brief        Update the estimated state based on measured values.
    * @param        [IN] {const Eigen::VectorXd&} y - measured values
    * @return       {*}
    */
    void update(const Eigen::VectorXd& y);

    /*
    * @brief        Update the estimated state based on measured values.
    * @param        [IN] {const Eigen::VectorXd&} y - measured values
    * @param        [IN] {const Eigen::MatrixXd&} Q - Given process noise covariance
    * @return       {*}
    */
    void update(const Eigen::VectorXd& y, const Eigen::MatrixXd Q);

    /*
    * @brief        Return the current state.
    * @return       {Eigen::VectorXd} x_hat - current state
    */
    Eigen::VectorXd state() {
        return x_hat;
    };

   private:
    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K;

    // System dimensions
    // m - Number of measurements; n - Number of states
    int m, n;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;

};  // TRM_kalman_filter_c

}  // namespace trm

#endif /*GEOMETRY_KALMAN_EIGEN_H_*/
