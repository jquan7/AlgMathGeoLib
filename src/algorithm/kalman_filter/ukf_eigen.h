#ifndef GEOMETRY_UKF_EIGEN_H_
#define GEOMETRY_UKF_EIGEN_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace trm {

typedef struct {
    long long tus;
    VectorXd postion;
} TRM_ukf_measure_t;

class TRM_ukf_c {
   public:
    /*
     * @brief  Constructor
     */
    TRM_ukf_c();

    /*
     * @brief  Destructor
     */
    ~TRM_ukf_c() = default;

    /*
     * @brief  Process
     * @param  [IN] {TRM_ukf_measure_t} measure - The latest measurement data
     * @return {*}
     */
    int Process(TRM_ukf_measure_t measure);

    /*
     * @brief  Predicts the state mean and covariance matrix
     * @param  [IN] {double} dt - time difference
     * @return {*}
     */
    void Predict(double dt);

    /*
     * @brief  Updates the state mean x and covariance matrix P
     * @param  [IN] {const VectorXd&} z - measurement data
     * @return {*}
     */
    void Update(const VectorXd& z);

    /*
     * Returns mean state x
     * @return {VectorXd} state
     */
    VectorXd GetMeanState() const;

   private:
    /*
     * @brief  Initialize UKF
     * @param  [IN] {const TRM_ukf_measure_t&} measure - measurement data
     * @return {*}
     */
    void Init(const TRM_ukf_measure_t& measure);

    /*
     * @brief  Define structure of UKF
     * @return {*}
     */
    void GenerateSigmaPoints(MatrixXd* sig_out_mat);
    void PredictSigmaPoints(const MatrixXd& sig_aug_mat, MatrixXd* sig_out_mat,
        double dt);
    void PredictMeanAndCovariance();

    /*
     * @brief  Predicts a single sigma point based on augmented sigma point provided
     * @param  [IN] {const VectorXd&} x_aug - sigma point
     * @param  [IN] {double} dt - time difference
     * @return {*}
     */
    VectorXd PredictSingleSigmaPoint(const VectorXd& x_aug, double dt);

    // Initialize state
    bool m_is_initialized;

    // State vector
    VectorXd m_state;

    // Predicted sigma points matrix
    MatrixXd m_pred_sig_pts_mat;

    // Time when the state is true (us)
    long long m_tus;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double m_std_a;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double m_std_yawdd;

    // Measurement noise standard deviation x in m
    double m_std_px;

    // Measurement noise standard deviation y in m
    double m_std_py;

    // Weights of sigma points
    VectorXd m_weights;

    // State dimension
    int m_state_dim;

    // Augmented state dimension
    int m_aug_state_dim;

    // Total sigma points count
    int m_sigma_pts_num;

    // Sigma point spreading parameter
    double m_lambda;

    // Matrices for computation
    MatrixXd m_P, m_R, m_C, m_Q, m_K;

    // Identity
    MatrixXd m_I;
};  // TRM_ukf_c

}  // namespace trm

#endif /* GEOMETRY_UKF_EIGEN_H_ */
