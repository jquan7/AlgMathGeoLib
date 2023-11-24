#include "ukf_eigen.h"

#include <cmath>

namespace trm {

/*
 * @brief  Constructor
 */
TRM_ukf_c::TRM_ukf_c() {
    // initialize state
    m_is_initialized = false;

    // state mean dimension {CTRV:x,y,v,ψ,ω}
    m_state_dim = 5;

    // state mean dimension {CTRA:x,y,v,a,ψ,ω}
    // m_state_dim = 6;

    // augmented state mean dimension
    m_aug_state_dim = 7;

    // initial state vector
    m_state = VectorXd(m_state_dim);

    // TODO: Fine this noise parameter value
    // Process noise standard deviation longitudinal acceleration in m/s^2
    m_std_a = 0.8;

    // TODO: Fine this noise parameter value
    // Process noise standard deviation yaw acceleration in rad/s^2
    m_std_yawdd = M_PI / 4;

    // measurement noise standard deviation x in m
    m_std_px = 0.03;

    // measurement noise standard deviation y in m
    m_std_py = 0.15;

    m_tus = 0;

    // calculate total sigma points count
    m_sigma_pts_num = 2 * m_aug_state_dim + 1;

    // set spreading parameter
    m_lambda = 3 - m_aug_state_dim;

    // initialize weights
    m_weights = VectorXd(m_sigma_pts_num);
    m_weights.fill(1 / (2 * (m_lambda + m_aug_state_dim)));
    m_weights(0) = m_lambda / (m_lambda + m_aug_state_dim);

    // initialize predicted sigma points matrix
    m_pred_sig_pts_mat = MatrixXd::Zero(m_state_dim, m_sigma_pts_num);

    // initial covariance matrix
    m_P = MatrixXd(m_state_dim, m_state_dim);

    // initialize measurement covariance matrix
    m_R = MatrixXd(2, 2);
    m_R << m_std_px * m_std_px, 0, 0, m_std_py * m_std_py;

    // initialize noise covariance matrix
    m_Q = MatrixXd(2, 2);
    m_Q << m_std_a * m_std_a, 0, 0, m_std_yawdd * m_std_yawdd;

    // initialize the state transition matrix m_C
    m_C = MatrixXd(2, m_state_dim);
    m_C << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    m_I = MatrixXd::Identity(m_state_dim, m_state_dim);

    // initialize state mean and convariance
    m_state = VectorXd::Zero(m_state_dim);
    m_P = MatrixXd::Identity(m_state_dim, m_state_dim);
    m_P.diagonal().fill(0.5);
}

/**
 * Initialize m_state with the first measurement.
 */
void TRM_ukf_c::Init(const TRM_ukf_measure_t& measure) {
    m_state << measure.postion[0], measure.postion[1], 0, 0, 0;

    // done initializing, no need to predict or update
    m_tus = measure.tus;

    m_is_initialized = true;
}

/*
 * @brief  Process
 * @param  [IN] {TRM_ukf_measure_t} measure - The latest measurement data
 * @return {*}
 */
int TRM_ukf_c::Process(TRM_ukf_measure_t measure) {
    // initialization
    if (!m_is_initialized) {
        Init(measure);
        return 1;
    }

    // compute the time elapsed (s)
    double dt = (measure.tus - m_tus) / 1e6;

    // update timestamp to new measurement received timestamp
    m_tus = measure.tus;

    Predict(dt);

    VectorXd z = VectorXd(2);
    z << measure.postion[0], measure.postion[1];
    Update(z);

    // Check result.
    for (int d = 0; d < m_state_dim; ++d) {
        if (std::isnan(m_state(d)) || std::isinf(m_state(d))) {
            Init(measure);
            return 2;
        }
    }

    return 0;
}

/*
 * @brief  Predicts the state mean and covariance matrix
 * @param  [IN] {double} dt - time difference
 * @return {*}
 */
void TRM_ukf_c::Predict(double dt) {
    // Generate augmented sigma points
    MatrixXd sig_aug_mat = MatrixXd(m_aug_state_dim, m_sigma_pts_num);
    GenerateSigmaPoints(&sig_aug_mat);

    // Predict sigma points
    PredictSigmaPoints(sig_aug_mat, &m_pred_sig_pts_mat, dt);

    // Predict mean and covariance from sigma points
    PredictMeanAndCovariance();
}

/*
 * @brief  Updates the state mean x and covariance matrix P
 * @param  [IN] {const VectorXd&} z - measurement data
 * @return {*}
 */
void TRM_ukf_c::Update(const VectorXd& z) {
    // calculate Kalman Gain
    MatrixXd S = m_C * m_P * m_C.transpose() + m_R;
    m_K = m_P * m_C.transpose() * S.inverse();

    // calculate the difference between predicted and measurement
    VectorXd z_predicted = m_C * m_state;
    VectorXd y = z - z_predicted;

    // update the state based on Kalman gain
    m_state += m_K * y;

    // update the state covariance/uncertainty based on measurement
    m_P = (m_I - m_K * m_C) * m_P;
}

Eigen::VectorXd TRM_ukf_c::GetMeanState() const {
    return m_state;
}

void TRM_ukf_c::GenerateSigmaPoints(MatrixXd* sig_out_mat) {
    // create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(m_aug_state_dim);

    // set augmented mean vector, considering noise mean is zero
    x_aug.head(m_state_dim) = m_state;

    // create augmented covariance matrix with m_P,Q
    MatrixXd P_aug = MatrixXd::Zero(m_aug_state_dim, m_aug_state_dim);
    P_aug.topLeftCorner(m_state_dim, m_state_dim) = m_P;
    P_aug.bottomRightCorner(2, 2) = m_Q;

    // create a sigma point matrix
    MatrixXd sig_aug_mat = MatrixXd(m_aug_state_dim, m_sigma_pts_num);

    // calculate square root of P_aug by using Cholesky decomposition
    MatrixXd A = P_aug.llt().matrixL();

    /* calculate sigma points */
    // rule part1: first point is mean x
    sig_aug_mat.col(0) = x_aug;

    // rule part2: calculate next 1 to n_aug points
    sig_aug_mat.block(0, 1, m_aug_state_dim, m_aug_state_dim) =
        (std::sqrt(m_lambda + m_aug_state_dim) * A).colwise() + x_aug;

    // rule part3: calculate next n_aug to 2n_aug points
    sig_aug_mat.block(0, m_aug_state_dim + 1, m_aug_state_dim, m_aug_state_dim) =
        (-1 * sqrt(m_lambda + m_aug_state_dim) * A).colwise() + x_aug;

    *sig_out_mat = sig_aug_mat;
}

void TRM_ukf_c::PredictSigmaPoints(const MatrixXd& sig_aug_mat,
    MatrixXd* sig_out_mat, double dt) {
    // create matrix with predicted sigma points as columns
    MatrixXd sig_pred_mat = MatrixXd(m_state_dim, m_sigma_pts_num);

    // predict sigma points
    // avoid division by zero; write predicted sigma points into right column
    for (int i = 0; i < sig_aug_mat.cols(); ++i) {
        sig_pred_mat.col(i) = PredictSingleSigmaPoint(sig_aug_mat.col(i), dt);
    }

    *sig_out_mat = sig_pred_mat;
}

/*
 * @brief  Predicts a single sigma point based on augmented sigma point provided
 * @param  [IN] {const VectorXd&} x_aug - sigma point
 * @param  [IN] {double} dt - time difference
 * @return {*}
 */
VectorXd TRM_ukf_c::PredictSingleSigmaPoint(const VectorXd& x_aug, double dt) {
    double v = x_aug(2);
    double yaw_angle = x_aug(3);
    double yaw_rate = x_aug(4);
    double noise_longitudinal_a = x_aug(m_state_dim);
    double noise_yaw_a = x_aug(6);

    // do some common calculations
    double sin_yaw_angle = std::sin(yaw_angle);
    double cos_yaw_angle = std::cos(yaw_angle);
    double dt_square = dt * dt;

    double px_change = 0, py_change = 0;

    /* CTRV */
    if (fabs(yaw_rate) > 1e-6) {
        // calculations specific to this case
        double c1 = v / yaw_rate;
        double c2 = yaw_angle + yaw_rate * dt;
        double sin_c2 = std::sin(c2);
        double cos_c2 = std::cos(c2);

        px_change = (c1 * (sin_c2 - sin_yaw_angle));
        py_change = (c1 * (-cos_c2 + cos_yaw_angle));
    }
    /* CV */
    else {
        px_change = (v * cos_yaw_angle * dt);
        py_change = (v * sin_yaw_angle * dt);
    }

    VectorXd x_change = VectorXd(m_state_dim);
    x_change << px_change, py_change, 0, (yaw_rate * dt), 0;

    VectorXd x_noise = VectorXd(m_state_dim);
    x_noise << (0.5 * dt_square * cos_yaw_angle * noise_longitudinal_a),
        (0.5 * dt_square * sin_yaw_angle * noise_longitudinal_a),
        (dt * noise_longitudinal_a), (0.5 * dt_square * noise_yaw_a),
        (dt * noise_yaw_a);
    x_change += x_noise;

    // extract state vector x from augmented state vector
    // (contains last 2 elements as noise values after augmentation)
    VectorXd x_old = x_aug.head(m_state_dim);

    // return total change in x state
    return (x_old + x_change);
}

static float NormalizeAngle(float rad) {
    rad = rad - 2 * M_PI * floor((rad + M_PI) / (2 * M_PI));
    return rad;
}

void TRM_ukf_c::PredictMeanAndCovariance() {
    // for ease of calculation let's make a matrix of weights
    MatrixXd W = m_weights.transpose().replicate(m_pred_sig_pts_mat.rows(), 1);

    /* predict state mean */
    // multiply weights with sigma points element wise
    MatrixXd pred_sig_weight = (m_pred_sig_pts_mat.array() * W.array()).matrix();
    // take element wise (rowwise) sum of all sigma points
    m_state = pred_sig_weight.rowwise().sum();

    /* predict state covariance matrix */
    MatrixXd mean_distance = (m_pred_sig_pts_mat.colwise() - m_state);
    // angle normalization
    for (int i = 0; i < mean_distance.cols(); ++i) {
        mean_distance.col(i)(3) = NormalizeAngle(mean_distance.col(i)(3));
    }
    // multiply each sigma point mean distance with it's weight
    MatrixXd mean_distance_weight = (mean_distance.array() * W.array()).matrix();
    m_P = mean_distance_weight * mean_distance.transpose();
}

#if 0
void UkfDemo() {
  TRM_ukf_c ukf;
  vector<TRM_ukf_measure_t> pos_list;
  size_t N = pos_list.size();
  for (size_t k = 0; k < N; ++k) {
    ukf.Process(pos_list[k]);
    VectorXd x = ukf.GetMeanState();

    float x = x(0);
    float y = x(1);
    float v = x(2);
    float yaw = x(3);
    float yaw_rate = x(4);
    float vx = x(2) * cos(x(3));
    float vy_estimate_ = x(2) * sin(x(3));
  }
}
#endif

}  // namespace trm
