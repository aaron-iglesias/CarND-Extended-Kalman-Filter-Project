#include "Eigen/Dense"
#include "kalman_filter.h"
#include "util.h"

std::unique_ptr<KalmanFilter>
KalmanFilter::Create(const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
                     const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  // TODO(aaron-iglesias): Check if matrices have the correct dimensions.
  return std::unique_ptr<KalmanFilter>(new KalmanFilter(x, P, F, Q, H, R));
}

KalmanFilter::KalmanFilter(const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
                           const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  x_ = x;
  P_ = P;
  F_ = F;
  Q_ = Q;
  H_ = H;
  R_ = R;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

// TODO(aaron-iglesias): (1) change string to ENUM, (2) throw exception for
// invalid string input.
void KalmanFilter::Update(const Eigen::VectorXd &z,
                          const std::string &sensor_type) {
  if (sensor_type == "lidar") {
    UpdateWithLidarMeasurement(z);
  } else if (sensor_type == "radar") {
    UpdateWithRadarMeasurement(z);
  }
}

// TODO(aaron-iglesias): (1) refactor code to avoid redundancy.
void KalmanFilter::UpdateWithLidarMeasurement(const Eigen::VectorXd &z) {
  const Eigen::VectorXd y = z - H_ * x_;
  const Eigen::MatrixXd Ht = H_.transpose();
  const Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  const Eigen::MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(K.rows(), K.rows());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateWithRadarMeasurement(const Eigen::VectorXd &z) {
  const Eigen::VectorXd y = z - util::CartesianToPolar(x_);
  const Eigen::MatrixXd Ht = H_.transpose();
  const Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  const Eigen::MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(K.rows(), K.rows());
  P_ = (I - K * H_) * P_;
}