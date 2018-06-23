#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Eigen/Dense"
#include "sensor_type.h"

class KalmanFilter {
public:
  static std::unique_ptr<KalmanFilter>
  Create(const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
         const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
         const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

  ~KalmanFilter();

  void Predict();

  void Update(const Eigen::VectorXd &z, const SensorType sensor_type);

private:
  KalmanFilter(const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
               const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
               const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

  void UpdateWithLidarMeasurement(const Eigen::VectorXd &z);
  void UpdateWithRadarMeasurement(const Eigen::VectorXd &z);

  // Estimate.
  Eigen::VectorXd x_;
  // Uncertainty covariance.
  Eigen::MatrixXd P_;
  // State transition matrix.
  Eigen::MatrixXd F_;
  // Process covariance matrix.
  Eigen::MatrixXd Q_;
  // Measurement function.
  Eigen::MatrixXd H_;
  // Measurement noise.
  Eigen::MatrixXd R_;
};

#endif