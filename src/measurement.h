#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "Eigen/Dense"
#include "sensor_type.h"

class Measurement {
public:
  Measurement(const Eigen::VectorXd &measurement, const SensorType sensor_type,
              const long long timestamp);

  const Eigen::VectorXd &GetMeasurement() const;
  const SensorType GetSensorType() const;
  const long long GetTimestamp() const;

private:
  const Eigen::VectorXd measurement_;
  const SensorType sensor_type_;
  const long long timestamp_;
};

#endif