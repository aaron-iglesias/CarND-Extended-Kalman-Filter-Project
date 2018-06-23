#include "measurement.h"
#include "sensor_type.h"

Measurement::Measurement(const Eigen::VectorXd &measurement,
                         const SensorType sensor_type,
                         const long long timestamp)
    : measurement_(measurement), sensor_type_(sensor_type),
      timestamp_(timestamp) {}

const Eigen::VectorXd &Measurement::GetMeasurement() const {
  return measurement_;
}

const SensorType Measurement::GetSensorType() const { return sensor_type_; }

const long long Measurement::GetTimestamp() const { return timestamp_; }