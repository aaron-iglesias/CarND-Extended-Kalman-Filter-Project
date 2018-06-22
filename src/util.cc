#include "util.h"
#include "Eigen/Dense"
#include <cmath>

namespace util {
const Eigen::VectorXd
CartesianToPolar(const Eigen::VectorXd &cartesian_coordinates) {
  Eigen::VectorXd polar_coordinates(3);

  const float p_x = cartesian_coordinates(0);
  const float p_y = cartesian_coordinates(1);
  const float v_x = cartesian_coordinates(2);
  const float v_y = cartesian_coordinates(3);

  const float range = sqrt(pow(p_x, 2) + pow(p_y, 2));
  const float bearing = atan2(p_y, p_x);
  const float range_rate = (p_x * v_x + p_y * v_y) / range;

  polar_coordinates << range, bearing, range_rate;
  return polar_coordinates;
}

// TODO(aaron-iglesias): Document expected ordering of contents for x_state
// input.
const Eigen::MatrixXd Jacobian(const Eigen::VectorXd &x_state) {
  Eigen::MatrixXd Hj(3, 4);

  const float p_x = x_state(0);
  const float p_y = x_state(1);
  const float v_x = x_state(2);
  const float v_y = x_state(3);

  const float min_denom = pow(10, -4);

  const float range_squared = max(pow(p_x, 2) + pow(p_y, 2), min_denom);
  const float range = max(sqrt(range_squared), min_denom);

  Hj << p_x / range, p_y / range, 0, 0, -p_y / range_squared,
      p_x / range_squared, 0, 0,
      p_y * (v_x * p_y - v_y * p_x) / pow(range_squared, 1.5),
      p_x * (v_y * p_x - v_x * p_y) / pow(range_squared, 1.5), p_x / range,
      p_y / range;

  return Hj;
}
} // namespace util