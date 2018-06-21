#include "util.h"
#include "Eigen/Dense"
#include <cmath>

const Eigen::VectorXd
Util::CartesianToPolar(const Eigen::VectorXd &cartesian_coordinates) {
  Eigen::VectorXd polar_coordinates(3);

  const float p_x = cartesian_coordinates(0);
  const float p_y = cartesian_coordinates(1);
  const float v_x = cartesian_coordinates(2);
  const float v_y = cartesian_coordinates(3);

  const float range = sqrt(pow(p_x, 2) + pow(p_y, 2));
  const float phi = atan2(p_y, p_x);
  const float range_rate = (p_x * v_x + p_y * v_y) / range;

  polar_coordinates << range, phi, range_rate;
  return polar_coordinates;
}