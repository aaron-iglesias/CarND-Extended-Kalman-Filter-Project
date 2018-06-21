#ifndef UTIL_H
#define UTIL_H

#include "Eigen/Dense"

class Util {
public:
  const Eigen::VectorXd
  CartesianToPolar(const Eigen::VectorXd &cartesian_coordinates);
};

#endif