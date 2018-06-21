#ifndef UTIL_H
#define UTIL_H

#include "Eigen/Dense"

namespace util {
	const Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &cartesian_coordinates);
}

#endif