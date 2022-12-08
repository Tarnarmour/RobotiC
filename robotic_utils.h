#ifndef ROBOTIC_UTILS_H
#define ROBOTIC_UTILS_H

#include "Eigen/Dense"


namespace ru
{

int sign(double a);
Eigen::Matrix3d enforce_orthonormal(Eigen::Matrix3d A);
double to_degrees(double rads);
double to_rads(double degrees);
Eigen::Matrix3d skew(Eigen::Vector3d v);
}

#endif // ROBOTIC_UTILS_H
