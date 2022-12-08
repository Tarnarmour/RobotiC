#define _USE_MATH_DEFINES

#include "robotic_utils.h"
#include <cmath> // this has to be at the top for some inane reason

#include "Eigen/Dense"


int ru::sign(double a)
{
    return (a > 0.0) - (a < 0.0);
}

Eigen::Matrix3d ru::enforce_orthonormal(Eigen::Matrix3d A)
{
    if ((A.transpose() * A).isApprox(Eigen::Matrix3d::Identity()))
    {
        return A;
    }
    else
    {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        return svd.matrixU() * svd.matrixV().transpose();
    }
}

double ru::to_degrees(double rads)
{
    return rads * 180.0 / M_PI;
}

double ru::to_rads(double degrees)
{
    return degrees * M_PI / 180.0;
}

Eigen::Matrix3d ru::skew(Eigen::Vector3d v)
{
    Eigen::Matrix3d S;
    S << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return S;
}
