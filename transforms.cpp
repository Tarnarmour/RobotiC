#define _USE_MATH_DEFINES
#include <cmath> // this has to be at the top for some inane reason

#include "transforms.h"

#include "robotic_utils.h"
#include "Eigen/Dense"


using namespace Eigen;


SO3::SO3()
{

}

SO3::SO3(Matrix3d R) : _R{R}
{

}

SO3 SO3::operator-() const
{
    SO3 R(_R.transpose());
    return R;
}

SO3 SO3::inv() const
{
    SO3 R(_R.transpose());
    return R;
}

Vector3d SO3::get_z() const
{
    Vector3d z = _R.col(2);
    return z;
}

AxisAngle SO3::get_axis() const
{
    AxisAngle aa;
    aa.theta = std::acos(0.5 * (_R(0, 0) + _R(1, 1) + _R(2, 2) - 1));
    Vector3d v;
    if (std::abs(aa.theta) < 1e-12) // identity matrix
    {
        v << 1, 0, 0;
    }
    else
    {
        v << (_R(2, 1) - _R(1, 2)) / (2 * std::sin(aa.theta)),
                (_R(2, 0) - _R(2, 0)) / (2 * std::sin(aa.theta)),
                (_R(1, 0) - _R(0, 1)) / (2 * std::sin(aa.theta));
    }
    aa.v = v;
    return aa;
}

Vector4d SO3::get_quat() const
{
    Vector4d v;
    v << 0.5 * std::sqrt(std::abs(_R(0, 0) + _R(1, 1) + _R(2, 2) + 1)),
            0.5 * ru::sign(_R(2, 1) - _R(1, 2)) * std::sqrt(std::abs(_R(0, 0) - _R(1, 1) - _R(2, 2) + 1)),
            0.5 * ru::sign(_R(0, 2) - _R(2, 0)) * std::sqrt(std::abs(_R(1, 1) - _R(2, 2) - _R(0, 0) + 1)),
            0.5 * ru::sign(_R(1, 0) - _R(0, 1)) * std::sqrt(std::abs(_R(2, 2) - _R(0, 0) - _R(1, 1) + 1));
    return v;
}

SO3 rotx(double theta)
{
    Matrix3d R;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    R << 1.0, 0.0, 0.0,
        0.0, cth, -sth,
        0.0, sth, cth;
    SO3 A = SO3(R);
    return A;
}

SO3 roty(double theta)
{
    Matrix3d R;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    R << cth, 0.0, sth,
        0.0, 1.0, 0.0,
        -sth, 0.0, cth;
    SO3 A = SO3(R);
    return A;
}

SO3 rotz(double theta)
{
    Matrix3d R;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    R << cth, -sth, 0.0,
        sth, cth, 0.0,
        0.0, 0.0, 1.0;
    SO3 A = SO3(R);
    return A;
}

SO3 from_rpy(double roll, double pitch, double yaw, bool degree)
{
    SO3 R = rotx(roll) + roty(pitch) + rotz(yaw);
    return R;
}

SO3 from_axis(double theta, Vector3d v)
{
    if (v.norm() < 1e-12)
    {
        return SO3();
    }
    v.normalize();
    Matrix3d S = ru::skew(v);
    Matrix3d R = Matrix3d::Identity() + std::sin(theta) * S + (1.0 - std::cos(theta)) * S * S;
    return R;
}

SO3 from_quat(Vector4d q)
{
    q.normalize();
    if (q.norm() < 1e-12)
    {
        return SO3();
    }
    double nu = q[0];
    double ex = q[1];
    double ey = q[2];
    double ez = q[3];

    Matrix3d R;
    R << 2 * (nu * nu + ex * ex) - 1, 2 * (ex * ey - nu * ez), 2 * (ex * ez + nu * ey),
            2 * (ex * ey + nu * ez), 2 * (nu * nu + ey * ey) - 1, 2 * (ey * ez - nu * ex),
            2 * (ex * ez - nu * ey), 2 * (ey * ez + nu * ex), 2 * (nu * nu + ez * ez) - 1;
    return R;
}

Matrix3d SO3::get_R() const
{
    return _R;
}

SO3 operator+ (const SO3 lh, const SO3 rh)
{
    SO3 R(lh.get_R() * rh.get_R());
    return R;
}

SO3 operator- (const SO3 lh, const SO3 rh)
{
    SO3 R(lh.get_R() * rh.get_R().transpose());
    return R;
}

bool operator== (const SO3 lh, const SO3 rh)
{
    return lh.get_R().isApprox(rh.get_R());
}

// SE3 Functions
SE3::SE3()
{
    _A = Matrix4d::Identity();
}

SE3::SE3(SO3 R)
{
    _A = Matrix4d::Identity();
    _A({0, 1, 2}, {0, 1, 2}) = R.get_R();
}

SE3::SE3(Vector3d p)
{
    _A = Matrix4d::Identity();
    _A({0, 1, 2}, {3}) = p;
}

SE3::SE3(Matrix4d A) : _A{A}
{

}

Matrix3d SE3::get_R() const
{
    return A({0, 1, 2}, {0, 1, 2});
}

Vector3d SE3::get_p() const
{
    Vector3d p = A({0, 1, 2}, {3});
    return p;
}

void SE3::inv()
{
    Matrix3d R = this->get_R();
    Vector3d p = this->get_p();
    A << R.transpose(), -R.transpose() * p, 0, 0, 0, 1;
}

Matrix4d inv(const SE3 T)
{
    Matrix3d R = T.get_R();
    Vector3d p = T.get_p();
    Matrix4d A;
    A << R.transpose(), -R.transpose() * p, 0, 0, 0, 1;
    return A;
}

SE3 trotx(double theta)
{
    Matrix4d A;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    A << 1.0, 0.0, 0.0, 0.0,
        0.0, cth, -sth, 0.0,
        0.0, sth, cth, 0.0,
        0.0, 0.0, 0.0, 1.0;
    SE3 T = SE3(A);
    return T;
}

SE3 troty(double theta)
{
    Matrix4d A;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    A << cth, 0.0, sth, 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sth, 0.0, cth, 0.0,
        0.0, 0.0, 0.0, 1.0;
    SE3 T = SE3(A);
    return A;
}

SE3 trotz(double theta)
{
    Matrix4d A;
    double sth = std::sin(theta);
    double cth = std::cos(theta);
    A << cth, -sth, 0.0, 0.0,
        sth, cth, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    SE3 T = SE3(A);
    return A;
}

Vector4d convert_R_to_quaternion(Matrix3d R)
{
    Vector4d q;
    q(0) = 0.5 * std::sqrt(std::abs(R(0, 0) + R(1, 1) + R(2, 2) + 1.0));
    q(1) = 0.5 * ru::sign(R(2, 1) - R(1, 2)) * std::sqrt(std::abs(R(0, 0) - R(1, 1) - R(2, 2) + 1));
    q(2) = 0.5 * ru::sign(R(0, 2) - R(2, 0)) * std::sqrt(std::abs(R(1, 1) - R(2, 2) - R(0, 0) + 1));
    q(3) = 0.5 * ru::sign(R(1, 0) - R(0, 2)) * std::sqrt(std::abs(R(2, 2) - R(0, 0) - R(1, 1) + 1));
    return q;
}
