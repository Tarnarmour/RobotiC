#include "transforms.h"
#include "Eigen/Dense"
#include "Eigen/Jacobi"


using namespace Eigen;


// SE3 Functions
SE3::SE3()
{
    A = Matrix4d::Identity();
}

SE3::SE3(Matrix3d R)
{
    A = Matrix4d::Identity();
    A({0, 1, 2}, {0, 1, 2}) = R;
}

SE3::SE3(Vector3d p)
{
    A = Matrix4d::Identity();
    A({0, 1, 2}, {3}) = p;
}

SE3::SE3(Matrix3d R, Vector3d p)
{
    A = Matrix4d::Identity();
    A({0, 1, 2}, {0, 1, 2}) = R;
    A({0, 1, 2}, {3}) = p;
}

// Delegate to the R p constructor
SE3::SE3(Vector3d p, Matrix3d R) : SE3(R, p) {}

SE3::SE3(Matrix4d T) : A{T}
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


// Non-class Functions
Matrix3d enforce_orthonormal(Matrix3d A)
{
    if ((A.transpose() * A).isApprox(Matrix3d::Identity()))
    {
        return A;
    }
    else
    {
        JacobiSVD<Matrix3d> svd(A, ComputeFullU | ComputeFullV);
        return svd.matrixU() * svd.matrixV().transpose();
    }
}

Matrix4d inv(const SE3 T)
{
    Matrix3d R = T.get_R();
    Vector3d p = T.get_p();
    Matrix4d A;
    A << R.transpose(), -R.transpose() * p, 0, 0, 0, 1;
    return A;
}
