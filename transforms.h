#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Dense>


class SE3
{
public:
    SE3();
    SE3(Eigen::Matrix3d R);
    SE3(Eigen::Vector3d p);
    SE3(Eigen::Matrix3d R, Eigen::Vector3d p);
    SE3(Eigen::Vector3d p, Eigen::Matrix3d R);
    SE3(Eigen::Matrix4d T);
    SE3(const SE3& other);
    SE3 operator= (const SE3&);
    SE3(SE3&&);
    SE3 operator= (SE3&&);

    Eigen::Matrix4d A;

    Eigen::Matrix3d get_R() const;
    Eigen::Vector3d get_p() const;

    void inv();
protected:
};

Eigen::Matrix3d enforce_orthonormal(Eigen::Matrix3d A);
Eigen::Matrix4d inv(SE3 T);

#endif // TRANSFORMS_H
