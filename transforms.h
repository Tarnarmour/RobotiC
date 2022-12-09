#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Dense>


struct AxisAngle
{
    Eigen::Vector3d v{1, 0, 0};
    double theta{0.0};
};


class SO3
{
public:
    SO3();
    SO3(Eigen::Matrix3d R);
    SO3 operator-() const;
    SO3 inv() const;

    Eigen::Matrix3d get_R() const;
    Eigen::Vector3d get_z() const;

    AxisAngle get_axis() const;
    Eigen::Vector4d get_quat() const;

protected:
    Eigen::Matrix3d _R{{1.0, 0, 0}, {0, 1.0, 0}, {0, 0, 1.0}};
};

SO3 rotx(double theta);
SO3 roty(double theta);
SO3 rotz(double theta);

SO3 from_rpy(double roll, double pitch, double yaw, bool degree = false);
SO3 from_axis(double theta, Eigen::Vector3d v); // axis angle
SO3 from_quat(Eigen::Vector4d q);

SO3 operator+ (const SO3 lh, const SO3 rh);
SO3 operator- (const SO3 lh, const SO3 rh);
bool operator== (const SO3 lh, const SO3 rh);


class SE3
{
public:
    SE3();
    SE3(SO3 R);
    SE3(Eigen::Vector3d p);
    SE3(SO3 R, Eigen::Vector3d p);
    SE3(Eigen::Matrix4d A);

    SE3 operator-() const;

    SO3 get_R() const;
    Eigen::Vector3d get_p() const;
    Eigen::Matrix4d get_A() const;

    SE3 inv() const;
protected:
    Eigen::Matrix4d _A{{1, 0, 0, 0},
                       {0, 1, 0, 0},
                       {0, 0, 1, 0},
                       {0, 0, 0, 1}};
};

SE3 transl(Eigen::Vector3d p);
SE3 transl(double x, double y, double z);
SE3 trotx(double theta);
SE3 troty(double theta);
SE3 trotz(double theta);

SE3 operator+ (const SE3 lh, const SE3 rh);
SE3 operator- (const SE3 lh, const SE3 rh);
bool operator== (const SE3 lh, const SE3 rh);

#endif // TRANSFORMS_H
