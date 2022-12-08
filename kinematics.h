#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "transforms.h"

#include <Eigen/Dense>
#include <vector>


class SerialArm
{
public:
    SerialArm(std::vector<std::vector<double>> dh, std::vector<char> jt);
    std::vector<std::vector<double>> get_dh();
    std::vector<char> get_jt();

    SE3 fk(Eigen::VectorXd q) const;
    SE3 fk(Eigen::VectorXd q, int iend) const;
    SE3 fk(Eigen::VectorXd q, int istart, int iend) const;

private:
    std::vector<std::vector<double>> _dh;
    std::vector<char> _jt;
    int _n;
};

#endif // KINEMATICS_H
