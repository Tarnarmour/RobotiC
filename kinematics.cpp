#define _USE_MATH_DEFINES
#include <cmath> // this has to be at the top for some inane reason

#include "kinematics.h"
#include "transforms.h"

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>


SerialArm::SerialArm(std::vector<std::vector<double>> dh, std::vector<char> jt) :
    _dh(dh), _jt(jt)
{
    _n = dh.size();
}

std::vector<std::vector<double>> SerialArm::get_dh()
{
    return _dh;
}

std::vector<char> SerialArm::get_jt()
{
    return _jt;
}

SE3 SerialArm::fk(Eigen::VectorXd q) const
{
    return this->fk(q, 0, _n);
}

SE3 SerialArm::fk(Eigen::VectorXd q, int iend) const
{
    return this->fk(q, 0, iend);
}

SE3 SerialArm::fk(Eigen::VectorXd q, int istart, int iend) const
{
    // check size of q, istart, and iend

    if (q.size() != _n)
    {
        throw std::length_error("q input to fk does not match number of joints!");
    }

    // for loop, convert dh params to SE3 and compose

//    [[cth, -sth * cal, sth *sal, a * cth],
//                         [sth, cth * cal, -cth * sal, a * sth],
//                         [0, sal, cal, d],
//                         [0, 0, 0, 1]],

    SE3 T;

    for (int i = istart; i < iend; i++)
    {
        double th;
        double d;

        if (_jt[i] == 'r')
        {
            d = _dh[i][0];
            th = _dh[i][1] + q(i);
        }
        else
        {
            d = _dh[i][0] + q(i);
            th = _dh[i][1];
        }
        double a = _dh[i][2];
        double al = _dh[i][3];

        double sth{std::sin(th)};
        double cth{std::cos(th)};
        double cal{std::cos(al)};
        double sal{std::sin(al)};

        Eigen::Matrix4d A{{cth, -sth * cal, sth * sal, a * cth},
                          {sth, cth * cal, -cth * sal, a * sth},
                          {0, sal, cal, d},
                          {0, 0, 0, 1}};
        SE3 Ti{A};
        T = T + Ti;
    }
    // return
    return T;
}
