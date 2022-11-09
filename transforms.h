#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Dense>


class SO3
{
public:
    SO3();
protected:
    Eigen::Matrix2d mMatrix;
};

#endif // TRANSFORMS_H
