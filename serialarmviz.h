#ifndef SERIALARMVIZ_H
#define SERIALARMVIZ_H


#include "kinematics.h"
#include "transforms.h"
#include "Eigen/Dense"

#include <vector>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>



class SerialArmViz
{
public:
    SerialArmViz();
    SerialArmViz(SerialArm arm, Qt3DCore::QEntity* parent);
    void update(Eigen::Vector4d q);
    SerialArm _arm;
    int _n;
private:

    std::vector<Qt3DCore::QEntity*> _jointList;
    std::vector<Qt3DCore::QEntity*> _linkList;
    std::vector<Qt3DCore::QTransform*> _transformList;
};

#endif // SERIALARMVIZ_H
