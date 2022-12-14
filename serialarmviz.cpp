#include "serialarmviz.h"
#include "kinematics.h"
#include "transforms.h"
#include "Eigen/Dense"

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include "QColor"
#include "QVector3D"

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>

SerialArmViz::SerialArmViz()
{
    _arm = SerialArm();
    _n = 1;
}

SerialArmViz::SerialArmViz(SerialArm arm, Qt3DCore::QEntity* parent)
{
    _arm = arm;
    _n = arm.get_n();

    Qt3DExtras::QCuboidMesh *cubeMesh;
    cubeMesh = new Qt3DExtras::QCuboidMesh();
    double size{20};
    cubeMesh->setXExtent(size * 1);
    cubeMesh->setYExtent(size * 1);
    cubeMesh->setZExtent(size * 1);

    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(QColor(QColor::fromRgb(255, 0, 0)));

    Eigen::VectorXd q = Eigen::VectorXd::Zero(_n);

    double s = 50;

    for (int i = 0; i < _n; i++)
    {
        SE3 T = _arm.fk(q, i + 1);
        Eigen::Vector3d p = T.get_p();
        Eigen::Vector4d quat = T.get_R().get_quat();
        _transformList.push_back(new Qt3DCore::QTransform());
        _transformList[i]->setTranslation(QVector3D(p(0) * s, p(1) * s, p(2) * s));
        _transformList[i]->setRotation(QQuaternion(quat(0), quat(1), quat(2), quat(3)));
        _transformList[i]->setScale(1);

        _jointList.push_back(new Qt3DCore::QEntity());
        _jointList[i]->addComponent(cubeMesh);
        _jointList[i]->addComponent(_transformList[i]);
        _jointList[i]->addComponent(material);
        _jointList[i]->setEnabled(true);
        _jointList[i]->setParent(parent);
    }
}

void SerialArmViz::update(Eigen::Vector4d q)
{
    double s = 50;
    for (int i = 0; i < _n; i++)
    {
        SE3 T = _arm.fk(q, i + 1);
        Eigen::Vector3d p = T.get_p();
        Eigen::Vector4d quat = T.get_R().get_quat();

        _transformList[i]->setTranslation(QVector3D(p(0) * s, p(1) * s, p(2) * s));
        _transformList[i]->setRotation(QQuaternion(quat(0), quat(1), quat(2), quat(3)));
    }
}
