#include "se3viz.h"

#include "QColor"
#include "QVector3D"

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>


SE3Viz::SE3Viz()
{
    Qt3DExtras::QCuboidMesh *cubeMesh = new Qt3DExtras::QCuboidMesh();
    cubeMesh->setXExtent(1.0);
    cubeMesh->setYExtent(1.0);
    cubeMesh->setZExtent(1.0);

    transform = new Qt3DCore::QTransform();
    transform->setTranslation(QVector3D(0.0, 0.0, 0.0));
    transform->setScale(1.0);
    entity = new Qt3DCore::QEntity();
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(QColor(QColor::fromRgb(255, 0, 0)));

    entity->addComponent(cubeMesh);
    entity->addComponent(transform);
    entity->addComponent(material);
    entity->setEnabled(true);
}
