#include "se3viz.h"

#include "QColor"

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>


SE3Viz::SE3Viz()
{
    Qt3DExtras::QCuboidMesh *cubeMesh = new Qt3DExtras::QCuboidMesh();

    transform = new Qt3DCore::QTransform();
    entity = new Qt3DCore::QEntity();
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(QColor(QColor::fromRgb(1, 0, 0)));

    entity->addComponent(cubeMesh);
    entity->addComponent(transform);
    entity->addComponent(material);
    entity->setEnabled(true);
}
