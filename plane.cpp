#include "plane.h"

#include <QColor>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>

Plane::Plane(Qt3DCore::QEntity *parent)
{
    _parent = parent;

    _mesh = new Qt3DExtras::QPlaneMesh();
    _mesh->setWidth(500);
    _mesh->setHeight(500);

    _material = new Qt3DExtras::QPhongMaterial();
    _color = QColor(200, 200, 200, 255);
    _material->setDiffuse(_color);

    _transform = new Qt3DCore::QTransform();
    _transform->setScale(1.0);
    _transform->setRotationX(90.0);
    _transform->setTranslation(QVector3D(0, 0, -100));

    _entity = new Qt3DCore::QEntity(parent);
    _entity->addComponent(_mesh);
    _entity->addComponent(_material);
    _entity->addComponent(_transform);

    _entity->setEnabled(true);
}
