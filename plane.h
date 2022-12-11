#ifndef PLANE_H
#define PLANE_H


#include <QColor>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>


class Plane
{
public:
    Plane(Qt3DCore::QEntity *parent);
    Qt3DCore::QEntity *_parent;
    Qt3DExtras::QPlaneMesh *_mesh;
    Qt3DCore::QEntity *_entity;
    Qt3DCore::QTransform *_transform;
    Qt3DExtras::QPhongMaterial *_material;
    QColor _color;
};

#endif // PLANE_H
