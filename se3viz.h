#ifndef SE3VIZ_H
#define SE3VIZ_H

#include "transforms.h"

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>


class SE3Viz : public SE3
{
public:
    SE3Viz();

    Qt3DCore::QTransform *transform = nullptr;
    Qt3DCore::QEntity *entity = nullptr;
private:

};

#endif // SE3VIZ_H
