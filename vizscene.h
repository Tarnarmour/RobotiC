#ifndef VIZSCENE_H
#define VIZSCENE_H

#include "kinematics.h"
#include "serialarmviz.h"

#include <QtCore>
#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DCore/qentity.h>
#include <QWidget>


class VizScene : public QWidget
{
    Q_OBJECT
public:
    explicit VizScene(QWidget *parent = 0, SerialArm arm = SerialArm());
    void update();
public slots:

private:
    Qt3DExtras::Qt3DWindow *window = nullptr;
    Qt3DCore::QEntity *root;
    SerialArm _arm;
    SerialArmViz _viz;
};


#endif // VIZSCENE_H
