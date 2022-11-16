#ifndef VIZSCENE_H
#define VIZSCENE_H

#include "se3viz.h"

#include <QtCore>
#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DCore/qentity.h>
#include <QWidget>


class VizScene : public QWidget
{
    Q_OBJECT
public:
    explicit VizScene(QWidget *parent = 0);

public slots:

private:
    Qt3DExtras::Qt3DWindow *window = nullptr;
    Qt3DCore::QEntity *root;


};


#endif // VIZSCENE_H
