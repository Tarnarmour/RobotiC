// My stuff
#include "vizscene.h"

// Basic QT stuff
#include <QWidget>
#include <QHBoxLayout>

// Qt 3D stuff
#include <Qt3DCore/qattribute.h>
#include <Qt3DCore/qbuffer.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DExtras/qorbitcameracontroller.h>

#include <Qt3DRender/qcameralens.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DRender/qcamera.h>


VizScene::VizScene(QWidget *parent) :
    QWidget{parent}
{
    window = new Qt3DExtras::Qt3DWindow();
    QHBoxLayout *layout = new QHBoxLayout();
    QWidget *container = QWidget::createWindowContainer(window);
    container->setMinimumSize(600, 300);
    layout->addWidget(container);
    this->setLayout(layout);

    window->defaultFrameGraph()->setClearColor(QColor(QColor::fromRgb(0.0, 0.0, 0.0)));

    root = new Qt3DCore::QEntity();


    // Camera
    Qt3DRender::QCamera *cameraEntity = window->camera();

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(200.0f, 0, 75));
    cameraEntity->setUpVector(QVector3D(0, 0, 1.0f));
    cameraEntity->setViewCenter(QVector3D(0, 0, 50));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(root);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(.9f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(200.0f, 200.0f, 200.0f));
    lightEntity->addComponent(lightTransform);

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(root);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(cameraEntity);

     window->setRootEntity(root);

     SE3Viz *frame = new SE3Viz();
     frame->entity->setParent(root);
}
