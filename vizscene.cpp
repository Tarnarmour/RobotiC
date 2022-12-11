// My stuff
#include "vizscene.h"
#include "trackballcameracontroller.h"
#include "plane.h"
#include "sliderbar.h"

#include "Eigen/Dense"
#include "kinematics.h"
#include <vector>
#include "serialarmviz.h"

// Basic QT stuff
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QColor>
#include <QSlider>

// Qt 3D stuff
#include <Qt3DCore/qattribute.h>
#include <Qt3DCore/qbuffer.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DExtras/qorbitcameracontroller.h>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>

#include <Qt3DRender/qcameralens.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DRender/qcamera.h>


VizScene::VizScene(QWidget *parent, SerialArm arm) :
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
    window->setRootEntity(root);


    // Camera
    Qt3DRender::QCamera *cameraEntity = window->camera();

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 5000.0f);
    cameraEntity->setPosition(QVector3D(0.0f, 0, 500));
    cameraEntity->setUpVector(QVector3D(0, 0, 1.0f));
    cameraEntity->setViewCenter(QVector3D(0, 0, 50));

    // Lighting
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(root);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(.9f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(200.0f, 200.0f, 200.0f));
    lightEntity->addComponent(lightTransform);

    // Camera Controller
    TrackballCameraController *camController = new TrackballCameraController(root);
    //    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(root);
    camController->setCamera(cameraEntity);
    camController->setWindowSize(this->size());

    // Ground Plane
    Plane groundPlane(root);

    // Serial Arm
    _arm = arm;
    _viz = SerialArmViz(_arm, root);
    int n = _arm.get_n();

    // Slider Controllers
    SliderBar *controlPanel = new SliderBar(_viz);
    layout->addWidget(controlPanel);

//    Eigen::Vector4d q{0.01, 1.5, 0.01, -1.5};

//    _viz.update(q);

}

void VizScene::update()
{
    Eigen::VectorXd q = Eigen::VectorXd::Random(_arm.get_n());
    _viz.update(q);
}
