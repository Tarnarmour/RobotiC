#ifndef SLIDERBAR_H
#define SLIDERBAR_H

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QColor>
#include <QSlider>
#include <QLineEdit>

#include <vector>
#include <Eigen/Dense>
#include "serialarmviz.h"


class SliderBar : public QWidget
{
    Q_OBJECT

public:
    SliderBar();
    explicit SliderBar(SerialArmViz arm);
    int _n;
//    QWidget *_parent;
    std::vector<QSlider*> _sliderList;
    std::vector<QLineEdit*> _textList;
    SerialArmViz _viz;

    Eigen::VectorXd get_joint_values();

public slots:
    void update_from_slider();
    void update_from_text();
};

#endif // SLIDERBAR_H
