#define _USE_MATH_DEFINES
#include <cmath> // this has to be at the top for some inane reason

#include "sliderbar.h"
#include "serialarmviz.h"
#include <vector>

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QColor>
#include <QSlider>
#include <QLineEdit>
#include <QIntValidator>
#include <QString>

SliderBar::SliderBar()
{

}

SliderBar::SliderBar(SerialArmViz viz)
{
    _viz = viz;
    _n = viz._n;
    QVBoxLayout *sliderLayout = new QVBoxLayout();

    for (int i = 0; i < _n; i++)
    {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(-180);
        slider->setMaximum(180);
        slider->setValue(0);
//        QSlider::connect(slider, SIGNAL(valueChanged()), this, SLOT(update_from_slider()));
        QObject::connect(slider, &QSlider::valueChanged, this, &SliderBar::update_from_slider);

        _sliderList.push_back(slider);

        QLineEdit *lineEdit = new QLineEdit();
        QIntValidator *validate = new QIntValidator(-180, 180);
        lineEdit->setText("0");
        lineEdit->setValidator(validate);
        QLineEdit::connect(lineEdit, &QLineEdit::textEdited, this, &SliderBar::update_from_slider);

        _textList.push_back(lineEdit);

        sliderLayout->addWidget(lineEdit);
        sliderLayout->addWidget(slider);
    }

    this->setLayout(sliderLayout);
}

void SliderBar::update_from_slider()
{
    Eigen::VectorXd q{_n};
    for (int i = 0; i < _n; i ++)
    {
        int v{_sliderList[i]->value()};
        QString s = QString::fromStdString(std::to_string(v));
        _textList[i]->setText(s);
        q(i) = v * M_PI / 180.0;
    }

    _viz.update(q);
}

void SliderBar::update_from_text()
{
    Eigen::VectorXd q{_n};
    for (int i = 0; i < _n; i ++)
    {
        QString s = _textList[i]->text();
        _sliderList[i]->setValue(s.toInt());

        q(i) = s.toInt() * M_PI / 180.0;
    }

    _viz.update(q);
}
