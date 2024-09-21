#include "dockview.h"
#include "ui_dockview.h"

DockView::DockView(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::DockView)
{
    ui->setupUi(this);

    connect(ui->sliderK1, &QSlider::valueChanged, this, &DockView::onSliderK1ValueChanged);
    connect(ui->sliderK2, &QSlider::valueChanged, this, &DockView::onSliderK2ValueChanged);
    connect(ui->checkLabel, &QCheckBox::clicked, this, &DockView::onCheckChanged);
    connect(ui->checkFinal, &QCheckBox::clicked, this, &DockView::onCheckChanged);
    connect(ui->checkGrid, &QCheckBox::clicked, this, &DockView::onCheckChanged);

    ui->checkLabel->setChecked(true);
    ui->checkFinal->setChecked(true);
    ui->checkGrid->setChecked(true);
}

DockView::~DockView()
{
    delete ui;
}



void DockView::onSliderK1ValueChanged(int value)
{
    Q_UNUSED(value)
    PinholeViewPort &vp = PinholeViewPort::get();
    auto range = vp.rangeK1;
    auto slider = ui->sliderK1;

    float v = range.first + (
                (range.second - range.first) *
                (slider->value()   - slider->minimum())/
                (slider->maximum() - slider->minimum())
                );

    // Nastavime
    vp.k.setX(v);
    ui->labelK1value->setText(QString("%1").arg(v));

    // signal von
    emit viewChanged();

}

void DockView::onSliderK2ValueChanged(int value)
{
    Q_UNUSED(value)
    PinholeViewPort &vp = PinholeViewPort::get();
    auto range = vp.rangeK2;
    auto slider = ui->sliderK2;

    float v = range.first + (
                (range.second - range.first) *
                (slider->value()   - slider->minimum())/
                (slider->maximum() - slider->minimum())
                );

    // Nastavime
    vp.k.setY(v);
    ui->labelK2value->setText(QString("%1").arg(v));

    // signal von
    emit viewChanged();
}

void DockView::onCheckChanged(bool value)
{
    Q_UNUSED(value)

    PinholeViewPort &vp = PinholeViewPort::get();

    vp.labelEnabled = ui->checkLabel->isChecked();
    vp.finalEnabled = ui->checkFinal->isChecked();
    vp.gridEnabled = ui->checkGrid->isChecked();

    // signal von
    emit viewChanged();
}
