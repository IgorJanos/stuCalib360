#include "pch.h"


//-----------------------------------------------------------------------------
//
//  MultiPreviewer class
//
//-----------------------------------------------------------------------------

MultiPreviewer::MultiPreviewer(QWidget *parent) :
    QWidget(parent),
    viewMode(ImageViewMode::Panorama),
    panoPreviewer(nullptr)
{
    // Initialize our previewers
    QGridLayout         *gl = new QGridLayout();
    setLayout(gl);
    gl->setContentsMargins(0, 0, 0, 0);
    gl->setSpacing(0);

    panoPreviewer = new MappingPreviewer(this);
    panoPreviewer->setFrameShape(QFrame::NoFrame);
    gl->addWidget(panoPreviewer);

    pinholePreviewer = new PinholePreviewer(this);
    gl->addWidget(pinholePreviewer);

    // Najskor ukazeme panoramu
    setViewMode(ImageViewMode::Pinhole);
}


void MultiPreviewer::setViewMode(ImageViewMode avalue)
{
    viewMode = avalue;

    // prehodime daco
    switch (viewMode) {
    case ImageViewMode::Panorama:
        pinholePreviewer->hide();
        panoPreviewer->show();
        break;

    case ImageViewMode::Pinhole:
        panoPreviewer->hide();
        pinholePreviewer->show();
        break;
    }
}


void MultiPreviewer::onViewChanged()
{
    if (pinholePreviewer) {
        pinholePreviewer->onViewChanged();
    }

}

//-----------------------------------------------------------------------------
//  IMappingPreviewer
//-----------------------------------------------------------------------------

void MultiPreviewer::setMapping(QString aimagePath, QSharedPointer<Data::ImageMapping> amapping)
{
    QImage img(aimagePath);

    if (panoPreviewer) {
        panoPreviewer->setMapping(img, amapping);
    }
    if (pinholePreviewer) {
        pinholePreviewer->setMapping(img, amapping);
    }
}

void MultiPreviewer::closeMapping()
{
    if (panoPreviewer) {
        panoPreviewer->closeMapping();
    }
}

