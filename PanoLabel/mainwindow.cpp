#include "pch.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QImageReader>
#include <QtDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    dockPlayfieldModels(nullptr),
    dockDataset(nullptr)
{
    ui->setupUi(this);

    this->setWindowTitle("Panorama Labeling Tool");

    QIcon::setThemeName("breeze");
    QImageReader::setAllocationLimit(512 * 1024*1024);

    createMenus();
    createDockWindows();
    createPreviewer();

    QSettings settings("fiit", "panolabel");
    restoreGeometry(settings.value("geometry").toByteArray());
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::createMenus()
{
    QMenu       *menuView = menuBar()->addMenu("View");
    QMenu       *menuExport = menuBar()->addMenu("Export");

    //--------------------------------------
    //  View:

    QAction     *actViewPanorama = new QAction("View As Panorama", this);
    actViewPanorama->setStatusTip("Display 360 by 180 degrees equirectangular image");
    connect(actViewPanorama, &QAction::triggered, this, &MainWindow::onViewPanoramaClick);

    QAction     *actViewPinhole = new QAction("View As Pinhole", this);
    actViewPinhole->setStatusTip("Display pinhole camera rectilinear image");
    connect(actViewPinhole, &QAction::triggered, this, &MainWindow::onViewPinholeClick);

    //--------------------------------------
    //  Export:

    QAction     *actExportExport = new QAction("Export Dataset", this);
    actExportExport->setStatusTip("Export dataset folder");
    connect(actExportExport, &QAction::triggered, this, &MainWindow::onExportExportClick);

    QAction     *actExportExportCalibration = new QAction("Export Calibration Dataset", this);
    actExportExportCalibration->setStatusTip("Export calibration dataset folder");
    connect(actExportExportCalibration, &QAction::triggered, this, &MainWindow::onExportExportCalibrationClick);



    // add actions
    menuView->addAction(actViewPanorama);
    menuView->addAction(actViewPinhole);
    menuView->addSeparator();

    menuExport->addAction(actExportExport);
    menuExport->addAction(actExportExportCalibration);
}


void MainWindow::createDockWindows()
{
    dockPlayfieldModels = new DockPlayfieldModels(this);
    dockPlayfieldModels->setAllowedAreas(Qt::AllDockWidgetAreas);
    addDockWidget(Qt::LeftDockWidgetArea, dockPlayfieldModels);

    dockView = new DockView(this);
    dockView->setAllowedAreas(Qt::AllDockWidgetAreas);
    tabifyDockWidget(dockPlayfieldModels, dockView);

    dockDataset = new DockDataset(dockPlayfieldModels, this);
    dockDataset->setAllowedAreas(Qt::AllDockWidgetAreas);
    addDockWidget(Qt::LeftDockWidgetArea, dockDataset);

    dockPlayfieldModels->raise();

    connect(dockDataset, &DockDataset::onFileDoubleClicked, this, &MainWindow::onFileDoubleClicked);
    connect(dockDataset, &DockDataset::onFileClosed, this, &MainWindow::onFileClosed);
}

void MainWindow::createPreviewer()
{
    // Previewer
    multiPreviewer = new MultiPreviewer();
    if (multiPreviewer) {
        auto layout = new QGridLayout();
        ui->centralwidget->setLayout(layout);
        ui->centralwidget->layout()->addWidget(multiPreviewer);

        // Events
        connect(dockView, &DockView::viewChanged, multiPreviewer, &MultiPreviewer::onViewChanged);
    }
}


void MainWindow::onFileClosed()
{
    auto mp = as<IMappingPreviewer>(multiPreviewer);
    if (mp) {
        mp->closeMapping();
    }
}

void MainWindow::onFileDoubleClicked(QString filePath, QSharedPointer<Data::ImageMapping> mapping)
{
    auto mp = as<IMappingPreviewer>(multiPreviewer);
    if (mp) {
        mp->setMapping(filePath, mapping);
    }
}

void MainWindow::onViewPanoramaClick()
{
    multiPreviewer->setViewMode(ImageViewMode::Panorama);
}

void MainWindow::onViewPinholeClick()
{
    multiPreviewer->setViewMode(ImageViewMode::Pinhole);
}

void MainWindow::onExportExportClick()
{
    // Ideme exportovat
    ExportDatasetDialog     dialog(this);

    // Setup first!
    dialog.setupDialog(
                dockDataset->getDatasetModel(),
                dockPlayfieldModels->getPlayfieldCollectionModel()
                );

    if (dialog.exec() == QDialog::DialogCode::Accepted) {
        // TODO: ...
    }
}

void MainWindow::onExportExportCalibrationClick()
{
    // Ideme exportovat
    ExportCalibrationDialog     dialog(this);

    // Setup first!
    dialog.setupDialog(
        dockDataset->getDatasetModel(),
        dockPlayfieldModels->getPlayfieldCollectionModel()
        );
    if (dialog.exec() == QDialog::DialogCode::Accepted) {
        // TODO: ...
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QSettings settings("fiit", "panolabel");
    settings.setValue("geometry", saveGeometry());
    QWidget::closeEvent(event);
}


