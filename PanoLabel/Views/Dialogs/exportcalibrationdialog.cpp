#include "exportcalibrationdialog.h"
#include "ui_exportcalibrationdialog.h"

#include <QtConcurrent>


struct RESOLUTION {
    int     w;
    int     h;
};

const RESOLUTION    RenderSizes[] = {
    { 1920, 1080 },
    { 1280, 720 },
    { 960, 540 },
    { 640, 360 }
};

const RESOLUTION    ScaleSizes[] = {
    { 1920, 1080 },
    { 1280, 720 },
    { 960, 540 },
    { 640, 360 },
    { 512, 512 },
    { 448, 448 },
    { 224, 224 }
};

#define COUNTOF(x)      (sizeof(x) / (sizeof(x[0])))



ExportCalibrationDialog::ExportCalibrationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExportCalibrationDialog)
{
    ui->setupUi(this);

    connect(ui->spinPerImage, &QSpinBox::valueChanged, this, &ExportCalibrationDialog::onSpinEditChanged);
    connect(ui->spinCycles, &QSpinBox::valueChanged, this, &ExportCalibrationDialog::onSpinEditChanged);

    connect(ui->buttonStart, &QPushButton::clicked, this, &ExportCalibrationDialog::onStartExportClick);
    connect(ui->buttonAbort, &QPushButton::clicked, this, &ExportCalibrationDialog::onAbortClick);

    connect(ui->buttonAll, &QPushButton::clicked, this, &ExportCalibrationDialog::onButtonAllClick);
    connect(ui->buttonNone, &QPushButton::clicked, this, &ExportCalibrationDialog::onButtonNoneClick);

    // Timer progress
    timerProgress = new QTimer(this);
    connect(timerProgress, &QTimer::timeout, this, &ExportCalibrationDialog::onUpdateProgress);

    // Future watcher
    exportProcess = new QFutureWatcher<bool>(this);
    connect(exportProcess, &QFutureWatcher<bool>::started, this, &ExportCalibrationDialog::onExportProcessStarted);
    connect(exportProcess, &QFutureWatcher<bool>::finished, this, &ExportCalibrationDialog::onExportProcessFinished);

    // Nic nam nebezi ...
    onExportProcessFinished();

}

ExportCalibrationDialog::~ExportCalibrationDialog()
{
    if (offs) {
        delete offs;
        offs = nullptr;
    }

    delete ui;
}


void ExportCalibrationDialog::setupDialog(
    DatasetModel *dsmodel,
    PlayfieldCollectionModel *pfcm
    )
{
    this->setWindowTitle("Export Calibration Dataset ...");
    this->datasetModel = dsmodel;
    this->playfieldModel = pfcm;

    // Spocitame kolko mame zmapovanych obrazkov
    inputImages = 0;
    if (dsmodel) {
        // Dame si novy model
        checkedModel = new CheckedImagesModel(dsmodel);
        connect(checkedModel, &CheckedImagesModel::dataChanged, this, &ExportCalibrationDialog::updateInputImages);

        ui->listImages->setModel(checkedModel);

        // Update inputs
        onButtonAllClick();
        if (dsmodel->Dataset()) {
            ui->editName->setText(dsmodel->Dataset()->datasetName);
        }
    }

    ui->checkTimestamp->setChecked(true);


    // DropDowns
    ui->comboRender->clear();
    for (int i=0; i<(int)COUNTOF(RenderSizes); i++) {
        ui->comboRender->addItem(QString("%1 x %2").arg(RenderSizes[i].w).arg(RenderSizes[i].h));
    }
    ui->comboRender->setCurrentIndex(0);

    ui->comboScale->clear();
    for (int i=0; i<(int)COUNTOF(ScaleSizes); i++) {
        ui->comboScale->addItem(QString("%1 x %2").arg(ScaleSizes[i].w).arg(ScaleSizes[i].h));
    }
    ui->comboScale->setCurrentIndex(3);

    // Defaults
    Exporter::CalibrationSampleArgs     defaultArgs;
    ui->editRollLow->setText(QString::number(defaultArgs.rangeRoll.first));
    ui->editRollHigh->setText(QString::number(defaultArgs.rangeRoll.second));
    ui->editFOVLow->setText(QString::number(defaultArgs.rangeFOV.first));
    ui->editFOVHigh->setText(QString::number(defaultArgs.rangeFOV.second));

    ui->spinPerImage->setValue(1);
    ui->spinCycles->setValue(1);

    // Preview area
    scrollArea = new QScrollArea();
    if (scrollArea) {

        scrollArea->setBackgroundRole(QPalette::Dark);

        auto layout = new QGridLayout();
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);

        imageLabel = new QLabel();
        imageLabel->setBackgroundRole(QPalette::Base);
        imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        imageLabel->setScaledContents(true);

        scrollArea->setWidget(imageLabel);
        scrollArea->setWidgetResizable(true);

        ui->frame->setLayout(layout);
        ui->frame->layout()->addWidget(scrollArea);
    }

    // Offscreen rendering
    offs = new QOffscreenSurface();
    offs->create();

}

void ExportCalibrationDialog::updateInputImages()
{
    inputImages = 0;

    auto dsmodel = datasetModel;
    auto chmodel = checkedModel;
    if (dsmodel && chmodel) {

        for (int i=0; i<dsmodel->rowCount(); i++) {
            if (dsmodel->hasMapping(dsmodel->index(i, 0)) &&
                chmodel->data(chmodel->index(i,0), Qt::CheckStateRole) == Qt::Checked
                ) {
                inputImages ++;
            }
        }

    }

    onSpinEditChanged(0);
}


void ExportCalibrationDialog::onButtonAllClick()
{
    if (checkedModel) {
        for (int i=0; i<checkedModel->rowCount(); i++) {
            checkedModel->setData(checkedModel->index(i, 0), Qt::Checked, Qt::CheckStateRole);
        }
    }

    updateInputImages();
}

void ExportCalibrationDialog::onButtonNoneClick()
{
    if (checkedModel) {
        for (int i=0; i<checkedModel->rowCount(); i++) {
            checkedModel->setData(checkedModel->index(i, 0), Qt::Unchecked, Qt::CheckStateRole);
        }
    }

    updateInputImages();
}

void ExportCalibrationDialog::onSpinEditChanged(int value)
{
    Q_UNUSED(value)

    int     perImage = ui->spinPerImage->value();
    int     cycles = ui->spinCycles->value();

    int     outCount = inputImages * perImage * cycles;

    // Dame spocitat
    ui->labelInputValue->setText(QString::number(inputImages));
    ui->labelOutputValue->setText(QString::number(outCount));
}

void ExportCalibrationDialog::onAbortClick()
{
    if (exportProcess->isRunning()) {

        // skusime ho zrusit
        exportState->abort();
        exportProcess->waitForFinished();
    }
}

void ExportCalibrationDialog::onExportProcessStarted()
{
    qDebug() << "Started";

    ui->buttonStart->setEnabled(false);
    ui->buttonAbort->setEnabled(true);

    // odmeriame cas zaciatku
    timeStart = QDateTime::currentDateTime();
    onUpdateProgress();
    timerProgress->start(100);
}


static QString formatTimeSpan(qint64 ms)
{
    QTime t = QTime(0, 0).addMSecs(ms);
    return t.toString("HH:mm:ss");
}

void ExportCalibrationDialog::onExportProcessFinished()
{
    qDebug() << "Finished";

    timerProgress->stop();
    onUpdateProgress();

    ui->buttonStart->setEnabled(true);
    ui->buttonAbort->setEnabled(false);
    ui->progressBar->setMaximum(100);
    ui->progressBar->setValue(0);

    ui->labelElapsed->setText(formatTimeSpan(0));
    ui->labelRemaining->setText(formatTimeSpan(0));
}


void ExportCalibrationDialog::onUpdateProgress()
{
    // ideme aktualizovat progressbar
    if (exportState) {

        QDateTime   tNow = QDateTime::currentDateTime();
        qint64      elapsedMs = timeStart.msecsTo(tNow);
        qint64      remainingMs = 0;

        int     total, progress;
        exportState->getProgress(progress, total);

        if (total > 0) {
            ui->progressBar->setMaximum(total);
            ui->progressBar->setValue(progress);

            // spocitame pomer, kolko sme urobili
            double rate = (double)progress / (double)total;
            double totalMs = 0;
            if (rate > 0) {
                totalMs = elapsedMs / rate;
            }
            remainingMs = totalMs - elapsedMs;
            if (remainingMs < 0) remainingMs = 0;

        } else {
            ui->progressBar->setMaximum(100);
            ui->progressBar->setValue(0);

            elapsedMs = 0;
            remainingMs = 0;
        }

        // Formatujeme
        ui->labelElapsed->setText(formatTimeSpan(elapsedMs));
        ui->labelRemaining->setText(formatTimeSpan(remainingMs));

        // este aj obrazok
        auto image = exportState->getCurrentFrame();
        if (imageLabel && image) {
            imageLabel->setPixmap(QPixmap::fromImage(image->imagePlayfield));
        }
    }
}



static bool executeExportCalibrationProcess(
    QSharedPointer<Exporter::PipelineSource<Exporter::Image>> source,
    QSharedPointer<Exporter::ExportCalibrationProcessState> state,
    QSharedPointer<Exporter::CalibrationRenderer> renderer,
    Exporter::CalibrationSampleArgs sampleArgs,
    QSharedPointer<Exporter::PlayfieldGridImages> gridImages,
    QSharedPointer<Exporter::CalibrationDatasetSink> sink
    )
{
    //-----------------------------------------
    //  Exporting process
    //-----------------------------------------

    bool    is_aborting = false;

    sink->start();

    source->reset();
    while (source->hasCurrent() && (!is_aborting)) {

        // Abortuju nas ?
        if (state->shouldAbort()) {
            is_aborting = true;
        } else {

            auto inputImage = source->current();
            if (inputImage) {

                // Samplujeme a renderujeme !
                Exporter::CalibrationSample         sample = Exporter::CalibrationSample::Random(
                        sampleArgs,
                        inputImage->mappingCalibration->mean_position
                    );
                Exporter::CalibrationCameraPose     cameraPose;
                Exporter::computeCameraPose(inputImage, sample, cameraPose);

                // Zrenderujeme
                auto image = renderer->render(inputImage, gridImages, cameraPose);
                state->setCurrentFrame(image);

                // Tlacime von
                sink->write(image, cameraPose);
            }

            // Dokoncenie kroku
            state->next();
            source->next();
        }
    }

    sink->stop();

    return true;
}


template<class T>
static void parseRange(std::pair<double, double> &range, T *c1, T *c2)
{
    range.first = c1->text().toFloat();
    range.second = c2->text().toFloat();
}


void ExportCalibrationDialog::onStartExportClick()
{
    // Nas stavovy objekt
    int     inputCount = this->inputImages;
    int     cycles = ui->spinCycles->value();
    int     perImage = ui->spinPerImage->value();
    int     totalCount = inputCount * cycles * perImage;
    exportState = makeNew<Exporter::ExportCalibrationProcessState>(totalCount);

    // Source pipeline
    auto s1 = makeNew<Exporter::DatasetImageSource>(datasetModel, checkedModel, playfieldModel);
    auto s2 = makeNew<Exporter::Repeater>(s1, perImage);
    auto s3 = makeNew<Exporter::CycleCounter>(s2, cycles);

    // Sampling Args
    Exporter::CalibrationSampleArgs     sampleArgs;
    parseRange(sampleArgs.rangeRoll, ui->editRollLow, ui->editRollHigh);
    parseRange(sampleArgs.rangeFOV, ui->editFOVLow, ui->editFOVHigh);
    sampleArgs.playfieldModel = this->playfieldModel->getPlayfieldModel(
            this->datasetModel->Dataset()->modelFilename
        );

    // Renderer
    RESOLUTION  rs = RenderSizes[ui->comboRender->currentIndex()];
    auto renderer = makeNew<Exporter::CalibrationRenderer>(offs, QSize(rs.w, rs.h));

    // Grid images
    QSharedPointer<Exporter::PlayfieldGridImages>       gridImages;
    gridImages = Exporter::PlayfieldGridImages::FromFolder(
        QDir::homePath() + "/.Playfield/Grids"
        );

    // sink
    RESOLUTION  ss = ScaleSizes[ui->comboScale->currentIndex()];

    // Meno datasetu
    QString     basePath = QDir::homePath() + "/.Playfield/ExportCalib";
    QString     expName = ui->editName->text();
    if (ui->checkTimestamp->isChecked()) {
        // pridame datum a cas
        QDateTime       tnow = QDateTime::currentDateTime();
        QString         st = tnow.toString("-yyMMdd-hhmmss");
        expName += st;
    }
    basePath += "/" + expName;

    auto sink = makeNew<Exporter::CalibrationDatasetSink>(
            basePath, totalCount, ss.w, ss.h
            );

    if (exportProcess) {

        // Nakopneme exportovanie
        exportProcess->setFuture(
            QtConcurrent::run(
                executeExportCalibrationProcess,
                s3, exportState,
                renderer,
                sampleArgs, gridImages,
                sink
                )
            );
    }

}








