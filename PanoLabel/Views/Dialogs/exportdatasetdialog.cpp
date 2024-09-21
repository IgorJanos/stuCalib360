#include "exportdatasetdialog.h"
#include "ui_exportdatasetdialog.h"

#include <QtConcurrent>

// test


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



ExportDatasetDialog::ExportDatasetDialog(QWidget *parent) :
    QDialog(parent),
    datasetModel(nullptr),
    checkedModel(nullptr),
    inputImages(0),    
    offs(nullptr),
    timerProgress(nullptr),
    imageLabel(nullptr),
    scrollArea(nullptr),
    ui(new Ui::ExportDatasetDialog)
{
    ui->setupUi(this);

    connect(ui->spinPerImage, &QSpinBox::valueChanged, this, &ExportDatasetDialog::onSpinEditChanged);
    connect(ui->spinCycles, &QSpinBox::valueChanged, this, &ExportDatasetDialog::onSpinEditChanged);

    connect(ui->buttonStart, &QPushButton::clicked, this, &ExportDatasetDialog::onStartExportClick);
    connect(ui->buttonAbort, &QPushButton::clicked, this, &ExportDatasetDialog::onAbortClick);

    connect(ui->buttonAll, &QPushButton::clicked, this, &ExportDatasetDialog::onButtonAllClick);
    connect(ui->buttonNone, &QPushButton::clicked, this, &ExportDatasetDialog::onButtonNoneClick);

    // Timer progress
    timerProgress = new QTimer(this);
    connect(timerProgress, &QTimer::timeout, this, &ExportDatasetDialog::onUpdateProgress);

    // Future watcher
    exportProcess = new QFutureWatcher<bool>(this);
    connect(exportProcess, &QFutureWatcher<bool>::started, this, &ExportDatasetDialog::onExportProcessStarted);
    connect(exportProcess, &QFutureWatcher<bool>::finished, this, &ExportDatasetDialog::onExportProcessFinished);

    // Nic nam nebezi ...
    onExportProcessFinished();

}

ExportDatasetDialog::~ExportDatasetDialog()
{
    if (offs) {
        delete offs;
        offs = nullptr;
    }

    delete ui;
}

void ExportDatasetDialog::updateInputImages()
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

void ExportDatasetDialog::onButtonAllClick()
{
    if (checkedModel) {
        for (int i=0; i<checkedModel->rowCount(); i++) {
            checkedModel->setData(checkedModel->index(i, 0), Qt::Checked, Qt::CheckStateRole);
        }
    }

    updateInputImages();
}

void ExportDatasetDialog::onButtonNoneClick()
{
    if (checkedModel) {
        for (int i=0; i<checkedModel->rowCount(); i++) {
            checkedModel->setData(checkedModel->index(i, 0), Qt::Unchecked, Qt::CheckStateRole);
        }
    }

    updateInputImages();
}


void ExportDatasetDialog::setupDialog(
            DatasetModel *dsmodel, PlayfieldCollectionModel *pfcm
        )
{
    this->setWindowTitle("Export Dataset ...");
    this->datasetModel = dsmodel;
    this->playfieldModel = pfcm;

    // Spocitame kolko mame zmapovanych obrazkov
    inputImages = 0;
    if (dsmodel) {
        // Dame si novy model
        checkedModel = new CheckedImagesModel(dsmodel);
        connect(checkedModel, &CheckedImagesModel::dataChanged, this, &ExportDatasetDialog::updateInputImages);

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
    ui->comboScale->setCurrentIndex(5);

    // Defaults
    Exporter::CropArgs          defaultArgs;

    ui->editPanLow->setText(QString::number(defaultArgs.rangePan.first));
    ui->editPanHigh->setText(QString::number(defaultArgs.rangePan.second));
    ui->editTiltLow->setText(QString::number(defaultArgs.rangeTilt.first));
    ui->editTiltHigh->setText(QString::number(defaultArgs.rangeTilt.second));
    ui->editRollLow->setText(QString::number(defaultArgs.rangeRoll.first));
    ui->editRollHigh->setText(QString::number(defaultArgs.rangeRoll.second));
    ui->editFOVLow->setText(QString::number(defaultArgs.rangeFOV.first));
    ui->editFOVHigh->setText(QString::number(defaultArgs.rangeFOV.second));

    ui->checkDistortion->setChecked(defaultArgs.useDistortion);
    ui->editK1Low->setText(QString::number(defaultArgs.rangeK1.first));
    ui->editK1High->setText(QString::number(defaultArgs.rangeK1.second));
    ui->editK2NosieVar->setText(QString::number(defaultArgs.rangeK2.second));

    ui->spinPerImage->setValue(50);
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

void ExportDatasetDialog::onSpinEditChanged(int value)
{
    Q_UNUSED(value)

    int     perImage = ui->spinPerImage->value();
    int     cycles = ui->spinCycles->value();

    int     outCount = inputImages * perImage * cycles;

    // Dame spocitat
    ui->labelInputValue->setText(QString::number(inputImages));
    ui->labelOutputValue->setText(QString::number(outCount));
}


static bool executeExportProcess(
                QSharedPointer<Exporter::PipelineSource<Exporter::Image>> source,
                QSharedPointer<Exporter::CropRenderer> renderer,
                Exporter::CropArgs cropArgs,
                QSharedPointer<Exporter::DatasetSink> sink,
                QSharedPointer<Exporter::ExportProcessState> state
            )
{

    //----------------------------------------------
    //  Exporting process
    //----------------------------------------------

    // Skusime daco porobit
    source->reset();
    while (source->hasCurrent()) {

        // Ak nas abortuju, tak koncime ...
        if (state->shouldAbort()) return false;

        auto inputImage = source->current();
        if (inputImage) {

            // Nasamplujeme view
            Exporter::CropSample    crop = Exporter::CropSample::Random(cropArgs);

            // Zrenderujeme
            auto image = renderer->render(inputImage, crop);
            state->setCurrentFrame(image);

            // Tlacime do vystupu
            sink->write(image, crop);
        }

        // Hlasime dokoncenie kroku
        state->next();
        source->next();
    }

    return true;
}


template<class T>
static void parseRange(std::pair<float, float> &range, T *c1, T *c2)
{
    range.first = c1->text().toFloat();
    range.second = c2->text().toFloat();
}

void ExportDatasetDialog::onStartExportClick()
{

    // Nas stavovy objekt
    int     inputCount = this->inputImages;
    int     cycles = ui->spinCycles->value();
    int     perImage = ui->spinPerImage->value();
    int     totalCount = inputCount * cycles * perImage;
    exportState = makeNew<Exporter::ExportProcessState>(totalCount);

    // Source pipeline
    auto s1 = makeNew<Exporter::DatasetImageSource>(datasetModel, checkedModel, playfieldModel);
    auto s2 = makeNew<Exporter::Repeater>(s1, perImage);
    auto s3 = makeNew<Exporter::CycleCounter>(s2, cycles);

    // View data
    Exporter::CropArgs      cropArgs;
    parseRange(cropArgs.rangePan, ui->editPanLow, ui->editPanHigh);
    parseRange(cropArgs.rangeTilt, ui->editTiltLow, ui->editTiltHigh);
    parseRange(cropArgs.rangeRoll, ui->editRollLow, ui->editRollHigh);
    parseRange(cropArgs.rangeFOV, ui->editFOVLow, ui->editFOVHigh);
    cropArgs.useDistortion = ui->checkDistortion->isChecked();
    parseRange(cropArgs.rangeK1, ui->editK1Low, ui->editK1High);
    cropArgs.rangeK2.first = 0.0;
    cropArgs.rangeK2.second = ui->editK2NosieVar->text().toFloat();
    cropArgs.useColorAdjust = ui->checkColorAdjust->isChecked();

    // renderer
    RESOLUTION  rs = RenderSizes[ui->comboRender->currentIndex()];
    auto renderer = makeNew<Exporter::CropRenderer>(offs, QSize(rs.w, rs.h));

    // sink
    RESOLUTION  ss = ScaleSizes[ui->comboScale->currentIndex()];

    // Meno datasetu
    QString     basePath = QDir::homePath() + "/.Playfield/Export";
    QString     expName = ui->editName->text();
    if (ui->checkTimestamp->isChecked()) {
        // pridame datum a cas
        QDateTime       tnow = QDateTime::currentDateTime();
        QString         st = tnow.toString("-yyMMdd-hhmmss");
        expName += st;
    }
    basePath += "/" + expName;

    auto sink = makeNew<Exporter::DatasetSink>(
                basePath, totalCount, ss.w, ss.h,
                ui->checkLabels->isChecked()
                );

    if (exportProcess) {

        // Nakopneme exportovanie
        exportProcess->setFuture(
                    QtConcurrent::run(
                        executeExportProcess,
                        s3, renderer, cropArgs, sink, exportState
                        )
                    );
    }
}

void ExportDatasetDialog::onAbortClick()
{
    if (exportProcess->isRunning()) {

        // skusime ho zrusit
        exportState->abort();
        exportProcess->waitForFinished();
    }
}


void ExportDatasetDialog::onExportProcessStarted()
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

void ExportDatasetDialog::onExportProcessFinished()
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


void ExportDatasetDialog::onUpdateProgress()
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
            imageLabel->setPixmap(QPixmap::fromImage(image->image));
        }
    }
}


//-----------------------------------------------------------------------------
//  CheckedImagesModel class
//-----------------------------------------------------------------------------

CheckedImagesModel::CheckedImagesModel(
        DatasetModel *ainner, QObject *parent
    ) :
    QAbstractListModel(parent),
    inner(ainner)
{
}


Qt::ItemFlags CheckedImagesModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags defaultFlags = Qt::NoItemFlags;
    if (index.isValid()){
        return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
    }
    return defaultFlags;
}

int CheckedImagesModel::rowCount(const QModelIndex& parent) const
{
    return inner->rowCount(parent);
}

int CheckedImagesModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)
    return 1;
}

QVariant CheckedImagesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if(role == Qt::CheckStateRole)
        return (checkedItems.contains(index) ? Qt::Checked : Qt::Unchecked);

    return inner->data(index, role);
}

bool CheckedImagesModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if(!index.isValid() || role != Qt::CheckStateRole)
        return false;

    if(value == Qt::Checked)
        checkedItems.insert(index);
    else
        checkedItems.remove(index);

    emit dataChanged(index, index);
    return true;
}
















