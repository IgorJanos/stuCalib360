#ifndef EXPORTCALIBRATIONDIALOG_H
#define EXPORTCALIBRATIONDIALOG_H

#include <QDialog>

namespace Ui {
class ExportCalibrationDialog;
}

class ExportCalibrationDialog : public QDialog
{
    Q_OBJECT

private:

    DatasetModel                    *datasetModel;
    CheckedImagesModel              *checkedModel;
    PlayfieldCollectionModel        *playfieldModel;
    int                             inputImages;

    // Exportovaci proces
    QOffscreenSurface                                   *offs;
    QSharedPointer<Exporter::ExportCalibrationProcessState> exportState;
    QFutureWatcher<bool>                                *exportProcess;
    QTimer                                              *timerProgress;
    QDateTime                                           timeStart;

    // Controls
    QLabel          *imageLabel;
    QScrollArea     *scrollArea;


public:
    explicit ExportCalibrationDialog(QWidget *parent = nullptr);
    ~ExportCalibrationDialog();

    void setupDialog(
        DatasetModel *dsmodel,
        PlayfieldCollectionModel *pfcm
        );

private:
    Ui::ExportCalibrationDialog *ui;

    void onSpinEditChanged(int value);

    void onStartExportClick();
    void onAbortClick();

    // Exportovaci process
    void onExportProcessStarted();
    void onExportProcessFinished();
    void onUpdateProgress();

    void updateInputImages();
    void onButtonAllClick();
    void onButtonNoneClick();

};

#endif // EXPORTCALIBRATIONDIALOG_H
