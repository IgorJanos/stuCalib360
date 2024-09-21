#ifndef EXPORTDATASETDIALOG_H
#define EXPORTDATASETDIALOG_H

#include <QDialog>

#include <QFuture>
#include <QFutureWatcher>


#include <QLabel>
#include <QScrollArea>
#include <QOffscreenSurface>
#include <QStringListModel>
#include <QSet>

namespace Ui {
class ExportDatasetDialog;
}


class CheckedImagesModel : public QAbstractListModel
{
    Q_OBJECT

protected:

    DatasetModel          *inner;
    QSet<QModelIndex>     checkedItems;

public:
    CheckedImagesModel(DatasetModel *ainner, QObject *parent=0);

    Qt::ItemFlags flags(const QModelIndex& index) const;
    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role);

};




class ExportDatasetDialog : public QDialog
{
    Q_OBJECT

private:

    DatasetModel                    *datasetModel;
    CheckedImagesModel              *checkedModel;
    PlayfieldCollectionModel        *playfieldModel;
    int                             inputImages;

    // Exportovaci proces
    QOffscreenSurface                                   *offs;
    QSharedPointer<Exporter::ExportProcessState>        exportState;
    QFutureWatcher<bool>                                *exportProcess;
    QTimer                                              *timerProgress;
    QDateTime                                           timeStart;

    // Controls
    QLabel          *imageLabel;
    QScrollArea     *scrollArea;

public:
    explicit ExportDatasetDialog(QWidget *parent = nullptr);
    ~ExportDatasetDialog();

    void setupDialog(
            DatasetModel *dsmodel, PlayfieldCollectionModel *pfcm
            );


private:
    Ui::ExportDatasetDialog *ui;

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

#endif // EXPORTDATASETDIALOG_H
