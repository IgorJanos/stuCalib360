#ifndef DOCKDATASET_H
#define DOCKDATASET_H

#include <QDockWidget>
#include <QTableView>


#include "Models/datasetcollectionmodel.h"
#include "Models/datasetmodel.h"


namespace Ui {
class DockDataset;
}





class DockDataset : public QDockWidget
{
    Q_OBJECT

public:
    explicit DockDataset(IPlayfieldCollectionModelSource *pfcSource, QWidget *parent = nullptr);
    ~DockDataset();

private:
    Ui::DockDataset *ui;
    QTableView *tableImages;
    IPlayfieldCollectionModelSource *pfcSource;

    QString                         datasetPath;

    void setupView();

    void onDatasetFolderChanged(const QString &text);
    void onImagesDoubleClicked(const QModelIndex &index);

    void onNewDatasetClick();
    void onEditDatasetClick();
    void onDeleteDatasetClick();

public:

    QSharedPointer<Data::Dataset> getCurrentDataset();
    DatasetModel *getDatasetModel();

signals:
    void onFileClosed();
    void onFileDoubleClicked(QString filePath, QSharedPointer<Data::ImageMapping> mapping);

};

#endif // DOCKDATASET_H
