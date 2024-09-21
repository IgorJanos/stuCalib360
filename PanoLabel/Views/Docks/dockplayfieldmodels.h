#ifndef DOCKPLAYFIELDMODELS_H
#define DOCKPLAYFIELDMODELS_H

#include <QDockWidget>
#include <QListView>
#include <QStandardItemModel>


namespace Ui {
class DockPlayfieldModels;
}


class IPlayfieldCollectionModelSource
{
public:
    virtual PlayfieldCollectionModel *getPlayfieldCollectionModel() = 0;
};



class DockPlayfieldModels :
        public QDockWidget,
        public IPlayfieldCollectionModelSource
{
    Q_OBJECT

public:
    explicit DockPlayfieldModels(QWidget *parent = nullptr);
    ~DockPlayfieldModels();

    // IPlayfieldCollectionModelSource
    PlayfieldCollectionModel *getPlayfieldCollectionModel();

private:
    Ui::DockPlayfieldModels *ui;
    QListView *listModels;

    QString         modelsPath;

    void setupView();

    void onNewModelClick();
    void onEditModelClick();
    void onDeleteModelClick();
};

#endif // DOCKPLAYFIELDMODELS_H
