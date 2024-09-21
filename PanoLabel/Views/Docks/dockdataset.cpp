#include "pch.h"

#include "dockdataset.h"
#include "ui_dockdataset.h"


#include "Views/Dialogs/editdatasetdialog.h"

#include <QToolBar>
#include <QComboBox>
#include <QListView>
#include <QTableView>
#include <QHeaderView>
#include <QStyledItemDelegate>


DockDataset::DockDataset(
        IPlayfieldCollectionModelSource *apfcsource,
        QWidget *parent
        ) :
    QDockWidget(parent),
    ui(new Ui::DockDataset),
    tableImages(nullptr),
    pfcSource(apfcsource)
{
    datasetPath = QDir::homePath() + "/.Playfield/Datasets";

    ui->setupUi(this);
    setupView();
}

DockDataset::~DockDataset()
{
    delete ui;
}



void DockDataset::setupView()
{
    /*
     *  ToolBar:        Combo, New, Edit, Delete
     *  ListView:       Subory z datasetu
    */

    // Spravime layout
    QWidget         *content = new QWidget();
    QBoxLayout      *box = new QBoxLayout(QBoxLayout::TopToBottom, content);

    // ListView
    tableImages = new QTableView();
    connect(tableImages, &QTableView::doubleClicked,
            this, &DockDataset::onImagesDoubleClicked);
    tableImages->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableImages->setSelectionMode(QAbstractItemView::SingleSelection);
    tableImages->verticalHeader()->hide();
    tableImages->horizontalHeader()->hide();
    tableImages->setShowGrid(false);

    // Toolbar
    QToolBar        *toolbar = new QToolBar();

        // Toolbar items
        QComboBox   *comboDataset = new QComboBox();
        connect(comboDataset, &QComboBox::currentTextChanged,
                this, &DockDataset::onDatasetFolderChanged);
        comboDataset->setModel(new DatasetCollectionModel(datasetPath));
        toolbar->addWidget(comboDataset);
        toolbar->addSeparator();

        QAction     *newAct = new QAction(QIcon::fromTheme("document-new"), "New", this);
        connect(newAct, &QAction::triggered, this, &DockDataset::onNewDatasetClick);
        toolbar->addAction(newAct);

        QAction     *editAct = new QAction(QIcon::fromTheme("document-edit"), "Edit", this);
        connect(editAct, &QAction::triggered, this, &DockDataset::onEditDatasetClick);
        toolbar->addAction(editAct);
        toolbar->addSeparator();

        QAction     *deleteAct = new QAction(QIcon::fromTheme("edit-delete"), "Delete", this);
        connect(deleteAct, &QAction::triggered, this, &DockDataset::onDeleteDatasetClick);
        toolbar->addAction(deleteAct);

    box->setContentsMargins(0, 0, 0, 0);
    box->setSpacing(0);
    box->addWidget(toolbar);
    box->addWidget(tableImages);

    this->setWidget(content);
}

void DockDataset::onDatasetFolderChanged(const QString &text)
{
    // Spravime novy model
    QString     currentDatasetPath = datasetPath + "/" + text;

    if (tableImages) {

        // Zavri co mas
        emit onFileClosed();

        DatasetModel    *dsModel = new DatasetModel(currentDatasetPath);
        tableImages->setModel(dsModel);

        tableImages->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
        tableImages->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
        tableImages->setColumnWidth(1, 32);
    }
}

void DockDataset::onImagesDoubleClicked(const QModelIndex &index)
{
    if (tableImages) {
        DatasetModel *model = (DatasetModel*)tableImages->model();
        if (model) {

            // Sourcovy obrazok
            QString filename = model->getFullFilename(index);

            // Ideme vyrabat mapping pre tento subor
            QSharedPointer<Data::ImageMapping> mapping = model->getImageMapping(
                        pfcSource->getPlayfieldCollectionModel(), index
                        );

            // Otvor daco nove
            emit onFileDoubleClicked(filename, mapping);

        }
    }
}

void DockDataset::onNewDatasetClick()
{
    EditDatasetDialog         dialog(this);

    // Setupneme dialog
    dialog.setupDialog(nullptr, pfcSource->getPlayfieldCollectionModel());
    if (dialog.exec() == QDialog::DialogCode::Accepted) {
        auto result = dialog.getResult();
        if (result) {
            // Spravime novy folder a strukturu
            result->toFile(datasetPath);
        }
    }
}

void DockDataset::onEditDatasetClick()
{
    EditDatasetDialog         dialog(this);

    // Setupneme dialog
    dialog.setupDialog(getCurrentDataset(), pfcSource->getPlayfieldCollectionModel());
    if (dialog.exec() == QDialog::DialogCode::Accepted) {
        auto result = dialog.getResult();
        if (result) {
            // Spravime novy folder a strukturu
            result->toFile(datasetPath);

            // Reloadujeme dataset ?
            if (tableImages) {
                DatasetModel *model = (DatasetModel*)tableImages->model();
                if (model) {
                    model->reloadInnerDataset();
                }
            }
        }
    }
}

void DockDataset::onDeleteDatasetClick()
{

}

QSharedPointer<Data::Dataset> DockDataset::getCurrentDataset()
{
    QSharedPointer<Data::Dataset> result = nullptr;

    if (tableImages) {
        DatasetModel *model = (DatasetModel*)tableImages->model();
        if (model) {
            result = model->Dataset();
        }
    }

    return result;
}

DatasetModel *DockDataset::getDatasetModel()
{
    if (tableImages) {
        return (DatasetModel*)tableImages->model();
    }
    return nullptr;
}

