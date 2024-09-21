#include "pch.h"

#include "dockplayfieldmodels.h"
#include "ui_dockplayfieldmodels.h"


#include <QToolBar>
#include <QListView>
#include <QStyledItemDelegate>



DockPlayfieldModels::DockPlayfieldModels(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::DockPlayfieldModels)
{
    modelsPath = QDir::homePath() + "/.Playfield/Models";

    ui->setupUi(this);
    setupView();

    this->setMaximumHeight(400);
}

DockPlayfieldModels::~DockPlayfieldModels()
{
    delete ui;
}


void DockPlayfieldModels::setupView()
{
    /*
     *      ToolBar:    New, Edit, Delete
     *      ListView:   Model items
    */

    // Spravime layout
    QWidget         *content = new QWidget();
    QBoxLayout      *box = new QBoxLayout(QBoxLayout::TopToBottom, content);

    // Toolbar
    QToolBar        *toolbar = new QToolBar();

        // Toolbar items
        QAction     *newAct = new QAction(QIcon::fromTheme("document-new"), "New", this);
        connect(newAct, &QAction::triggered, this, &DockPlayfieldModels::onNewModelClick);
        toolbar->addAction(newAct);

        QAction     *editAct = new QAction(QIcon::fromTheme("document-edit"), "Edit", this);
        connect(editAct, &QAction::triggered, this, &DockPlayfieldModels::onEditModelClick);
        toolbar->addAction(editAct);

        toolbar->addSeparator();

        QAction     *deleteAct = new QAction(QIcon::fromTheme("edit-delete"), "Delete", this);
        connect(deleteAct, &QAction::triggered, this, &DockPlayfieldModels::onDeleteModelClick);
        toolbar->addAction(deleteAct);


    // ListView
    listModels = new QListView();
    listModels->setSelectionMode(QAbstractItemView::SingleSelection);

        // Spravime playfield collection model
        PlayfieldCollectionModel *pfcModel = new PlayfieldCollectionModel(modelsPath);
        listModels->setModel(pfcModel);

    box->setContentsMargins(0, 0, 0, 0);
    box->setSpacing(0);
    box->addWidget(toolbar);
    box->addWidget(listModels);

    this->setWidget(content);
}


void DockPlayfieldModels::onNewModelClick()
{
    EditModelDialog         dialog(this);

    dialog.setupView(nullptr, true);
    if (dialog.exec() == QDialog::DialogCode::Accepted) {
        auto m = dialog.Model();
        if (m) {
            m->toFile(modelsPath);
        }
    }
}

void DockPlayfieldModels::onEditModelClick()
{
    auto current = listModels->currentIndex();
    if (current.isValid()) {

        EditModelDialog     dialog(this);
        QVariant            v = current.data(Qt::UserRole + 1);
        Data::PlayfieldModel *m = v.value<Data::PlayfieldModel*>();
        if (m) {
            dialog.setupView(m, false);
            if (dialog.exec() == QDialog::DialogCode::Accepted) {
                auto editM = dialog.Model();
                if (editM) {
                    editM->toFile(modelsPath);
                }
            }

        }
    }
}

void DockPlayfieldModels::onDeleteModelClick()
{
    // TODO: ...

    // reloadModels();
}

PlayfieldCollectionModel *DockPlayfieldModels::getPlayfieldCollectionModel()
{
    PlayfieldCollectionModel *result = nullptr;

    if (listModels) {
        result = (PlayfieldCollectionModel*)listModels->model();
    }

    return result;
}



