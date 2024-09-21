#include "editdatasetdialog.h"
#include "ui_editdatasetdialog.h"




EditDatasetDialog::EditDatasetDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::EditDatasetDialog)
{
    ui->setupUi(this);
}

EditDatasetDialog::~EditDatasetDialog()
{
    delete ui;
}

void EditDatasetDialog::setupDialog(
        QSharedPointer<Data::Dataset> acurrent,
        PlayfieldCollectionModel *pfcModel
        )
{
    // zoznam modelov
    ui->comboBoxPlayfield->setModel(pfcModel);

    // Edit alebo New ?
    if (acurrent) {

        this->setWindowTitle("Edit dataset ...");

        // loadneme si veci
        ui->lineEdit->setText(acurrent->datasetName);

        int index = pfcModel->indexOfFile(acurrent->modelFilename);
        if (index >= 0) {
            ui->comboBoxPlayfield->setCurrentIndex(index);
        }

    } else {

        this->setWindowTitle("Create new dataset ...");
    }
}

QSharedPointer<Data::Dataset> EditDatasetDialog::getResult()
{
    Data::Dataset   *result = nullptr;

    result = new Data::Dataset();
    if (result) {
        result->datasetName = ui->lineEdit->text();
        QVariant pfModel = ui->comboBoxPlayfield->currentData(Qt::UserRole + 1);
        Data::PlayfieldModel *pfm = pfModel.value<Data::PlayfieldModel*>();
        if (pfm) {
            result->modelFilename = pfm->filename;
        }

        // Sanity check
        if (result->datasetName == "" || result->modelFilename == "") {
            delete result;
            result = nullptr;
        }
    }

    return QSharedPointer<Data::Dataset>(result);
}
