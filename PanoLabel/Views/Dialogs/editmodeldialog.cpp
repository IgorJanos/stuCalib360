#include "editmodeldialog.h"
#include "ui_editmodeldialog.h"

EditModelDialog::EditModelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::EditModelDialog)
{
    ui->setupUi(this);
    this->resize(1024, 600);
}

EditModelDialog::~EditModelDialog()
{
    delete ui;
}


void EditModelDialog::setupView(Data::PlayfieldModel *amodel, bool anew)
{
    // Spravime novy layout
    QBoxLayout      *box = new QBoxLayout(QBoxLayout::TopToBottom, ui->rightPanel);
    box->setContentsMargins(0, 0, 0, 0);
    previewer = new PlayfieldPreviewer();
    box->addWidget(previewer);

    if (anew) {
        setWindowTitle("Create new model...");

        // Spravime novu instanciu modelu
        model = QSharedPointer<Data::PlayfieldModel>(new Data::PlayfieldModel());

    } else {
        setWindowTitle("Edit model...");

        // Spravime kopiu sourcoveho modelu
        model = QSharedPointer<Data::PlayfieldModel>(amodel->clone());
    }

    // Loadneme data z modelu
    if (model) {
        ui->editName->setText(model->name);
        ui->editFilename->setText(model->filename);
        ui->editLength->setText(QString("%1").arg(model->length));
        ui->editWidth->setText(QString("%1").arg(model->width));
    }

    refreshModel();
}

void EditModelDialog::on_editLength_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    refreshModel();
}

void EditModelDialog::on_editWidth_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    refreshModel();
}


void EditModelDialog::on_editName_textChanged(const QString &arg1)
{
    if (model) {
        model->name = arg1;
    }
}


void EditModelDialog::on_editFilename_textChanged(const QString &arg1)
{
    if (model) {
        model->filename = arg1;
    }
}

void EditModelDialog::refreshModel()
{
    bool        ok = true;
    float       fLength, fWidth;
    fLength = ui->editLength->text().toFloat(&ok);
    if (!ok) return ;
    fWidth = ui->editWidth->text().toFloat(&ok);
    if (!ok) return ;

    Data::PlayfieldModelGenerator       gen;
    Data::PlayfieldModel                *newModel;
    newModel = new Data::PlayfieldModel();
    gen.generate(newModel, fLength, fWidth);
    previewer->setPlayfieldModel(newModel);
    model = QSharedPointer<Data::PlayfieldModel>(newModel);

    // nastavime este meno
    model->name = ui->editName->text();
    model->filename = ui->editFilename->text();
}






