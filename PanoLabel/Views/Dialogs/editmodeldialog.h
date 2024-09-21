#ifndef EDITMODELDIALOG_H
#define EDITMODELDIALOG_H

#include <QDialog>

namespace Ui {
class EditModelDialog;
}

class EditModelDialog : public QDialog
{
    Q_OBJECT

public:
    explicit EditModelDialog(QWidget *parent = nullptr);
    ~EditModelDialog();

    void setupView(Data::PlayfieldModel *amodel, bool anew);

private slots:

    void on_editLength_textChanged(const QString &arg1);
    void on_editWidth_textChanged(const QString &arg1);
    void on_editName_textChanged(const QString &arg1);
    void on_editFilename_textChanged(const QString &arg1);

private:
    Ui::EditModelDialog *ui;
    PlayfieldPreviewer *previewer;

    QSharedPointer<Data::PlayfieldModel>   model;

    void refreshModel();

public:
    inline Data::PlayfieldModel *Model() { return model.data(); }
};

#endif // EDITMODELDIALOG_H
