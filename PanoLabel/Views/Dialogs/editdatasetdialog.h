#ifndef EDITDATASETDIALOG_H
#define EDITDATASETDIALOG_H

#include <QDialog>

namespace Ui {
class EditDatasetDialog;
}

class EditDatasetDialog : public QDialog
{
    Q_OBJECT

public:
    explicit EditDatasetDialog(QWidget *parent = nullptr);
    ~EditDatasetDialog();

    void setupDialog(
            QSharedPointer<Data::Dataset> acurrent,
            PlayfieldCollectionModel *pfcModel
            );

    QSharedPointer<Data::Dataset> getResult();

private:
    Ui::EditDatasetDialog *ui;


};

#endif // EDITDATASETDIALOG_H
