#ifndef PLAYFIELDCOLLECTIONMODEL_H
#define PLAYFIELDCOLLECTIONMODEL_H

#include <QAbstractListModel>

class PlayfieldCollectionModel : public QAbstractListModel
{
    Q_OBJECT

protected:

    QString                                     path;
    QList<QSharedPointer<Data::PlayfieldModel>> items;

private:

    void onDirectoryChanged(const QString &path);

public:
    explicit PlayfieldCollectionModel(QString apath, QObject *parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;

    void reload();

    QSharedPointer<Data::PlayfieldModel> getPlayfieldModel(QString filename);
    int indexOfFile(QString filename);

};

#endif // PLAYFIELDCOLLECTIONMODEL_H
