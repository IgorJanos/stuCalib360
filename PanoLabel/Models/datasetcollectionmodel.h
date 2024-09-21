#ifndef DATASETCOLLECTIONMODEL_H
#define DATASETCOLLECTIONMODEL_H



#include <QAbstractListModel>
#include <QFileIconProvider>

class DatasetCollectionModel : public QAbstractListModel
{
    Q_OBJECT

protected:

    QString             path;
    QStringList         items;
    QSharedPointer<QFileIconProvider>  iconProvider;

private:

    void onDirectoryChanged(const QString &path);

public:
    explicit DatasetCollectionModel(QString apath, QObject *parent = nullptr);
    ~DatasetCollectionModel();

    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;

    void reload();

signals:

};

#endif // DATASETCOLLECTIONMODEL_H
