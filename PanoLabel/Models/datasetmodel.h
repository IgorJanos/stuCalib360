#ifndef DATASETMODEL_H
#define DATASETMODEL_H


#include <QAbstractListModel>
#include <QFileIconProvider>

class PlayfieldCollectionModel;


class DatasetModel : public QAbstractListModel
{
    Q_OBJECT

protected:

    QString                             path;
    QStringList                         images;
    QMap<QString, bool>                 mappingExists;
    QSharedPointer<Data::Dataset>       dataset;
    QSharedPointer<QFileIconProvider>   iconProvider;

private:

    void reloadImages();
    void reloadMapping();
    void onDirectoryChanged(const QString &path);

    QString mappingFile(QString image) const;
    bool isMappingFile(QString image);

public:
    DatasetModel(QString fullPath, QObject *parent = nullptr);

    void reloadInnerDataset();

    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;
    QString getFullFilename(const QModelIndex &index) const;

    bool hasMapping(const QModelIndex &index) const;
    bool hasValidCameraPosition(const QModelIndex &index) const;

    QSharedPointer<Data::ImageMapping> getImageMapping(
            PlayfieldCollectionModel *pfcm,
            const QModelIndex &index
            ) const;

    inline QSharedPointer<Data::Dataset> Dataset() { return dataset; }

    inline QString Path() { return path; }
};


#endif // DATASETMODEL_H
