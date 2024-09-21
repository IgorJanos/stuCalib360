#include "pch.h"


#include "datasetcollectionmodel.h"
#include <QFileSystemWatcher>
#include <QFileIconProvider>


DatasetCollectionModel::DatasetCollectionModel(
        QString apath,
        QObject *parent
    ) :
    QAbstractListModel(parent),
    path(apath)
{
    // Assert dir exists!
    QDir        dir(path);
    dir.mkpath(".");

    iconProvider = QSharedPointer<QFileIconProvider>(new QFileIconProvider());

    // File Watcher - automaticky reload pri zmene
    QFileSystemWatcher *watcher = new QFileSystemWatcher(this);
    watcher->addPath(path);
    connect(watcher, &QFileSystemWatcher::directoryChanged,
            this, &DatasetCollectionModel::onDirectoryChanged);

    reload();
}

DatasetCollectionModel::~DatasetCollectionModel()
{
}


int DatasetCollectionModel::rowCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return items.count();
}

int DatasetCollectionModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return 1;
}

QVariant DatasetCollectionModel::data(
        const QModelIndex &index, int role
        ) const
{
    if  (!index.isValid()) return QVariant();

    switch (role) {
    case Qt::DisplayRole:
        if (index.column() == 0) {
            // vratime item
            return items[index.row()];
        }
        break;

    case Qt::DecorationRole:
        return iconProvider->icon(QFileIconProvider::IconType::Folder);

    }

    return QVariant();
}

void DatasetCollectionModel::reload()
{
    beginResetModel();

    items.clear();

    QDirIterator    it(path, QDir::Dirs | QDir::NoDotAndDotDot,
                       QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        QDir        dir(it.next());
        QString     dn = dir.dirName();
        items.push_back(dn);
    }

    items.sort();

    endResetModel();
}

void DatasetCollectionModel::onDirectoryChanged(const QString &path)
{
    Q_UNUSED(path)
    reload();
}


