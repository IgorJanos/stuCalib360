#include "pch.h"
#include "playfieldcollectionmodel.h"


#include <QFileSystemWatcher>



PlayfieldCollectionModel::PlayfieldCollectionModel(
        QString apath,
        QObject *parent
        ) :
    QAbstractListModel(parent),
    path(apath)
{
    // Assert dir exists!
    QDir        dir(path);
    dir.mkpath(".");

    // File Watcher - automaticky reload pri zmene
    QFileSystemWatcher *watcher = new QFileSystemWatcher(this);
    watcher->addPath(path);
    connect(watcher, &QFileSystemWatcher::directoryChanged,
            this, &PlayfieldCollectionModel::onDirectoryChanged);

    reload();
}

int PlayfieldCollectionModel::indexOfFile(QString filename)
{
    int result = -1;

    for (int i=0; i<items.count(); i++) {
        if (items[i]->filename == filename) {
            return i;
        }
    }

    return result;
}

QSharedPointer<Data::PlayfieldModel> PlayfieldCollectionModel::getPlayfieldModel(QString filename)
{
    for (int i=0; i<items.count(); i++) {
        if (items[i]->filename == filename) {
            return items[i];
        }
    }

    return QSharedPointer<Data::PlayfieldModel>();
}

int PlayfieldCollectionModel::rowCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return items.count();
}

int PlayfieldCollectionModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return 1;
}

QVariant PlayfieldCollectionModel::data(
        const QModelIndex &index, int role
        ) const
{
    if  (!index.isValid()) return QVariant();

    switch (role) {
    case Qt::DisplayRole:
        if (index.column() == 0) {
            // vratime item
            return items[index.row()]->name;
        }
        break;

    case Qt::DecorationRole:
        return QIcon::fromTheme("zoom");

    case (Qt::UserRole + 1):
        return QVariant::fromValue(items[index.row()].data());
    }

    return QVariant();
}

void PlayfieldCollectionModel::reload()
{
    beginResetModel();

    items.clear();

    QDirIterator    it(path, QDir::Files | QDir::NoDotAndDotDot,
                       QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        QFileInfo       fi(it.next());

        Data::PlayfieldModel *pfm = Data::PlayfieldModel::fromFile(path, fi.fileName());
        if (pfm) {
            items.push_back(QSharedPointer<Data::PlayfieldModel>(pfm));
        }
    }

    endResetModel();
}

void PlayfieldCollectionModel::onDirectoryChanged(const QString &path)
{
    Q_UNUSED(path)
    reload();
}

