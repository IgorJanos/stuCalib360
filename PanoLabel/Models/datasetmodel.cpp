#include "pch.h"

#include "datasetmodel.h"


//-----------------------------------------------------------------------------
//
//  DatasetModel class
//
//-----------------------------------------------------------------------------

DatasetModel::DatasetModel(
        QString fullPath, QObject *parent
        ) :
    QAbstractListModel(parent),
    path(fullPath)
{
    iconProvider = QSharedPointer<QFileIconProvider>(new QFileIconProvider());

    // Loadneme si interny dataset
    dataset = QSharedPointer<Data::Dataset>(Data::Dataset::loadFromFolder(fullPath));

    // File Watcher - automaticky reload pri zmene
    QFileSystemWatcher *watcher = new QFileSystemWatcher(this);
    watcher->addPath(path + "/images");
    watcher->addPath(path + "/mapping");
    connect(watcher, &QFileSystemWatcher::directoryChanged,
            this, &DatasetModel::onDirectoryChanged);

    reloadImages();
}

void DatasetModel::reloadInnerDataset()
{
    // Loadneme si interny dataset
    dataset = QSharedPointer<Data::Dataset>(Data::Dataset::loadFromFolder(path));
}

void DatasetModel::onDirectoryChanged(const QString &path)
{
    Q_UNUSED(path)

    QDir        di(path);
    if (di.dirName() == "mapping") {
        reloadMapping();
    } else {
        reloadImages();
    }
}

QString DatasetModel::mappingFile(QString image) const
{
    QFileInfo       fi(image);
    QString         mappingFile = fi.baseName() + ".json";
    return path + "/mapping/" + mappingFile;
}

bool DatasetModel::isMappingFile(QString image)
{
    QFileInfo       fmfi(mappingFile(image));
    return fmfi.exists();
}

void DatasetModel::reloadMapping()
{
    QList<int> roles;
    roles.push_back(Qt::DecorationRole);

    // Prejdeme subory
    for (int i=0; i<images.count(); i++) {
        QString image = images[i];

        bool isMapping = isMappingFile(image);
        bool dirty = false;

        if (!mappingExists.contains(image)) {
            dirty = true;
        } else {
            dirty = (isMapping != mappingExists[image]);
        }
        mappingExists.insert(images[i], isMapping);

        if (dirty) {
            QModelIndex     mi = index(i, 1);
            emit dataChanged(mi, mi, roles);
        }
    }
}

void DatasetModel::reloadImages()
{
    beginResetModel();

    images.clear();
    mappingExists.clear();

    QString         imagesPath = path + "/images";
    QDirIterator    it(imagesPath, QDir::Files | QDir::NoDotAndDotDot,
                       QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        QFileInfo   fi(it.next());
        QString     image = fi.fileName();
        images.push_back(image);
        mappingExists.insert(image, isMappingFile(image));
    }

    images.sort();

    endResetModel();
}

int DatasetModel::rowCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return images.count();
}

int DatasetModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent)

    return 2;
}

QVariant DatasetModel::data(
        const QModelIndex &index, int role
        ) const
{
    if  (!index.isValid()) return QVariant();
    if  (role == Qt::DisplayRole) {

        switch (index.column()) {
        case 0:

            // vratime item
            return images[index.row()];

        default:
            return QVariant();
        }
    } else
    if (role == Qt::DecorationRole) {

        switch (index.column()) {
        case 0:
            {                        
                QFileInfo fi(getFullFilename(index));
                return iconProvider->icon(fi);
            }
            break;

        case 1:
            {
                if (hasMapping(index)) {
                    if (hasValidCameraPosition(index)) {
                        return QIcon(":/icons/custom/checkmark.green.svg");
                    }
                    return QIcon(":/icons/custom/checkmark.fill.svg");
                }
            }
        }
    }
    return QVariant();
}

QString DatasetModel::getFullFilename(const QModelIndex &index) const
{
    if (index.isValid()) {
        return path + "/images/" + images[index.row()];
    }
    return "";
}

QSharedPointer<Data::ImageMapping> DatasetModel::getImageMapping(
        PlayfieldCollectionModel *pfcm,
        const QModelIndex &index
        ) const
{
    QSharedPointer<Data::ImageMapping> result;

    if (index.isValid()) {

        if (pfcm) {

            QSharedPointer<Data::PlayfieldModel> pfModel = pfcm->getPlayfieldModel(dataset->modelFilename);
            QString mf = mappingFile(images[index.row()]);

            // Skusime loadnut zo suboru
            result = QSharedPointer<Data::ImageMapping>(
                        Data::ImageMapping::loadFromFile(mf, pfModel.data())
                        );
            if (!result) {
                // Vytvorime novy
                result = QSharedPointer<Data::ImageMapping>(new Data::ImageMapping());

                // obrazok a mapping file
                result->imageFileName = images[index.row()];
                result->mappingFileName = mf;

                // Nakopneme z modelu
                result->loadKeyPoints(pfModel.data(), 0, 0);
            }
        }
    }

    return result;
}

bool DatasetModel::hasMapping(const QModelIndex &index) const
{
    QFileInfo       fi(images[index.row()]);
    QString         mappingFile = fi.baseName() + ".json";
    QFileInfo       fmfi(path + "/mapping/" + mappingFile);

    return fmfi.exists();
}

bool DatasetModel::hasValidCameraPosition(const QModelIndex &index) const
{
    if (index.isValid()) {

        QString mf = mappingFile(images[index.row()]);
        QSharedPointer<Data::ImageMapping> result;

        result = QSharedPointer<Data::ImageMapping>(
            Data::ImageMapping::loadFromFile(mf, NULL)
        );
        if (result) {
            return result->is_position_valid;
        }
    }

    return false;
}





