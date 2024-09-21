#ifndef MULTI_PREVIEWER_H
#define MULTI_PREVIEWER_H


//-----------------------------------------------------------------------------
//
//  MultiPreviewer class
//
//-----------------------------------------------------------------------------


enum class ImageViewMode
{
    Panorama,
    Pinhole
};


class MultiPreviewer :
        public QWidget,
        public IMappingPreviewer
{
protected:

    ImageViewMode           viewMode;

    MappingPreviewer        *panoPreviewer;
    PinholePreviewer        *pinholePreviewer;

public:
    MultiPreviewer(QWidget *parent = nullptr);

    void setViewMode(ImageViewMode avalue);
    inline ImageViewMode ViewMode() { return viewMode; }

    // IMappingPreviewer
    void setMapping(QString aimagePath, QSharedPointer<Data::ImageMapping> amapping) override;
    void closeMapping() override;

public slots:

    void onViewChanged();

};


#endif // MULTI_PREVIEWER_H
