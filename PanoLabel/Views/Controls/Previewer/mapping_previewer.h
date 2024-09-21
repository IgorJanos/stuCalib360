#ifndef MAPPINGPREVIEWER_H
#define MAPPINGPREVIEWER_H



#include <QWheelEvent>
#include <QGraphicsItem>


class IMappingPreviewer
{
public:

    virtual void setMapping(QString aimagePath, QSharedPointer<Data::ImageMapping> amapping) = 0;
    virtual void closeMapping() = 0;

};

template<class IFACE, class CLASS> IFACE *as(CLASS *c) { return dynamic_cast<IFACE*>(c); }


class MappingPreviewer;

//-----------------------------------------------------------------------------
//  MappingNode class
//-----------------------------------------------------------------------------

class MappingNode : public QGraphicsItem
{
protected:

    MappingPreviewer    *previewer;
    QPointF             p;
    qreal               radius;
    int                 index;
    bool                hot;

public:
    MappingNode(QPointF ap, int aindex, MappingPreviewer *apreviewer, QGraphicsItem *parent=nullptr);

    // QGraphicsItem methods
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    void setHot(bool ahot);
    void setP(QPointF ap);

    inline QPointF getP() const { return p; }
    inline bool isHot() const { return hot; }
};



//-----------------------------------------------------------------------------
//  MappingLine class
//-----------------------------------------------------------------------------

class MappingLine : public QGraphicsItem
{
protected:

    MappingPreviewer    *previewer;
    MappingNode         *p1;
    MappingNode         *p2;

public:
    MappingLine(MappingNode *ap1, MappingNode *ap2, MappingPreviewer *apreviewer, QGraphicsItem *parent=nullptr);

    // QGraphicsItem methods
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget = nullptr) override;

    void notifyNodeUpdated(MappingNode *anode);
};


//-----------------------------------------------------------------------------
//  NodeDragOperation
//-----------------------------------------------------------------------------

class NodeDragOperation : public PreviewerMouseOperation
{
private:
    MappingPreviewer        *view;
    QPointF                 pos;
    MappingNode             *node;

public:
    NodeDragOperation(MappingPreviewer *aview, QMouseEvent *aevent, MappingNode *anode);

    void mouseMoveEvent(QMouseEvent *event) override;
    void finish(QMouseEvent *event) override;
};


//-----------------------------------------------------------------------------
//
//  MappingPreviewer class
//
//-----------------------------------------------------------------------------

class MappingPreviewer :
        public BasePreviewer,
        public IMappingPreviewer
{
protected:

    QGraphicsPixmapItem                     *currentPixmap;
    QRectF                                  currentImageRect;

    QSharedPointer<Data::ImageMapping>      mapping;

public:
    MappingPreviewer(QWidget *parent = nullptr);

    // IMappingPreviewer
    void setMapping(QString aimagePath, QSharedPointer<Data::ImageMapping> amapping) override;
    void closeMapping() override;
    void setMapping(QImage &aimg, QSharedPointer<Data::ImageMapping> amapping);

    // Mouse events
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    // Helpers
    void notifyNodeUpdated(MappingNode *anode);

    inline QSharedPointer<Data::ImageMapping> Mapping() { return mapping; }
};


#endif // MAPPINGPREVIEWER_H
