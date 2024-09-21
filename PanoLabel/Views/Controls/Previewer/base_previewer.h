#ifndef BASE_PREVIEWER_H
#define BASE_PREVIEWER_H

#include <QWheelEvent>



//-----------------------------------------------------------------------------
//
//  BasePreviewer class
//
//-----------------------------------------------------------------------------

class BasePreviewer;


enum class PreviewerHitType
{
    None,

    // Edges / Corners
    TopLeft, Top, TopRight,
    Left, Right,
    BottomLeft, Bottom, BottomRight,

    Inside
};


struct PreviewerState
{
    bool                spaceDown;
    bool                leftDown;
    bool                hit;

    qreal               scale;
    PreviewerHitType    hitType;

    PreviewerState() :
        spaceDown(false),
        leftDown(false),
        hit(false),
        scale(1.0),
        hitType(PreviewerHitType::None)
    {
    }
};


//-----------------------------------------------------------------------------
//
//  Mouse Operations
//
//-----------------------------------------------------------------------------

class PreviewerMouseOperation
{
public:
    virtual ~PreviewerMouseOperation() { }

    virtual void mouseMoveEvent(QMouseEvent *event) { Q_UNUSED(event) }
    virtual void finish(QMouseEvent *event) { Q_UNUSED(event) }
};


class ViewDragOperation : public PreviewerMouseOperation
{
private:
    BasePreviewer         *view;
    QPoint                pos;
    QPointF               psStart;

public:
    ViewDragOperation(BasePreviewer *aview, QMouseEvent *aevent);

    virtual void mouseMoveEvent(QMouseEvent *event);
};




//-----------------------------------------------------------------------------
//
//  BasePreviewer class
//
//-----------------------------------------------------------------------------


class BasePreviewer : public QGraphicsView
{
protected:

    QSharedPointer<PreviewerMouseOperation>     operation;
    PreviewerState                              state;

    virtual void updateState();

    virtual void mouseHitTest(QMouseEvent *event);

public:
    BasePreviewer(QWidget *parent=nullptr);

    // overriden spravanie
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    // Viewport handling
    void scaleView(qreal scaleFactor);

public:
    inline qreal CurrentScale() { return state.scale; }
};





#endif // BASE_PREVIEWER_H
