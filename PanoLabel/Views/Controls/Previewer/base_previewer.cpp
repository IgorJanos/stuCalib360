#include "pch.h"



static QCursor hitTypeToCursor(PreviewerHitType value)
{
    switch (value) {
    case PreviewerHitType::Top:
    case PreviewerHitType::Bottom:
        return Qt::SizeVerCursor;

    case PreviewerHitType::Left:
    case PreviewerHitType::Right:
        return Qt::SizeHorCursor;

    case PreviewerHitType::TopLeft:
    case PreviewerHitType::BottomRight:
        return Qt::SizeFDiagCursor;

    case PreviewerHitType::TopRight:
    case PreviewerHitType::BottomLeft:
        return Qt::SizeBDiagCursor;

    default:
        return Qt::ArrowCursor;
    }

    return Qt::ArrowCursor;
}



//-----------------------------------------------------------------------------
//
//  BasePreviewer class
//
//-----------------------------------------------------------------------------

BasePreviewer::BasePreviewer(QWidget *parent) :
    QGraphicsView(parent)
{
    // trackujeme mys
    setMouseTracking(true);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
}


//-----------------------------------------------------------------------------
//  Keyboard handling
//-----------------------------------------------------------------------------

void BasePreviewer::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Space:
        state.spaceDown = true;
        updateState();
        break;
    default:
        QGraphicsView::keyPressEvent(event);
    }
}

void BasePreviewer::keyReleaseEvent(QKeyEvent *event)
{
    if (state.spaceDown) {
        if (event->key() == Qt::Key_Space) {
            state.spaceDown = false;
            updateState();
        }
    }
    QGraphicsView::keyReleaseEvent(event);
}

//-----------------------------------------------------------------------------
//  Mouse handling
//-----------------------------------------------------------------------------

void BasePreviewer::wheelEvent(QWheelEvent *event)
{    
    QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8;
    qreal sf = 1.0;

    if (!numPixels.isNull()) {
        sf = pow((double)2, -numPixels.y() / 240.0);
    } else if (!numDegrees.isNull()) {
        sf = pow((double)2, numDegrees.y() / 50.0);
    }

    scaleView(sf);
    event->accept();
}

void BasePreviewer::mousePressEvent(QMouseEvent *event)
{
    // Zaciname daco robit
    if (event->button() == Qt::MouseButton::LeftButton) {

        // upravime stav
        state.leftDown = true;
        updateState();

        // Nova drag operacia ?
        if (state.spaceDown) {
            operation = QSharedPointer<PreviewerMouseOperation>(
                            new ViewDragOperation(this, event)
                        );
        }
    }
}

void BasePreviewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (operation) {
        operation->finish(event);
        operation.clear();
    }

    // vypiname left flag
    if (event->button() == Qt::MouseButton::LeftButton) {
        state.leftDown = false;
        updateState();
    }
}

void BasePreviewer::mouseMoveEvent(QMouseEvent *event)
{
    if (operation) {
        operation->mouseMoveEvent(event);
    } else {

        // Hit Test na scenu
        mouseHitTest(event);
    }
}

void BasePreviewer::mouseHitTest(QMouseEvent *event)
{
    Q_UNUSED(event)

    // TODO: override
}


//-----------------------------------------------------------------------------
//  Viewport handling
//-----------------------------------------------------------------------------

void BasePreviewer::scaleView(qreal scaleFactor)
{
    qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0,0,1,1)).width();
    if (factor < 0.2 || factor > 25)
        return;

    // Zoskalujeme na novu uroven
    scale(scaleFactor, scaleFactor);
    state.scale *= scaleFactor;
}


//-----------------------------------------------------------------------------
//  State handling
//-----------------------------------------------------------------------------

void BasePreviewer::updateState()
{
    if (state.spaceDown) {
        if (state.leftDown) {
            setCursor(Qt::ClosedHandCursor);
        } else {
            setCursor(Qt::OpenHandCursor);
        }
    } else {

        if (state.hit) {
            setCursor(hitTypeToCursor(state.hitType));
        } else {
            setCursor(Qt::ArrowCursor);
        }
    }
}



//-----------------------------------------------------------------------------
//
//  ViewDragOperation class
//
//-----------------------------------------------------------------------------

ViewDragOperation::ViewDragOperation(
        BasePreviewer *aview, QMouseEvent *aevent
        ) :
    view(aview),
    pos(aevent->pos())
{
    // spocitame center point
    psStart = view->mapToScene(view->viewport()->rect().center());
}

void ViewDragOperation::mouseMoveEvent(QMouseEvent *event)
{
    view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    QPoint delta = (event->pos() - pos);
    QPointF psNow(
                psStart.x() - delta.x() / view->CurrentScale(),
                psStart.y() - delta.y() / view->CurrentScale()
                );
    view->centerOn(psNow);

    // For zooming to anchor from the view center.
    view->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}







