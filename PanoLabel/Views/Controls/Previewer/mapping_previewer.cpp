#include "pch.h"
#include <cmath>


//-----------------------------------------------------------------------------
//  MappingNode class
//-----------------------------------------------------------------------------

MappingNode::MappingNode(
        QPointF ap, int aindex,
        MappingPreviewer *apreviewer, QGraphicsItem *parent
        ) :
    QGraphicsItem(parent),
    previewer(apreviewer),
    p(ap),
    radius(16),
    index(aindex),
    hot(false)
{

}

void MappingNode::setP(QPointF ap)
{
    QRectF oldRc = boundingRect();

    p = ap;

    // storneme aj do modelu
    auto m = previewer->Mapping();
    if (m) {
        m->kps[index].x = p.x();
        m->kps[index].y = p.y();
    }

    QRectF newRc = boundingRect();

    update(oldRc);
    update(newRc);
}

void MappingNode::setHot(bool ahot)
{
    hot = ahot;
    update();
}

QRectF MappingNode::boundingRect() const
{
    qreal sf = previewer->CurrentScale();
    qreal r = radius / sf;
    return QRectF(
                p.x() - r, p.y() - r,
                2*r, 2*r
                );
}

void MappingNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
           QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

    qreal sf = previewer->CurrentScale();

    QRectF      rc = boundingRect();

    // Nakreslime kruzok
    if (isHot()) {

        QBrush      penColor = QBrush(QColor(0, 0, 0, 128));
        QPen        pen(penColor, 1.0 / sf);

        painter->setRenderHint(QPainter::Antialiasing);
        painter->setPen(pen);
        painter->drawEllipse(rc);
        painter->drawLine(rc.topLeft(), rc.bottomRight());
        painter->drawLine(rc.topRight(), rc.bottomLeft());

    } else {

        QPen        pen(Qt::black, 1.0 / sf);
        QBrush      brush(Qt::yellow);

        painter->setRenderHint(QPainter::Antialiasing);

        QFont       font("fixed-width", 8, QFont::Bold);
        font.setPointSizeF(16.0 / sf);
        painter->setFont(font);
        painter->setPen(pen);
        painter->setBrush(brush);
        painter->drawEllipse(rc);

        QTextOption     to;
        to.setAlignment(Qt::AlignCenter);

        QString         text;
        text = QString("%1").arg(index);
        painter->drawText(rc, text, to);
    }

}



//-----------------------------------------------------------------------------
//  MappingLine class
//-----------------------------------------------------------------------------

MappingLine::MappingLine(
        MappingNode *ap1, MappingNode *ap2,
        MappingPreviewer *apreviewer, QGraphicsItem *parent
        ) :
    QGraphicsItem(parent),
    previewer(apreviewer),
    p1(ap1),
    p2(ap2)
{

}

// QGraphicsItem methods
QRectF MappingLine::boundingRect() const
{
    qreal       padding = 2.0;
    QPointF     c1, c2;
    c1 = p1->getP();
    c2 = p2->getP();          

    return QRectF(
                QPointF(fmin(c1.x(), c2.x()) - padding, fmin(c1.y(), c2.y()) - padding),
                QPointF(fmax(c1.x(), c2.x()) + padding, fmax(c1.y(), c2.y()) + padding)
                );
}

void MappingLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
           QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

    qreal sf = previewer->CurrentScale();

    // Nakreslime ciarku
    qreal       penWidth = 2.0 / sf;
    QBrush      penColor = Qt::white;

    if (p1->isHot() || p2->isHot()) {

        penWidth = 1.0 / sf;
        penColor = QBrush(QColor(0, 0, 0, 128));

        QPen        pen(penColor, penWidth);
        painter->setPen(pen);

    } else {

        QPen        pen(penColor, penWidth);
        painter->setPen(pen);
    }

    // Nakreslime ciarku
    QPointF     pp1 = p1->getP(); //- rc.topLeft();
    QPointF     pp2 = p2->getP(); //- rc.topLeft();
    painter->drawLine(pp1, pp2);
}

void MappingLine::notifyNodeUpdated(MappingNode *anode)
{
    if (anode == p1 || anode == p2) {
        // TODO: ...
        update();
    }
}


//-----------------------------------------------------------------------------
//  NodeDragOperation class
//-----------------------------------------------------------------------------

NodeDragOperation::NodeDragOperation(
            MappingPreviewer *aview,
            QMouseEvent *aevent, MappingNode *anode
        ) :
    view(aview),
    pos(aevent->pos()),
    node(anode)
{
    //view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view->setCursor(Qt::BlankCursor);

    node->setHot(true);
}

void NodeDragOperation::mouseMoveEvent(QMouseEvent *event)
{
    Q_UNUSED(event)

    node->setP(view->mapToScene(event->pos()));
    view->notifyNodeUpdated(node);
}

void NodeDragOperation::finish(QMouseEvent *event)
{
    Q_UNUSED(event)

    //view->setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);

    node->setHot(false);
    view->notifyNodeUpdated(node);

    view->setCursor(Qt::ArrowCursor);
}

//-----------------------------------------------------------------------------
//
//  MappingPreviewer class
//
//-----------------------------------------------------------------------------

MappingPreviewer::MappingPreviewer(QWidget *parent) :
    BasePreviewer(parent)
{
    // mame scenu
    setScene(new QGraphicsScene(this));
    scene()->setBackgroundBrush(QBrush(QColor(64, 64, 64)));

}


//-----------------------------------------------------------------------------
//  IMappingPreviewer methods
//-----------------------------------------------------------------------------

void MappingPreviewer::setMapping(QImage &aimg, QSharedPointer<Data::ImageMapping> amapping)
{
    // Spravime si internu scenu podla mapping parametrov
    mapping = amapping;

    auto s = scene();
    s->clear();
    currentPixmap = nullptr;

    // Loadneme obrazok
    QPixmap image = QPixmap::fromImage(aimg);
    if (!image.isNull()) {

        // Novy pouzijeme
        auto rc = image.rect();
        int w = rc.width();
        int h = rc.height();
        rc.moveTo(-w/2.0, -h/2.0);

        currentPixmap = s->addPixmap(image);
        currentPixmap->setOffset(-w/2.0, -h/2.0);
        currentPixmap->setTransformationMode(Qt::TransformationMode::SmoothTransformation);
        currentPixmap->setZValue(0);
        currentImageRect = rc;

        s->setSceneRect(rc);
    }


    if (amapping) {

        int i;

        // Spravime nody
        QList<MappingNode*>     nodes;

        // Potom body
        for (i=0; i<amapping->kps.size(); i++) {
            auto kp = amapping->kps[i];
            MappingNode *node = new MappingNode(QPointF(kp.x, kp.y), i, this);
            nodes.push_back(node);
        }

        // Najskor ciary
        for (i=0; i<(int)amapping->edges.size(); i++) {
            auto np = amapping->edges[i];
            MappingLine *line = new MappingLine(nodes[np.first], nodes[np.second], this);
            s->addItem(line);
        }

        for (i=0; i<nodes.count(); i++) {
            s->addItem(nodes[i]);

            notifyNodeUpdated(nodes[i]);
        }

        // savneme rovno
        amapping->toFile();
    }
}

void MappingPreviewer::setMapping(QString aimagePath, QSharedPointer<Data::ImageMapping> amapping)
{
    QImage      img(aimagePath);
    setMapping(img, amapping);
}

void MappingPreviewer::closeMapping()
{
    scene()->clear();

    // Stary odprasime
    currentPixmap = nullptr;
}

void MappingPreviewer::mousePressEvent(QMouseEvent *event)
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
        } else {

            // Skusime spravit hit test na nody
            QPointF     pt = mapToScene(event->pos());

            for (int i=0; i<scene()->items().count(); i++) {
                MappingNode *node = dynamic_cast<MappingNode*>(scene()->items().at(i));
                if (node) {

                    if (node->boundingRect().contains(pt)) {

                        operation = QSharedPointer<PreviewerMouseOperation>(
                                        new NodeDragOperation(this, event, node)
                                    );

                        break;
                    }
                }
            }


        }
    }
}

void MappingPreviewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (operation) {
        operation->finish(event);
        operation.clear();

        if (mapping) {
            mapping->toFile();
        }
    }

    // vypiname left flag
    if (event->button() == Qt::MouseButton::LeftButton) {
        state.leftDown = false;
        updateState();
    }
}

void MappingPreviewer::notifyNodeUpdated(MappingNode *anode)
{
    for (int i=0; i<scene()->items().count(); i++) {
        MappingLine *line = dynamic_cast<MappingLine*>(scene()->items().at(i));
        if (line) {
            line->notifyNodeUpdated(anode);
        }
    }
}
