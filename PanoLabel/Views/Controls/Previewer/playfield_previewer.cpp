#include "pch.h"



//-----------------------------------------------------------------------------
//  PlayfieldNode class
//-----------------------------------------------------------------------------

class PlayfieldNode : public QGraphicsItem
{
protected:

    QPointF             p;
    qreal               radius;
    int                 index;


public:
    PlayfieldNode(QPointF ap, int aindex, QGraphicsItem *parent=nullptr) :
        QGraphicsItem(parent),
        p(ap),
        radius(8),
        index(aindex)
    {

    }

    // QGraphicsItem methods
    QRectF boundingRect() const override
    {
        qreal       d = 2*radius;
        return QRectF(
                    p.x() - radius, p.y() - radius,
                    d, d
                    );
    }

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget = nullptr) override
    {
        Q_UNUSED(option)
        Q_UNUSED(widget)

        // Nakreslime kruzok
        QPen        pen(Qt::black, 1.0);
        QBrush      brush(Qt::yellow);

        QRectF      rc = boundingRect();

        painter->setRenderHint(QPainter::Antialiasing);
        painter->setFont(QFont("fixed-width", 8, QFont::Bold));
        painter->setPen(pen);
        painter->setBrush(brush);
        painter->drawEllipse(rc);

        QTextOption     to;
        to.setAlignment(Qt::AlignCenter);

        QString         text;
        text = QString("%1").arg(index);
        painter->drawText(rc, text, to);
    }

};







//-----------------------------------------------------------------------------
//
//  PlayfieldPreviewer class
//
//-----------------------------------------------------------------------------

PlayfieldPreviewer::PlayfieldPreviewer(QWidget *parent) :
    BasePreviewer(parent),
    model(nullptr)
{
    // mame scenu
    setScene(new QGraphicsScene(this));
    scene()->setBackgroundBrush(QBrush(QColor(64, 64, 64)));
}


//-----------------------------------------------------------------------------
//  Previewer methods
//-----------------------------------------------------------------------------

void PlayfieldPreviewer::setPlayfieldModel(Data::PlayfieldModel *amodel)
{
    // Spravime scenu na kreslenie
    auto s = scene();
    s->clear();

    qreal sf = 3.0;

    if (amodel) {

        int i;

        // Najskor ciary
        for (i=0; i<amodel->edges.size(); i++) {
            auto np = amodel->edges[i];

            QPointF p1, p2;
            p1 = QPointF(amodel->keyPoints[np.first].x * sf, -amodel->keyPoints[np.first].z * sf);
            p2 = QPointF(amodel->keyPoints[np.second].x * sf, -amodel->keyPoints[np.second].z * sf);
            s->addLine(QLineF(p1, p2), QPen(Qt::white, 4.0));
        }

        // Potom body
        for (i=0; i<amodel->keyPoints.size(); i++) {
            auto kp = amodel->keyPoints[i];

            PlayfieldNode *node = new PlayfieldNode(QPointF(kp.x * sf, -kp.z * sf), i);
            s->addItem(node);
        }
    }
}






