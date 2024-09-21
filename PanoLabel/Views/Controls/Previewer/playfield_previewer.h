#ifndef PLAYFIELD_PREVIEWER_H
#define PLAYFIELD_PREVIEWER_H


#include <QWheelEvent>
#include <QGraphicsItem>


//-----------------------------------------------------------------------------
//
//  PlayfieldPreviewer class
//
//-----------------------------------------------------------------------------

class PlayfieldPreviewer : public BasePreviewer
{
protected:

    Data::PlayfieldModel        *model;

public:
    PlayfieldPreviewer(QWidget *parent = nullptr);

    // Previewer methods
    void setPlayfieldModel(Data::PlayfieldModel *amodel);

};




#endif // PLAYFIELD_PREVIEWER_H
