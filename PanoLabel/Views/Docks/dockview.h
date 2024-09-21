#ifndef DOCKVIEW_H
#define DOCKVIEW_H

#include <QDockWidget>

namespace Ui {
class DockView;
}

class DockView : public QDockWidget
{
    Q_OBJECT

public:
    explicit DockView(QWidget *parent = nullptr);
    ~DockView();

private:
    Ui::DockView *ui;


    void onSliderK1ValueChanged(int value);
    void onSliderK2ValueChanged(int value);
    void onCheckChanged(bool value);

signals:

    void viewChanged();
};

#endif // DOCKVIEW_H
