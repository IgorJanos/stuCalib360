#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsView>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow :
        public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    // Previewer
    MultiPreviewer          *multiPreviewer;
    DockPlayfieldModels     *dockPlayfieldModels;
    DockDataset             *dockDataset;
    DockView                *dockView;

    void createMenus();
    void createDockWindows();
    void createPreviewer();

private slots:

    void closeEvent(QCloseEvent *event) override;

    void onFileClosed();
    void onFileDoubleClicked(QString filePath, QSharedPointer<Data::ImageMapping> mapping);

    void onViewPanoramaClick();
    void onViewPinholeClick();
    void onExportExportClick();
    void onExportExportCalibrationClick();

};
#endif // MAINWINDOW_H
