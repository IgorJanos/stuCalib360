#include "pch.h"


int main(int argc, char *argv[])
{    
    QApplication a(argc, argv);


    QSurfaceFormat      glFormat;
    glFormat.setVersion(2, 2);
    glFormat.setProfile(QSurfaceFormat::CompatibilityProfile);
    glFormat.setSamples(16);
    QSurfaceFormat::setDefaultFormat(glFormat);

    MainWindow w;

    w.show();
    return a.exec();
}
