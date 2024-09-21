QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets openglwidgets concurrent

mac {
    PKG_CONFIG = /opt/homebrew/bin/pkg-config
}

QT_CONFIG -= no-pkg-config

CONFIG += c++11
CONFIG += precompile_header
CONFIG += link_pkgconfig

PKGCONFIG += opencv4

INCLUDEPATH += /Users/janos/.lib


# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

PRECOMPILED_HEADER = pch.h

SOURCES += \
    Data/data_dataset.cpp \
    Data/data_playfield.cpp \
    Models/datasetcollectionmodel.cpp \
    Models/datasetmodel.cpp \
    Models/playfieldcollectionmodel.cpp \
    Views/Controls/MultiPreviewer/multi_previewer.cpp \
    Views/Controls/PinholePreviewer/pinhole_previewer.cpp \
    Views/Controls/Previewer/base_previewer.cpp \
    Views/Controls/Previewer/mapping_previewer.cpp \
    Views/Controls/Previewer/playfield_previewer.cpp \
    Views/Dialogs/editdatasetdialog.cpp \
    Views/Dialogs/editmodeldialog.cpp \
    Views/Dialogs/exportcalibrationdialog.cpp \
    Views/Dialogs/exportdatasetdialog.cpp \
    Views/Dialogs/exporter.cpp \
    Views/Docks/dockdataset.cpp \
    Views/Docks/dockplayfieldmodels.cpp \
    Views/Docks/dockview.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Models/datasetcollectionmodel.h \
    Models/datasetmodel.h \
    Models/playfieldcollectionmodel.h \
    Views/Controls/MultiPreviewer/multi_previewer.h \
    Views/Controls/PinholePreviewer/pinhole_previewer.h \
    Views/Controls/Previewer/mapping_previewer.h \
    Views/Dialogs/editdatasetdialog.h \
    Views/Dialogs/exportcalibrationdialog.h \
    Views/Dialogs/exportdatasetdialog.h \
    Views/Dialogs/exporter.h \
    Views/Docks/dockplayfieldmodels.h \
    Views/Docks/dockview.h \
    pch.h \
    Data/data_interfaces.h \
    Views/Controls/Previewer/base_previewer.h \
    Views/Controls/Previewer/playfield_previewer.h \
    Views/Dialogs/editmodeldialog.h \
    Views/Docks/dockdataset.h \
    mainwindow.h \
    utils.h

FORMS += \
    Views/Dialogs/editdatasetdialog.ui \
    Views/Dialogs/editmodeldialog.ui \
    Views/Dialogs/exportcalibrationdialog.ui \
    Views/Dialogs/exportdatasetdialog.ui \
    Views/Docks/dockdataset.ui \
    Views/Docks/dockplayfieldmodels.ui \
    Views/Docks/dockview.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    Shaders/default.frag \
    Shaders/default.vert \
    Shaders/playfield.frag \
    Shaders/playfield.vert

RESOURCES += \
    labelapp.qrc \
    shaders.qrc
