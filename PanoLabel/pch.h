#ifndef PCH_H
#define PCH_H


#include <QApplication>
#include <QMainWindow>
#include <QGraphicsView>

#include <QtCore>
#include <QtDebug>

#include <opencv2/opencv.hpp>

#if 0
    #include <Eigen/Dense>
#endif

#include "utils.h"

#include "Data/data_interfaces.h"

#include "Models/playfieldcollectionmodel.h"
#include "Models/datasetmodel.h"

#include "Views/Dialogs/exporter.h"

#include "Views/Controls/Previewer/base_previewer.h"
#include "Views/Controls/Previewer/playfield_previewer.h"
#include "Views/Controls/Previewer/mapping_previewer.h"
#include "Views/Controls/PinholePreviewer/pinhole_previewer.h"
#include "Views/Controls/MultiPreviewer/multi_previewer.h"

#include "Views/Docks/dockplayfieldmodels.h"
#include "Views/Docks/dockdataset.h"
#include "Views/Docks/dockview.h"
#include "Views/Dialogs/editmodeldialog.h"
#include "Views/Dialogs/editdatasetdialog.h"


#include "Views/Dialogs/exportdatasetdialog.h"
#include "Views/Dialogs/exportcalibrationdialog.h"


#include "mainwindow.h"
#include "ui_mainwindow.h"

#endif // PCH_H
