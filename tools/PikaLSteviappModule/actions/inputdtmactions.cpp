#include "inputdtmactions.h"

#include "datablocks/inputdtm.h"

#include "gui/trajectoryvieweditor.h"

#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/project.h>

#include <steviapp/gui/imageviewer.h>
#include <steviapp/gui/imageadapters/imagedatadisplayadapter.h>

#include <QFileDialog>

namespace PikaLTools {

int addInputDtm(StereoVisionApp::Project* p, QString const& pFile) {

    QString file = pFile;

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw != nullptr and file.isEmpty()) {
        if (mw->isVisible()) {
            //in case we have a gui
            file = QFileDialog::getOpenFileName(mw, QObject::tr("Get dtm dataset"));

            if (file.isEmpty()) {
                return 1;
            }

            QFileInfo info(file);

            if (!info.isFile()) {
                return 1;
            }
        }
    }

    //create a bilsequence with the list
    qint64 id = p->createDataBlock(InputDtm::staticMetaObject.className());

    if (id < 0) {
        return 1;
    }

    InputDtm* dtm = p->getDataBlock<InputDtm>(id);

    if (dtm == nullptr) {
        return 1;
    }

    dtm->setDataSource(file);

    return 0;
}

bool viewInputDtm(InputDtm* inputDtm) {

    if (inputDtm == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false; //need main windows to display trajectory
    }

    StereoVisionApp::Editor* e = mw->openEditor(TrajectoryViewEditor::staticMetaObject.className());

    TrajectoryViewEditor* trjve = qobject_cast<TrajectoryViewEditor*>(e);

    if (trjve == nullptr) {
        return false;
    }

    trjve->setDtm(inputDtm);

    QObject::connect(inputDtm, &InputDtm::dataSourceChanged, trjve, [trjve, inputDtm] () {
        trjve->setDtm(inputDtm);
    }); //TODO: ensure this connection can be destroyed later

    return true;
}

bool viewInputDtm2D(InputDtm* inputDtm) {

    QTextStream out(stdout);

    if (inputDtm == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false; //need main windows to display trajectory
    }

    StereoVisionApp::Editor* e = mw->openEditor(StereoVisionApp::ImageViewer::staticMetaObject.className());

    StereoVisionApp::ImageViewer* imageViewer = qobject_cast<StereoVisionApp::ImageViewer*>(e);

    if (imageViewer == nullptr) {
        return false;
    }

    std::optional<StereoVisionApp::Geo::GeoRasterData<float, 2>> dtmData = inputDtm->dtmData();

    if (!dtmData.has_value()) {
        out << "Missing dtm data" << Qt::endl;
        return false;
    }

    float min = dtmData.value().raster.valueUnchecked(0,0);
    float max = dtmData.value().raster.valueUnchecked(0,0);

    auto shape = dtmData.value().raster.shape();

    for (int i = 0; i < shape[0]; i++) {
        for (int j = 0; j < shape[1]; j++) {

            float val = dtmData.value().raster.valueUnchecked(i,j);

            if (val < min) {
                min = val;
            }

            if (val > max) {
                max = val;
            }

        }
    }

    StereoVisionApp::ImageDataDisplayAdapter* dtmDataDisplay = new StereoVisionApp::ImageDataDisplayAdapter(imageViewer);

    std::function<QColor(float)> gradient = [] (float prop) -> QColor {

        constexpr std::array<float, 3> color1 = {15,0,255};
        constexpr std::array<float, 3> color2 = {0,255,0};
        constexpr std::array<float, 3> color3 = {255,255,0};

        std::array<float, 3> c1 = (prop < 0.5) ? color1 : color2;
        std::array<float, 3> c2 = (prop < 0.5) ? color2 : color3;

        float propLocal = 2*prop;

        if (propLocal >= 1) {
            propLocal -= 1;
        }

        if (propLocal < 0) {
            propLocal = 0;
        } if (propLocal > 1) {
            propLocal = 1;
        }

        float p1 = 1 - propLocal;
        float p2 = propLocal;

        std::array<float, 3> colorOut = {p1*c1[0] + p2*c2[0],
                                         p1*c1[1] + p2*c2[1],
                                         p1*c1[2] + p2*c2[2]};

        return QColor(colorOut[0], colorOut[1], colorOut[2]);
    };

    dtmDataDisplay->setGradient(gradient);

    bool logConvert = false;
    dtmDataDisplay->setImageFromArray<float>(dtmData.value().raster, max, min, logConvert);

    imageViewer->setImage(dtmDataDisplay, true);

    return true;
}

}
