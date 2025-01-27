#include "inputdtmactions.h"

#include "datablocks/inputdtm.h"

#include "gui/trajectoryvieweditor.h"

#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/project.h>

#include <steviapp/gui/sparsealignementeditor.h>
#include <steviapp/gui/imageadapters/imagedatadisplayadapter.h>

#include "../gui/dtmrastervieweditor.h"
#include "../gui/opengldrawables/opengldrawabledtm.h"

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

    StereoVisionApp::Editor* e = mw->openEditor(StereoVisionApp::SparseAlignementEditor::staticMetaObject.className());

    StereoVisionApp::SparseAlignementEditor* sae = qobject_cast<StereoVisionApp::SparseAlignementEditor*>(e);

    if (sae == nullptr) {
        return false;
    }

    QString traj_drawable_name = QString("InputDsm_%1").arg(inputDtm->objectName());

    StereoVisionApp::OpenGlDrawable* drawable = sae->getDrawable(traj_drawable_name);
    OpenGlDrawableDtm* drawableDtm = qobject_cast<OpenGlDrawableDtm*>(drawable);

    if (drawable == nullptr) {

        drawableDtm = new OpenGlDrawableDtm();
        drawableDtm->setSceneScale(sae->sceneScale());

        QObject::connect(sae, &StereoVisionApp::SparseAlignementEditor::sceneScaleChanged,
                         drawableDtm, &OpenGlDrawableDtm::setSceneScale);

        sae->addDrawable(traj_drawable_name, drawableDtm);

    } else if (drawableDtm == nullptr) {
        //a drawable already exist with the name and is not a OpenGlDrawableDtm
        return false; //error, this is not supposed to happen
    }

    drawableDtm->setDtm(inputDtm);

    QObject::connect(inputDtm, &InputDtm::dataSourceChanged, drawableDtm, [drawableDtm, inputDtm] () {
        drawableDtm->setDtm(inputDtm);
    });

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

    StereoVisionApp::Editor* e = mw->openEditor(DTMRasterViewEditor::staticMetaObject.className());

    DTMRasterViewEditor* imageViewer = qobject_cast<DTMRasterViewEditor*>(e);

    if (imageViewer == nullptr) {
        return false;
    }

    imageViewer->setDtm(inputDtm);

    return true;
}

}
