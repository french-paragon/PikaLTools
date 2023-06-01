#include "inputdtmactions.h"

#include "datablocks/inputdtm.h"

#include "gui/trajectoryvieweditor.h"

#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/project.h>

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

}
