#include "comparisonsequenceactions.h"

#include "datablocks/comparisontrajectory.h"

#include "gui/trajectoryvieweditor.h"

#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/project.h>

#include <QFileDialog>


namespace PikaLTools {

int addComparisonSequence(StereoVisionApp::Project* p, const QString &pFile) {

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
    qint64 id = p->createDataBlock(ComparisonTrajectory::staticMetaObject.className());

    if (id < 0) {
        return 1;
    }

    ComparisonTrajectory* seq = p->getDataBlock<ComparisonTrajectory>(id);

    if (seq == nullptr) {
        return 1;
    }

    seq->setDataSource(file);

    return 0;
}

bool viewComparisonTrajectory(ComparisonTrajectory* traj) {

    if (traj == nullptr) {
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

    trjve->setComparisonTrajectory(*traj);

    QObject::connect(traj, &ComparisonTrajectory::dataSourceChanged, trjve, [trjve, traj] () {
        trjve->setComparisonTrajectory(*traj);
    }); //TODO: ensure this connection can be destroyed later

    return true;
}

}
