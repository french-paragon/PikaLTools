#include "bilsequenceactions.h"

#include <steviapp/datablocks/project.h>
#include <steviapp/control/application.h>
#include <steviapp/control/mainwindow.h>
#include <steviapp/gui/stepprocessmonitorbox.h>

#include "LibStevi/geometry/core.h"
#include "LibStevi/geometry/rotations.h"
#include "LibStevi/io/image_io.h"

#include "../datablocks/bilacquisitiondata.h"
#include "../datablocks/inputdtm.h"

#include "../gui/trajectoryvieweditor.h"
#include "../gui/bilcubevieweditor.h"
#include "../gui/exportorthophotooptionsdialog.h"

#include "../processing/rectifybilseqtoorthosteppedprocess.h"

#include "io/read_envi_bil.h"

#include "geo/georasterreader.h"

#include <QList>
#include <QDir>
#include <QThread>

#include <QFileDialog>

#include <algorithm>

#include <proj.h>

namespace PikaLTools {

int loadBilSequenceFromFolder(StereoVisionApp::Project* p, QString const& pFolder) {

    QString folder = pFolder;

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw != nullptr and folder.isEmpty()) {
        if (mw->isVisible()) {
            //in case we have a gui
            folder = QFileDialog::getExistingDirectory(mw, QObject::tr("Get bil sequence dir"));

            if (folder.isEmpty()) {
                return 1;
            }
        }
    }

    QList<QString> bilFiles;

    QDir mainFolder(folder);

    //List the folders within the main folder
    QStringList subFolders = mainFolder.entryList();
    std::sort(subFolders.begin(), subFolders.end(), [] (QString str1, QString str2) {
        QString n1 = str1.split("-").last();
        QString n2 = str2.split("-").last();

        bool ok;
        int i1 = n1.toInt(&ok);

        if (!ok) {
            return false;
        }

        int i2 = n2.toInt(&ok);

        if (!ok) {
            return true;
        }

        return i1 < i2;
    });

    for (QString subFolder : subFolders) {

        if (subFolder == "." or subFolder == "..") {
            continue;
        }

        QString fullPath = mainFolder.absoluteFilePath(subFolder);

        QFileInfo dirInfos(fullPath);

        if (!dirInfos.isDir()) {
            continue;
        }

        QDir subDir(fullPath);

        QStringList entries = subDir.entryList({"*.bil"});

        if (entries.size() == 1) {
            bilFiles.push_back(subDir.absoluteFilePath(entries[0]));
        }
    }

    //create a bilsequence with the list
    qint64 id = p->createDataBlock(BilSequenceAcquisitionData::staticMetaObject.className());

    if (id < 0) {
        return 1;
    }

    BilSequenceAcquisitionData* sequence = p->getDataBlock<BilSequenceAcquisitionData>(id);

    if (sequence == nullptr) {
        return 1;
    }

    sequence->setBilSequence(bilFiles);

    return 0;

}



bool showLcfTrajectory(BilSequenceAcquisitionData* bilSequence) {

    if (bilSequence == nullptr) {
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

    trjve->setTrajectory(*bilSequence);

    QObject::connect(bilSequence, &BilSequenceAcquisitionData::bilSequenceChanged, trjve, [trjve, bilSequence] () {
        trjve->setTrajectory(*bilSequence);
    }); //TODO: ensure this connection can be destroyed later

    return true;
}

bool showBilImage(BilSequenceAcquisitionData *bilSequence) {

    if (bilSequence == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false; //need main windows to display trajectory
    }

    StereoVisionApp::Editor* e = mw->openEditor(BilCubeViewEditor::staticMetaObject.className());

    BilCubeViewEditor* bcve = qobject_cast<BilCubeViewEditor*>(e);

    if (bcve == nullptr) {
        return false;
    }

    bcve->setSequence(bilSequence);

    return true;

}



bool exportBilLandmarks(BilSequenceAcquisitionData *bilSequence, QString const& pOutFile) {


    QString outFile = pOutFile;

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw != nullptr and outFile.isEmpty()) {
        if (mw->isVisible()) {
            //in case we have a gui
            outFile = QFileDialog::getSaveFileName(mw, QObject::tr("Export landmarks to"), QString(), QObject::tr("csv Files (*.csv)"));

            if (outFile.isEmpty()) {
                return false;
            }
        }
    }

    QVector<BilSequenceLandmark*> landmarks;

    for (qint64 id : bilSequence->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className())) {
        BilSequenceLandmark* lm = bilSequence->getBilSequenceLandmark(id);

        if (lm == nullptr) {
            continue;
        }

        landmarks.push_back(lm);
    }

    QFile fout(outFile);

    if (!fout.open(QFile::WriteOnly)) {
        return false;
    }

    QTextStream out(&fout);

    out << "Landmark name" << "," << "X coord" << "," << "Y coord" << Qt::endl;

    for (BilSequenceLandmark* lm : landmarks) {
        StereoVisionApp::Landmark* alm = lm->attachedLandmark();

        if (alm == nullptr) {
            continue;
        }

        out << alm->objectName() << "," << lm->x().value() << "," << lm->y().value() << Qt::endl;
    }

    fout.close();
    return true;
}


bool computeOrthophoto(BilSequenceAcquisitionData *bilSequence, InputDtm* pInputDtm, QString const& pOutFile) {

    if (bilSequence == nullptr) {
        return false;
    }

    //get options
    StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

    if (app == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();
    StereoVisionApp::Project* proj = bilSequence->getProject();

    if (proj == nullptr) {
        return false;
    }

    InputDtm* inputDtm = pInputDtm;
    QString outFile = pOutFile;

    int minBilLine = -1;
    int maxBilLine = -1;

    if (mw != nullptr) {
        ExportOrthoPhotoOptionsDialog dialog(mw);

        dialog.setMaxFileId(bilSequence->getBilInfos().size());

        int code = dialog.exec();

        if (code == QDialog::Rejected) {
            return false;
        }

        inputDtm = proj->getDataBlock<InputDtm>(dialog.selectedDtmIdx());
        outFile = dialog.outFile();

        minBilLine = dialog.minFileId();
        maxBilLine = dialog.maxFileId();
    }

    if (inputDtm == nullptr) {
        return false;
    }

    if (outFile.isEmpty()) {
        return false;
    }

    RectifyBilSeqToOrthoSteppedProcess* processor = new RectifyBilSeqToOrthoSteppedProcess();
    processor->configure(bilSequence, inputDtm, outFile);
    processor->setMinAndMaxFileId(minBilLine, maxBilLine);

    QThread* t = new QThread();

    processor->moveToThread(t);
    QObject::connect(processor, &QObject::destroyed, t, &QThread::quit);
    QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

    if (mw != nullptr) {

        StereoVisionApp::StepProcessMonitorBox* box = new StereoVisionApp::StepProcessMonitorBox(mw);
        box->setWindowFlag(Qt::Dialog);
        box->setWindowModality(Qt::WindowModal);
        box->setWindowTitle(QObject::tr("Sparse optimization"));

        box->setProcess(processor);

        QObject::connect(box, &QObject::destroyed, processor, &QObject::deleteLater);

        box->show();

    } else {

        processor->deleteWhenDone(true);
    }

    t->start();
    processor->run();

    return true;
}

} // namespace PikaLTools
