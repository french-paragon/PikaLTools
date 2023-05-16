#include "bilsequenceactions.h"

#include <steviapp/datablocks/project.h>
#include <steviapp/control/mainwindow.h>

#include "../datablocks/bilacquisitiondata.h"
#include "../gui/trajectoryvieweditor.h"

#include <QList>
#include <QDir>

#include <QFileDialog>

#include <algorithm>

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

    trjve->setTrajectory(bilSequence->localTrajectory());

    QObject::connect(bilSequence, &BilSequenceAcquisitionData::bilSequenceChanged, trjve, [trjve, bilSequence] () {
        trjve->setTrajectory(bilSequence->localTrajectory());
    });

    return true;
}

} // namespace PikaLTools
