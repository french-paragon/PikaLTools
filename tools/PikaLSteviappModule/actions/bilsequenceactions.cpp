#include "bilsequenceactions.h"

#include <steviapp/datablocks/project.h>
#include <steviapp/control/application.h>
#include <steviapp/control/mainwindow.h>
#include <steviapp/gui/stepprocessmonitorbox.h>
#include <steviapp/gui/imageviewer.h>
#include <steviapp/gui/imageadapters/imagedatadisplayadapter.h>

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

    int nLines = bilSequence->nLinesInSequence();

    if (mw != nullptr) {
        ExportOrthoPhotoOptionsDialog dialog(mw);

        dialog.setLineFileId(nLines);

        int code = dialog.exec();

        if (code == QDialog::Rejected) {
            return false;
        }

        inputDtm = proj->getDataBlock<InputDtm>(dialog.selectedDtmIdx());
        outFile = dialog.outFile();

        minBilLine = dialog.minLineId();
        maxBilLine = dialog.maxLineId();
    }

    if (inputDtm == nullptr) {
        return false;
    }

    if (outFile.isEmpty()) {
        return false;
    }

    RectifyBilSeqToOrthoSteppedProcess* processor = new RectifyBilSeqToOrthoSteppedProcess();
    processor->configure(bilSequence, inputDtm, outFile);
    processor->setMinAndMaxLineId(minBilLine, maxBilLine);

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

bool showCovariance(BilSequenceAcquisitionData *bilSequence) {

    QTextStream out(stdout);

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

    if (mw == nullptr or proj == nullptr) {
        return false;
    }

    QVector<QString> files =  bilSequence->getBilFiles().toVector();

    Eigen::MatrixXd XXt;
    Eigen::VectorXd X;

    Eigen::VectorXd meanEst;

    int64_t nElems = 0;
    int64_t nSamplesAdded = 0;

    StereoVisionApp::Editor* editor = mw->openEditor(StereoVisionApp::ImageViewer::ImageViewerClassName);
    StereoVisionApp::ImageViewer* imageViewer = qobject_cast<StereoVisionApp::ImageViewer*>(editor);

    if (imageViewer == nullptr) {
        return false;
    }

    int rFileIdx = files.size()/2;

    //set a file from mid sequence as first, as it would be more representative of the spetrum in the flight
    QString randomFile = files[rFileIdx];
    files[rFileIdx] = files[0];
    files[0] = randomFile;

    for (int f = 0; f < files.size(); f += 10) {

        QString & file = files[f];

        out << "Reading bil file: " << file << Qt::endl;

        Multidim::Array<float,3> spectral_data = read_envi_bil_to_float(file.toStdString());

        if (spectral_data.empty()) {
            out << "Image is empty, or could not read image data!" << Qt::endl;
            return false;
        }

        out << "Data loaded" << Qt::endl;

        int nLines = spectral_data.shape()[LineAxis];
        int nSamples = spectral_data.shape()[SamplesAxis];
        int nBands = spectral_data.shape()[BandsAxis];

        out << "nLines: " << nLines << Qt::endl;
        out << "nSamples: " << nSamples << Qt::endl;
        out << "nBands: " << nBands << Qt::endl;

        nElems += nLines*nSamples;

        //first element
        if (XXt.size() == 0) {

            XXt.resize(nBands, nBands);
            XXt.setConstant(0);

            X.resize(nBands, 1);
            X.setConstant(0);

            meanEst.resize(nBands, 1);
            meanEst.setConstant(0);

            //compute a mean estimate for numerical stability of the algorithm

            int n = 0;

            for (int i = 0; i < nLines; i += 100) {
                for (int j = 0; j < nSamples; j += 20) {

                    Eigen::VectorXd Xtmp;

                    Xtmp.resize(nBands, 1);

                    for (int id1 = 0; id1 < nBands; id1++) {

                        std::array<int,3> idx1;
                        idx1[LineAxis] = i;
                        idx1[SamplesAxis] = j;
                        idx1[BandsAxis] = id1;

                        float v1 = spectral_data.valueUnchecked(idx1);

                        Xtmp[id1] = v1;
                    }

                    meanEst += Xtmp;
                    n++;

                }
            }

            meanEst /= n;
        }

        for (int i = 0; i < nLines; i += 500) {
            for (int j = 0; j < nSamples; j += 100) {

                Eigen::VectorXd Xtmp;

                Xtmp.resize(nBands, 1);

                for (int id1 = 0; id1 < nBands; id1++) {

                    std::array<int,3> idx1;
                    idx1[LineAxis] = i;
                    idx1[SamplesAxis] = j;
                    idx1[BandsAxis] = id1;

                    float v1 = spectral_data.valueUnchecked(idx1);

                    Xtmp[id1] = v1;
                }

                Xtmp -= meanEst; //numerical stability

                X += Xtmp;
                XXt += Xtmp*Xtmp.transpose();

                nSamplesAdded += 1;

            }
        }

    }

    Eigen::MatrixXd cov = 1/(static_cast<double>(nSamplesAdded)-1)*(XXt - X*X.transpose()/static_cast<double>(nSamplesAdded));
    int nBands = cov.rows();

    Multidim::Array<double, 2> covImg(nBands, nBands);

    double maxCov = 0;

    for (int i = 0; i < nBands; i++) {
        for (int j = 0; j < nBands; j++) {
            covImg.atUnchecked(i,j) = cov(i,j)/(std::sqrt(cov(i,i))*std::sqrt(cov(j,j)));

            if (cov(i,j) > maxCov) {
                maxCov = cov(i,j);
            }
        }
    }

    StereoVisionApp::ImageDataDisplayAdapter* covDataDisplay = new StereoVisionApp::ImageDataDisplayAdapter(imageViewer);

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

    covDataDisplay->setGradient(gradient);

    bool logConvert = false;
    covDataDisplay->setImageFromArray(covImg, 1., -1., logConvert);

    imageViewer->setImage(covDataDisplay, true);

    return true;

}

} // namespace PikaLTools
