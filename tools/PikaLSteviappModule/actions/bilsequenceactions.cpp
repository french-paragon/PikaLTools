#include "bilsequenceactions.h"

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/image.h>
#include <steviapp/datablocks/trajectory.h>
#include <steviapp/control/application.h>
#include <steviapp/control/mainwindow.h>
#include <steviapp/gui/stepprocessmonitorbox.h>
#include <steviapp/gui/imageviewer.h>
#include <steviapp/gui/imageadapters/imagedatadisplayadapter.h>

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/trajectory.h>

#include <steviapp/vision/indexed_timed_sequence.h>

#include <steviapp/sparsesolver/sbagraphreductor.h>
#include <steviapp/sparsesolver/modularsbasolver.h>
#include <steviapp/sparsesolver/sbamodules/trajectorybasesbamodule.h>
#include <steviapp/sparsesolver/sbamodules/landmarkssbamodule.h>
#include <steviapp/sparsesolver/sbamodules/imagealignementsbamodule.h>
#include <steviapp/sparsesolver/sbamodules/correspondencessetsbamodule.h>
#include <steviapp/sparsesolver/sbamodules/localcoordinatesystemsbamodule.h>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/io/image_io.h>
#include <StereoVision/interpolation/interpolation.h>

#include "../datablocks/bilacquisitiondata.h"
#include "../datablocks/inputdtm.h"

#include "../gui/trajectoryvieweditor.h"
#include "../gui/bilcubevieweditor.h"
#include "../gui/exportorthophotooptionsdialog.h"
#include "../gui/pushbroomoptimizationconfigdialog.h"
#include "../gui/simulatepushbroomtiepointsoptiondialog.h"

#include "../processing/rectifybilseqtoorthosteppedprocess.h"

#include "../solving/cerespushbroomsolver.h"
#include "../solving/bilsequencesbamodule.h"
#include "../solving/dtmtiepointsmodule.h"

#include "io/read_envi_bil.h"

#include "io/georasterreader.h"

#include "processing/texturegeneration.h"
#include "processing/pushbroomprojections.h"

#include <QList>
#include <QDir>
#include <QThread>

#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileInfo>
#include <QDesktopServices>

#include <algorithm>
#include <random>

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

    double targetGsd = 2.0;

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

        targetGsd = dialog.getTargetGSD();
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
    processor->setTargetGSD(targetGsd);

    QThread* t = new QThread();

    processor->moveToThread(t);
    QObject::connect(processor, &QObject::destroyed, t, &QThread::quit);
    QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);

    if (mw != nullptr) {

        StereoVisionApp::StepProcessMonitorBox* box = new StereoVisionApp::StepProcessMonitorBox(mw);
        box->setWindowFlag(Qt::Dialog);
        box->setWindowModality(Qt::WindowModal);
        box->setWindowTitle(QObject::tr("Exporting orthophoto"));

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

bool setBilSequenceTrajectory(BilSequenceAcquisitionData *bilSequence, qint64 trajId) {

    if (bilSequence == nullptr) {
        return false;
    }

    StereoVisionApp::Project* project = bilSequence->getProject();

    if (project == nullptr) {
        return false;
    }

    StereoVisionApp::Trajectory* traj = project->getDataBlock<StereoVisionApp::Trajectory>(trajId);

    if (traj == nullptr) {

        StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

        if (mw == nullptr) {
            return false;
        }

        QVector<qint64> trajIdxs = project->getIdsByClass(StereoVisionApp::Trajectory::staticMetaObject.className());
        QStringList names;

        for (qint64 id : trajIdxs) {
            names << project->getById(id)->objectName();
        }

        bool ok = true;
        QString selected = QInputDialog::getItem(mw, QObject::tr("Get trajectory"), QObject::tr("trajectory"), names, 0, false, &ok);

        if (!ok) {
            return false;
        }

        int idx = names.indexOf(selected);

        if (idx < 0 or idx >= trajIdxs.size()) {
            return false;
        }

        qint64 trajId = trajIdxs[idx];

        traj = project->getDataBlock<StereoVisionApp::Trajectory>(trajId);

    }

    if (traj == nullptr) {
        return false;
    }

    bilSequence->assignTrajectory(traj->internalId());

    return true;

}


bool initBilSequencesTiePoints() {

    QTextStream out(stdout);

    out << "init Bil Sequences Tie Points" << Qt::endl;

    //get options
    StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

    if (app == nullptr) {
        return false;
    }

    StereoVisionApp::Project* project = app->getCurrentProject();

    if (project == nullptr) {
        return false;
    }

    StereoVision::Geometry::AffineTransform<float> ecef2local = project->ecef2local();
    StereoVision::Geometry::AffineTransform<float> local2ecef(ecef2local.R.transpose(),
                                                              -ecef2local.R.transpose()*ecef2local.t);

    QVector<qint64> bilSeqIdxs = project->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    struct LmPointInfo{qint64 seqId; qint64 seqlmid;};

    QMap<qint64,QVector<LmPointInfo>> pointsInfos;

    for (qint64 bilSeqId : bilSeqIdxs) {

        BilSequenceAcquisitionData* bilSeq = project->getDataBlock<BilSequenceAcquisitionData>(bilSeqId);

        if (bilSeq == nullptr) {
            continue;
        }

        QVector<qint64> bilLandmarksIdxs = bilSeq->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

        for (qint64 bilLmId : bilLandmarksIdxs) {

            BilSequenceLandmark* lm = bilSeq->getBilSequenceLandmark(bilLmId);

            if (lm == nullptr) {
                continue;
            }

            qint64 lmId = lm->attachedLandmarkid();

            pointsInfos[lmId].push_back({bilSeqId, bilLmId});

        }
    }

    QList<qint64> selectedLmIdxs = pointsInfos.keys();

    out << "Candidate points: ";

    for (qint64 id : selectedLmIdxs) {
        out << id << " ";
    }

    out << Qt::endl;

    out << "Processed points: ";

    for (qint64 lmId : selectedLmIdxs) {

        StereoVisionApp::Landmark* lm = project->getDataBlock<StereoVisionApp::Landmark>(lmId);

        if (lm == nullptr) {
            continue;
        }

        if (lm->hasOptimizedParameters() or pointsInfos[lmId].size() < 2) { // initial solution already set
            continue;
        }

        int nMeasures = pointsInfos[lmId].size();

        //We build a system of equations d x (x - t) = 0, where x is the coordinate of the landmark, d is the ray direction and t is the ray origin
        // (or, alternatively, d x x = d x t
        Eigen::Matrix<double,Eigen::Dynamic,3> A;
        Eigen::Matrix<double,Eigen::Dynamic,1> b;

        A.resize(3*nMeasures,3);
        b.resize(3*nMeasures,1);

        #ifndef NDEBUG //be able to check positions and directions in debug variables
        Eigen::Matrix<double,Eigen::Dynamic,1> d;
        d.resize(3*nMeasures,1);
        Eigen::Matrix<double,Eigen::Dynamic,1> p;
        p.resize(3*nMeasures,1);
        #endif

        bool ok = true;

        for (int i = 0; i < pointsInfos[lmId].size(); i++) {

            LmPointInfo& lmInfos = pointsInfos[lmId][i];

            BilSequenceAcquisitionData* bilSeq = project->getDataBlock<BilSequenceAcquisitionData>(lmInfos.seqId);

            if (bilSeq == nullptr) {
                ok = false;
                break;
            }

            BilSequenceLandmark* lm = bilSeq->getBilSequenceLandmark(lmInfos.seqlmid);

            if (lm == nullptr) {
                ok = false;
                break;
            }

            std::optional<Eigen::Matrix<double,3,2>> rayInfos = lm->getRayInfos();

            if (!rayInfos.has_value()) {
                ok = false;
                break;
            }

            #ifndef NDEBUG
            d.block<3,1>(i*3,0) = rayInfos.value().col(1);
            p.block<3,1>(i*3,0) = rayInfos.value().col(0);
            #endif

            A.block<3,3>(i*3,0) = StereoVision::Geometry::skew<double>(rayInfos.value().col(1));

            b.block<3,1>(i*3,0) = rayInfos.value().col(1).cross(rayInfos.value().col(0));

        }

        if (!ok) {
            continue;
        }

        Eigen::Vector3f lsInteresctPos = A.matrix().colPivHouseholderQr().solve(b).cast<float>();

        constexpr bool optimized = true;
        lm->setPositionFromEcef(local2ecef*lsInteresctPos, optimized);

        out << lmId << " ";

    }

    out << Qt::endl;

    return true;
}

bool openBilSequenceFolder(BilSequenceAcquisitionData *bilSequence) {

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false;
    }

    if (bilSequence == nullptr) {
        QMessageBox::warning(mw,
                             QObject::tr("Could not open bil sequence directory"),
                             QObject::tr("Null bil sequence"));
        return false;
    }

    QStringList files = bilSequence->getBilFiles();

    QFileInfo fileInfo(files[0]);

    QDir dir = fileInfo.absoluteDir();

    if (!dir.exists()) {
        QMessageBox::warning(mw,
                             QObject::tr("Could not open bil sequence directory"),
                             QObject::tr("Directory %1 does not exist").arg(dir.absolutePath()));
        return false;
    }

    QDesktopServices::openUrl(dir.absolutePath());

    return true;

}

bool simulatePseudoPushBroomData() {

    using TimeSeq = CeresPushBroomSolver::IndexedVec3TimeSeq;

    QTextStream out(stdout);

    //get options
    StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

    if (app == nullptr) {
        return false;
    }

    StereoVisionApp::Project* project = app->getCurrentProject();

    if (project == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    SimulatePushBroomTiePointsOptionDialog configDialog(project, mw);
    configDialog.setModal(true);
    configDialog.setWindowTitle(QObject::tr("Configure pseudo push broom data generation"));

    int code = configDialog.exec();

    if (code == QDialog::Rejected) {
        return false;
    }

    DataColumnsSelectionWidget::ColumnSelectionInfos orientationInfosColsSelection = configDialog.getsOrientationColsSelectionInfos();
    DataColumnsSelectionWidget::ColumnSelectionInfos positionInfosColsSelection = configDialog.getsPositionColsSelectionInfos();

    CeresPushBroomSolver::GPSMeasurementInfos positionInfos;
    auto errorMessage = positionInfos.configureFromColumnsSelection(positionInfosColsSelection);

    if (errorMessage.has_value()) {
        QMessageBox::warning(mw, QObject::tr("Unable to simulate GCP data"), errorMessage.value());
        return false;
    }

    CeresPushBroomSolver::InitialOrientationInfos orientationInfos;
    errorMessage = orientationInfos.configureFromColumnsSelection(orientationInfosColsSelection);

    if (errorMessage.has_value()) {
        QMessageBox::warning(mw, QObject::tr("Unable to simulate GCP data"), errorMessage.value());
        return false;
    }

    TimeSeq positionSequence = positionInfos.compileSequence();
    TimeSeq orientationSequence = orientationInfos.compileSequence();

    if (positionSequence.nPoints() <= 0 or orientationSequence.nPoints() <= 0) {
        QMessageBox::warning(mw, QObject::tr("Unable to simulate GCP data"), QObject::tr("Empty trajectory or orientation time sequence"));
        return false;
    }

    double startTime = std::max(positionSequence.sequenceStartTime(), orientationSequence.sequenceStartTime());
    double endTime = std::min(positionSequence.sequenceEndTime(), orientationSequence.sequenceEndTime());

    if (endTime <= startTime) {
        QMessageBox::warning(mw, QObject::tr("Unable to simulate GCP data"), QObject::tr("Non-aligned trajectory and orientation time sequences"));
        return false;
    }

    int nBands = configDialog.nBands();
    int nPoints = configDialog.nPoints();
    int nPixels = configDialog.nPixels();
    int fLen = configDialog.focalLength();

    QVector<qint64> oldLandmarksIdxs = project->getIdsByClass(StereoVisionApp::Landmark::staticMetaObject.className());
    QVector<qint64> oldBilSeqsIdxs = project->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    bool hasData = !oldLandmarksIdxs.isEmpty() or !oldBilSeqsIdxs.isEmpty();

    if (hasData) {
        int code = QMessageBox::question(mw,
                                         QObject::tr("Are you sure you want to regenerate a simulated problem?"),
                                         QObject::tr("All landmarks and bill sequence datablocks in the project will be replaced!"));

        if (code != QMessageBox::Yes and code != QMessageBox::Ok) {
            return false;
        }
    }

    for (qint64 landmarkId : oldLandmarksIdxs) {
        project->clearById(landmarkId);
    }

    for (qint64 bilId : oldBilSeqsIdxs) {
        project->clearById(bilId);
    }

    qint64 bilSeqId = project->createDataBlock(BilSequenceAcquisitionData::staticMetaObject.className());
    BilSequenceAcquisitionData* bilSeq = project->getDataBlock<BilSequenceAcquisitionData>(bilSeqId);

    if (bilSeq == nullptr) {
        QMessageBox::warning(mw, QObject::tr("Unable to simulate GCP data"), QObject::tr("Failure to create bilSeq datablock"));
        return false;
    }

    BilSequenceAcquisitionData::SequenceInfos seqInfos;
    seqInfos.fLen = fLen;
    seqInfos.initial_time = startTime;
    seqInfos.lineWidth = nPixels;
    seqInfos.nLines = nBands;
    seqInfos.time_per_line = (endTime - startTime)/nBands;

    bilSeq->setSequenceInfos(seqInfos);

    //occupancy cube
    std::array<float, 3> minCorner = positionSequence[0].val;
    std::array<float, 3> maxCorner = positionSequence[0].val;

    for (int i = 1; i < positionSequence.nPoints(); i++) {

        for (int j = 0; j < minCorner.size(); j++) {
            float val = positionSequence[i].val[j];

            if (val < minCorner[j]) {
                minCorner[j] = val;
            }

            if (val > maxCorner[j]) {
                maxCorner[j] = val;
            }
        }
    }

    int terrainW = 128;
    int terrainH = 128;

    float baseLevel = minCorner[2]; //min z value of the flight

    float meanSection = (maxCorner[1] + maxCorner[0] - minCorner[1] - minCorner[0])/2;

    std::uniform_real_distribution<float> xDistribution(minCorner[0], maxCorner[0]);
    std::uniform_real_distribution<float> yDistribution(minCorner[1], maxCorner[1]);

    float zAmpl = meanSection*0.1;

    int rSeed = configDialog.randomSeed();

    if (rSeed < 0) {
        std::random_device rd;
        rSeed = rd();
    }

    rSeed += nPixels + nPoints + nBands;

    int nodeSquareSide = 10;
    float spatialCorrelation = 0.8;

    Multidim::Array<float, 2> terrain = generateTerrainTexture(terrainW,
                                                               terrainH,
                                                               baseLevel-zAmpl,
                                                               baseLevel+zAmpl,
                                                               nodeSquareSide,
                                                               spatialCorrelation,
                                                               rSeed);
    Multidim::Array<std::vector<int>, 2> pointsIndex(terrainH, terrainW);

    for (int i = 0; i < terrainH; i++) {
        for (int j = 0; j < terrainW; j++) {
            pointsIndex.atUnchecked(i,j) = std::vector<int>();
        }
    }

    std::vector<std::array<float, 3>> randomPoints(nPoints);
    std::vector<qint64> lm_idxs(nPoints);
    std::vector<int> viewCount(nPoints, 0);

    std::default_random_engine re(rSeed);

    for (int pt = 0; pt < nPoints; pt++) {

        float xVal = xDistribution(re);
        float yVal = yDistribution(re);

        float iTerrainFracIdx = (xVal - minCorner[0])/(maxCorner[0] - minCorner[0]) * terrainH;
        float jTerrainFracIdx = (yVal - minCorner[1])/(maxCorner[1] - minCorner[1]) * terrainW;

        float zVal = StereoVision::Interpolation::interpolateValue<2, float, StereoVision::Interpolation::pyramidFunction<float, 2>, 1>
                (terrain, {iTerrainFracIdx, jTerrainFracIdx});

        randomPoints[pt] = {xVal, yVal, zVal};

        int iBin = std::floor(iTerrainFracIdx);
        int jBin = std::floor(jTerrainFracIdx);

        if (iBin >= terrainH) {
            iBin = terrainH-1;
        }

        if (jBin >= terrainW) {
            jBin = terrainW-1;
        }

        std::vector<int> & ptList = pointsIndex.atUnchecked(iBin, jBin);

        if (std::find(ptList.begin(), ptList.end(), pt) != ptList.end()) {
            out << "Somethign strange is hapenning" << Qt::endl;
        }

        pointsIndex.atUnchecked(iBin, jBin).push_back(pt);

        Eigen::Vector3f pointCoord(randomPoints[pt][0], randomPoints[pt][1], randomPoints[pt][2]);

        qint64 lm_id = project->createDataBlock(StereoVisionApp::Landmark::staticMetaObject.className());
        StereoVisionApp::Landmark* lm = project->getDataBlock<StereoVisionApp::Landmark>(lm_id);

        if (lm != nullptr) {

            lm->setObjectName(QObject::tr("SimulatedLM%1").arg(pt));

            lm->setXCoord(pointCoord[0]);
            lm->setYCoord(pointCoord[1]);
            lm->setZCoord(pointCoord[2]);

            lm_idxs[pt] = lm_id;
        } else {
            lm_idxs[pt] = -1;
        }

    }

    Eigen::Matrix3f cam2frame = Eigen::Matrix3f::Zero();

    cam2frame(0,0) = 1;
    cam2frame(1,1) = -1;
    cam2frame(2,2) = -1;


    for (int l = 0; l < seqInfos.nLines; l++) {
        double time = seqInfos.initial_time + l*seqInfos.time_per_line;

        auto interpolabePos = positionSequence.getValueAtTime(time);
        auto interpolabeOrient = orientationSequence.getValueAtTime(time);

        Eigen::Vector3f interpolatedPos;
        interpolatedPos.x() = interpolabePos.weigthLower*interpolabePos.valLower[0] + interpolabePos.weigthUpper*interpolabePos.valUpper[0];
        interpolatedPos.y() = interpolabePos.weigthLower*interpolabePos.valLower[1] + interpolabePos.weigthUpper*interpolabePos.valUpper[1];
        interpolatedPos.z() = interpolabePos.weigthLower*interpolabePos.valLower[2] + interpolabePos.weigthUpper*interpolabePos.valUpper[2];

        Eigen::Vector3f interpolatedOrient;
        interpolatedOrient.x() = interpolabeOrient.weigthLower*interpolabeOrient.valLower[0] + interpolabeOrient.weigthUpper*interpolabeOrient.valUpper[0];
        interpolatedOrient.y() = interpolabeOrient.weigthLower*interpolabeOrient.valLower[1] + interpolabeOrient.weigthUpper*interpolabeOrient.valUpper[1];
        interpolatedOrient.z() = interpolabeOrient.weigthLower*interpolabeOrient.valLower[2] + interpolabeOrient.weigthUpper*interpolabeOrient.valUpper[2];

        Eigen::Matrix3f R = StereoVision::Geometry::rodriguezFormula(interpolatedOrient);

        StereoVision::Geometry::AffineTransform<float> cam2world(R*cam2frame, interpolatedPos);
        StereoVision::Geometry::AffineTransform<float> world2cam(cam2world.R.transpose(), -cam2world.R.transpose()*cam2world.t);

        float scaleTerrainX2I = 1./(maxCorner[0] - minCorner[0]) * terrainH;
        float scaleTerrainY2J = 1./(maxCorner[1] - minCorner[1]) * terrainW;

        Eigen::Matrix3f w2t_scale = Eigen::Matrix3f::Zero();

        w2t_scale(0,0) = scaleTerrainX2I;
        w2t_scale(1,1) = scaleTerrainY2J;
        w2t_scale(2,2) = 1;

        Eigen::Vector3f w2t_delta(- minCorner[0]*scaleTerrainX2I, - minCorner[1]*scaleTerrainY2J, 0);
        StereoVision::Geometry::AffineTransform<float> world2terrain(w2t_scale, w2t_delta);

        StereoVision::Geometry::AffineTransform<float> cam2terrain = world2terrain*cam2world;


        float maxHeight = terrain.valueUnchecked(0,0);

        for (int i = 0; i < terrainH; i++) {
            for (int j = 0; j < terrainW; j++) {
                float val = terrain.valueUnchecked(i,j);

                if (val > maxHeight) {
                    maxHeight = val;
                }
            }
        }

        std::vector<std::array<int, 2>> lineCoords = terrainPixelsSeenByScannerLine(terrain,
                                                                                    cam2terrain,
                                                                                    seqInfos.fLen,
                                                                                    static_cast<float>(nPixels/2),
                                                                                    seqInfos.lineWidth,
                                                                                    maxHeight);

        std::set<std::array<int, 2>> treated;

        for (std::array<int, 2> const& coord : lineCoords) {

            if (treated.count(coord) > 0) {
                out << "Duplicated coord detected!" << Qt::endl;
            }

            treated.insert(coord);

            if (coord[0] < 0 or coord[1] < 0 or coord[0] >= pointsIndex.shape()[0] or coord[1] >= pointsIndex.shape()[1]) {
                continue;
            }

            std::vector<int> & ptList = pointsIndex.atUnchecked(coord);

            for (int ptId : ptList) {

                qint64 lm_id = lm_idxs[ptId];

                if (lm_id < 0) {
                    continue;
                }

                Eigen::Vector3f pointCoord(randomPoints[ptId][0],randomPoints[ptId][1], randomPoints[ptId][2]);

                Eigen::Vector3f pointFrameCoord = world2cam*pointCoord;

                if (pointFrameCoord.z() < 0) {
                    continue;
                }

                Eigen::Vector2f projected = pointFrameCoord.block<2,1>(0,0)/pointFrameCoord[2];

                projected *= fLen;
                projected[0] += static_cast<float>(nPixels/2);

                if (std::abs(projected[1]) < 0.5) {
                    //points detected
                    bilSeq->addBilSequenceLandmark(QPointF(projected[0], projected[1] + l), lm_id);
                }
            }

        }

    }

    return true;

}

bool refineTrajectoryUsingDn() {

    //get options
    StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

    if (app == nullptr) {
        return false;
    }

    StereoVisionApp::Project* project = app->getCurrentProject();

    if (project == nullptr) {
        return false;
    }

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false;
    }

    //TODO: add config dialog for options if needed

    int nSteps = 400;

    if (nSteps <= 0) {
        return false;
    }

    bool computeUncertainty = false;
    bool sparse = true;
    bool verbose = true;

    StereoVisionApp::ModularSBASolver* solver =
            new StereoVisionApp::ModularSBASolver(project, computeUncertainty, sparse, verbose);

    solver->setOptimizationSteps(nSteps);

    QFileInfo projSourceInfos(project->source());

    QDir projDir = projSourceInfos.dir();
    solver->enableLogging(projDir.filePath("logging"));

    double gpsAccuracy = 0.02;
    double angularAccuracy = 0.1;
    double gyroAccuracy = 0.1;
    double accAccuracy = 0.5;
    double tiePointAccuracy = 0.5;

    double integrationtime = 0.5; //half a second

    StereoVisionApp::TrajectoryBaseSBAModule* trajectoryModule =
            new StereoVisionApp::TrajectoryBaseSBAModule(integrationtime);

    trajectoryModule->enableOrientationPrior(true);

    trajectoryModule->setDefaultGpsAccuracy(gpsAccuracy);
    trajectoryModule->setDefaultOrientAccuracy(angularAccuracy);
    trajectoryModule->setDefaultGyroAccuracy(gyroAccuracy);
    trajectoryModule->setDefaultAccAccuracy(accAccuracy);

    solver->addModule(trajectoryModule);

    StereoVisionApp::LandmarksSBAModule* landmarkModule =
                 new StereoVisionApp::LandmarksSBAModule();

     solver->addModule(landmarkModule);

     StereoVisionApp::ImageAlignementSBAModule* imageModule =
                  new StereoVisionApp::ImageAlignementSBAModule();

      solver->addModule(imageModule);

      StereoVisionApp::LocalCoordinateSystemSBAModule* lcsModule =
                   new StereoVisionApp::LocalCoordinateSystemSBAModule();

       solver->addModule(lcsModule);

     StereoVisionApp::CorrespondencesSetSBAModule * correspModule =
                  new StereoVisionApp::CorrespondencesSetSBAModule();

      solver->addModule(correspModule);

    BilSequenceSBAModule* bilSequenceModule =
            new BilSequenceSBAModule();

    solver->addModule(bilSequenceModule);

    DtmTiePointsModule* dtmTiePointsModule =
            new DtmTiePointsModule();

    solver->addModule(dtmTiePointsModule);

    //do the solving

    //QThread* t = new QThread();

    /*solver->moveToThread(t);
    QObject::connect(solver, &QObject::destroyed, t, &QThread::quit);
    QObject::connect(t, &QThread::finished, t, &QObject::deleteLater);*/

    if (mw != nullptr) {

        StereoVisionApp::StepProcessMonitorBox* box = new StereoVisionApp::StepProcessMonitorBox(mw);
        box->setWindowFlag(Qt::Dialog);
        box->setWindowModality(Qt::WindowModal);
        box->setWindowTitle(QObject::tr("Bil sequence optimization"));

        box->setProcess(solver);

        QObject::connect(box, &QObject::destroyed, solver, &QObject::deleteLater);

        box->show();

    } else {

        solver->deleteWhenDone(true);
    }

    //t->start();

    solver->run();

    return true;

}


bool estimateTimeDeltaRough(BilSequenceAcquisitionData *bilSequence) {

    QTextStream out(stdout);

    out << "Estimate bil sequence time delta:" << Qt::endl;

    if (bilSequence == nullptr) {
        out << "\tNull sequence provided, aborting!" << Qt::endl;
        return false;
    }

    StereoVisionApp::Project* project = bilSequence->getProject();

    if (project == nullptr) {
        out << "\tSequence provided not in a project, aborting!" << Qt::endl;
        return false;
    }

    StereoVisionApp::Trajectory* trajectory = bilSequence->getAssignedTrajectory();

    if (trajectory == nullptr) {
        out << "\tSequence provided has no assigned trajectory, aborting!" << Qt::endl;
        return false;
    }

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optTrajData =
            trajectory->loadTrajectoryProjectLocalFrameSequence();

    if (!optTrajData.isValid()) {
        out << "\t Could not load trajectory data! Message is: " << optTrajData.errorMessage() << Qt::endl;
        return false;
    }

    StereoVisionApp::Trajectory::TimeTrajectorySequence& trajData = optTrajData.value();

    bool optimized = true;
    std::vector<std::array<double, 3>> viewDirectionsSensor = bilSequence->getSensorViewDirections(optimized);

    if (viewDirectionsSensor.empty()) {
        out << "\t Could not load sensor view direction, aborting!" << Qt::endl;
        return false;
    }

    auto optPos = bilSequence->optPos();
    auto optRot = bilSequence->optRot();

    StereoVision::Geometry::RigidBodyTransform<double> sensor2body;

    if (!optPos.isSet() or !optRot.isSet()) {
        sensor2body.r.setZero();
        sensor2body.t.setZero();
    } else {
        sensor2body.r.x() = optRot.value(0);
        sensor2body.r.y() = optRot.value(1);
        sensor2body.r.z() = optRot.value(2);

        sensor2body.t.x() = optPos.value(0);
        sensor2body.t.y() = optPos.value(1);
        sensor2body.t.z() = optPos.value(2);
    }

    QVector<qint64> imlmids = bilSequence->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

    struct TimingPair {
        double bilSeqY;
        double time;
        double dist;
    };

    QMap<qint64, TimingPair> optTimes;

    for (qint64 imlmid : imlmids) {

        BilSequenceLandmark* blm = bilSequence->getBilSequenceLandmark(imlmid);

        if (blm == nullptr) {
            continue;
        }

        TimingPair timingInfo;
        timingInfo.bilSeqY = blm->y().value();
        timingInfo.time = std::nan("");

        timingInfo.dist = std::numeric_limits<double>::infinity();

        qint64 lmid = blm->attachedLandmarkid();

        StereoVisionApp::Landmark* lm = project->getDataBlock<StereoVisionApp::Landmark>(lmid);

        if (lm == nullptr) {
            out << "\tSkipping missing landmark (id = " << lmid << ")" << Qt::endl;
            continue;
        }

        constexpr bool optimized = true;
        constexpr bool applyProjectLocalTransform = true;
        std::optional<Eigen::Vector3f> optLmPos = lm->getOptimizableCoordinates(optimized, applyProjectLocalTransform);

        if (!optLmPos.has_value()) {
            out << "\tSkipping landmark without optimized position (id = " << lmid << ")" << Qt::endl;
            continue;
        }

        Eigen::Vector3d lmPos = optLmPos.value().cast<double>();

        out << "\tStart treating bil landmark " << imlmid << " (lmid = " << lmid << " pos = " << lmPos.x() << " " << lmPos.y() << " " << lmPos.z() << ")" << Qt::endl;

        Eigen::Vector3d viewDirection;
        double x = blm->x().value();
        int xm = std::floor(x);
        int xp = std::ceil(x);

        double wm = 1;
        double wp = 0;

        if (xp < 0) {
            continue;
        }

        if (xm >= viewDirectionsSensor.size()) {
            continue;
        }

        if (xm < 0) {
            xm = xp;
        }

        if (xp >= viewDirectionsSensor.size()) {
            xp = xm;
        }

        if (xm != xp) {
            wm = xp - x;
            wp = x - xm;
        }

        for (int i = 0; i < 3; i++) {
            viewDirection[i] = wm*viewDirectionsSensor[xm][i] + wp*viewDirectionsSensor[xp][i];
        }

        for (int i = 0; i < trajData.nPoints(); i++) {
            auto trajNode = trajData[i];

            double& time = trajNode.time;
            StereoVision::Geometry::RigidBodyTransform<double>& body2local = trajNode.val;
            StereoVision::Geometry::RigidBodyTransform<double> sensor2local = body2local*sensor2body;

            Eigen::Vector3d viewDirectionLocal = StereoVision::Geometry::angleAxisRotate(sensor2local.r, viewDirection);
            Eigen::Vector3d initialPositionLocal = sensor2local.t;

            //We try to solve initialPositionLocal + l * viewDirectionLocal = lmPos

            double l = viewDirectionLocal.dot(lmPos - initialPositionLocal)/viewDirectionLocal.dot(viewDirectionLocal);

            Eigen::Vector3d approx = initialPositionLocal + l * viewDirectionLocal;
            double dist = (approx - lmPos).norm();

            if (dist < timingInfo.dist) {
                timingInfo.dist = dist;
                timingInfo.time = time;
            }

        }

        if (std::isfinite(timingInfo.time)) {
            optTimes[imlmid] = timingInfo;
        }

    }

    out << "\tSelected timing points: ";
    for (qint64 imlmid : imlmids) {
        if (!optTimes.contains(imlmid)) {
            continue;
        }

        out << "\n" << "\t\tLandmark " << imlmid << " y = " << optTimes[imlmid].bilSeqY << " t = " << optTimes[imlmid].time << " d = " << optTimes[imlmid].dist;
    }

    out << Qt::endl;

    Eigen::Matrix<double,Eigen::Dynamic,2> A;
    Eigen::Matrix<double,Eigen::Dynamic,1> b;

    A.resize(optTimes.size(),2);
    b.resize(optTimes.size(),1);

    int i = 0;

    for (qint64 imlmid : imlmids) {
        if (!optTimes.contains(imlmid)) {
            continue;
        }

        A(i,0) = 1;
        A(i,1) = optTimes[imlmid].bilSeqY;

        b[i] = optTimes[imlmid].time;

        i++;
    }

    Eigen::Vector2d solution = A.fullPivHouseholderQr().solve(b);

    out << "\n\t" << "t = " << solution[1] << " * y + " << solution[0] << "\n" << Qt::endl;

    int nLines = bilSequence->nLinesInSequence();

    constexpr int maxTestLines = 20;
    int step = nLines / maxTestLines;

    for (int i = 0; i < nLines; i += step) {
        double timePredicted = solution[1]*i + solution[0];
        double timeObserved = bilSequence->getTimeFromPixCoord(i);
        out << "\t" << "Line " << (i+1) << " predicted time = " << timePredicted << " observed time = " << timeObserved << " delta = " << (timePredicted - timeObserved) << Qt::endl;
    }

    out << Qt::endl;

    return true;

}


bool analyzeReprojections(BilSequenceAcquisitionData *bilSequence) {

    QTextStream out(stdout);

    out << "Estimate bil sequence reprojections errors:" << Qt::endl;

    if (bilSequence == nullptr) {
        out << "\tNull sequence provided, aborting!" << Qt::endl;
        return false;
    }

    StereoVisionApp::Project* project = bilSequence->getProject();

    if (project == nullptr) {
        out << "\tSequence provided not in a project, aborting!" << Qt::endl;
        return false;
    }

    double sensorWidth = bilSequence->getBilWidth();

    StereoVisionApp::Trajectory* trajectory = bilSequence->getAssignedTrajectory();

    if (trajectory == nullptr) {
        out << "\tSequence provided has no assigned trajectory, aborting!" << Qt::endl;
        return false;
    }

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optTrajData =
            trajectory->optimizedTrajectory();

    if (!optTrajData.isValid()) {
        out << "\t Could not load trajectory data! Message is: " << optTrajData.errorMessage() << Qt::endl;
        return false;
    }

    StereoVisionApp::Trajectory::TimeTrajectorySequence& trajData = optTrajData.value();

    bool optimized = true;
    std::vector<std::array<double, 3>> viewDirectionsSensor = bilSequence->getSensorViewDirections(optimized);

    if (viewDirectionsSensor.empty()) {
        out << "\t Could not load sensor view direction, aborting!" << Qt::endl;
        return false;
    }

    auto optPos = bilSequence->optPos();
    auto optRot = bilSequence->optRot();

    StereoVision::Geometry::RigidBodyTransform<double> body2sensor;

    if (!optPos.isSet() or !optRot.isSet()) {
        body2sensor.r.setZero();
        body2sensor.t.setZero();
    } else {
        body2sensor.r.x() = optRot.value(0);
        body2sensor.r.y() = optRot.value(1);
        body2sensor.r.z() = optRot.value(2);

        body2sensor.t.x() = optPos.value(0);
        body2sensor.t.y() = optPos.value(1);
        body2sensor.t.z() = optPos.value(2);
    }

    double focalLenght = bilSequence->optimizedFLen().value();
    double pp = bilSequence->optimizedOpticalCenterX().value();

    if (!bilSequence->optimizedFLen().isSet()) {
        focalLenght = bilSequence->getFocalLen();
    }

    if (!bilSequence->optimizedOpticalCenterX().isSet()) {
        pp = bilSequence->getBilWidth()/2;
    }

    QVector<qint64> imlmids = bilSequence->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

    for (qint64 imlmid : imlmids) {

        BilSequenceLandmark* blm = bilSequence->getBilSequenceLandmark(imlmid);

        if (blm == nullptr) {
            continue;
        }

        double time = bilSequence->getTimeFromPixCoord(blm->y().value());

        qint64 lmid = blm->attachedLandmarkid();

        StereoVisionApp::Landmark* lm = project->getDataBlock<StereoVisionApp::Landmark>(lmid);

        if (lm == nullptr) {
            out << "\tSkipping missing landmark (id = " << lmid << ")" << Qt::endl;
            continue;
        }

        constexpr bool optimized = true;
        constexpr bool applyProjectLocalTransform = true;
        std::optional<Eigen::Vector3f> optLmPos = lm->getOptimizableCoordinates(optimized, applyProjectLocalTransform);

        if (!optLmPos.has_value()) {
            out << "\tSkipping landmark without optimized position " << lm->objectName() << " (id = " << lmid << ")" << Qt::endl;
            continue;
        }

        auto interpolablePose = trajData.getValueAtTime(time);

        StereoVision::Geometry::RigidBodyTransform<double> pose1topose2 = interpolablePose.valLower.inverse()*interpolablePose.valUpper;
        double w = interpolablePose.weigthUpper;

        StereoVision::Geometry::RigidBodyTransform<double> body2world = interpolablePose.valLower*(w*pose1topose2);
        body2world.t = interpolablePose.weigthLower*interpolablePose.valLower.t + interpolablePose.weigthUpper*interpolablePose.valUpper.t;

        StereoVision::Geometry::RigidBodyTransform<double> world2sensor = body2sensor*body2world.inverse();

        int low = std::floor(blm->x().value());
        int high = std::ceil(blm->x().value());

        low = std::clamp<int>(low, 0, viewDirectionsSensor.size()-1);
        high = std::clamp<int>(high, 0, viewDirectionsSensor.size()-1);

        double alpha = 1 - (blm->x().value() - low);

        Eigen::Vector3d expectedLow = Eigen::Vector3d(viewDirectionsSensor[low][0], viewDirectionsSensor[low][1], viewDirectionsSensor[low][2]);
        Eigen::Vector3d expectedHigh = Eigen::Vector3d(viewDirectionsSensor[high][0], viewDirectionsSensor[high][1], viewDirectionsSensor[high][2]);
        Eigen::Vector3d expected = alpha*expectedLow + (1-alpha)*expectedHigh;
        Eigen::Vector3d projected = world2sensor*optLmPos.value().cast<double>();

        projected /= projected.z();

        projected *= focalLenght;

        double errorU = expected[0] - projected[0];
        double errorV = expected[1] - projected[1];

        out << "\tLandmark " << lm->objectName() << "(" << lmid << "): error u = " << errorU << " error v = " << errorV << Qt::endl;

    }

    return true;
}

} // namespace PikaLTools
