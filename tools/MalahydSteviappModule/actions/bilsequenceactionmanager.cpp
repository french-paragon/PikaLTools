#include "bilsequenceactionmanager.h"

#include "../datablocks/bilacquisitiondata.h"

#include "bilsequenceactions.h"

#include <steviapp/datablocks/cameras/pushbroompinholecamera.h>
#include <steviapp/control/mainwindow.h>

#include <QAction>
#include <QMessageBox>
#include <QMenu>

namespace PikaLTools {

BilSequenceActionManager::BilSequenceActionManager(QObject *parent)
    : StereoVisionApp::DatablockActionManager{parent}
{

}

QString BilSequenceActionManager::ActionManagerClassName() const {
    return BilSequenceActionManager::staticMetaObject.className();
}
QString BilSequenceActionManager::itemClassName() const {
    return BilSequenceAcquisitionData::staticMetaObject.className();
}

QList<QAction*> BilSequenceActionManager::factorizeClassContextActions(QObject* parent, StereoVisionApp::Project* p) const {

    QAction* add = new QAction(tr("Add bil sequence"), parent);
    connect(add, &QAction::triggered, [p] () {
        loadBilSequenceFromFolder(p);
    });

    QAction* generateSimulated = new QAction(tr("Generate simulated bil sequence"), parent);
    connect(generateSimulated, &QAction::triggered, [p] () {
        simulatePseudoPushBroomData();
    });

    QAction* initLandmarks = new QAction(tr("Init landmarks from bil sequences"), parent);
    connect(initLandmarks, &QAction::triggered, [] () {
        bool ok = initBilSequencesTiePoints();

        if (!ok) {
            StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();
            QMessageBox::warning(mw, tr("Could not init tie points from bil sequence"), tr("Unknown error"));
        }
    });

    QAction* optimizeSeqs = new QAction(tr("Optimize bil sequences"), parent);
    connect(optimizeSeqs, &QAction::triggered, [] () {
        refineTrajectoryUsingDn();
    });

    return {add, generateSimulated, initLandmarks, optimizeSeqs};

}

QList<QAction*> BilSequenceActionManager::factorizeItemContextActions(QObject* parent, StereoVisionApp::DataBlock* d) const {

    BilSequenceAcquisitionData* bilSeq = qobject_cast<BilSequenceAcquisitionData*>(d);

    if (bilSeq == nullptr) {
        return {};
    }

    QAction* view_bil_cube = new QAction(tr("View hyperspectral cube"), parent);

    connect(view_bil_cube, &QAction::triggered, [bilSeq] () {
        showBilImage(bilSeq);
    });

    QAction* assignToCamera = createAssignToCameraAction(parent, bilSeq->getProject(), {bilSeq});

    QAction* export_landmarks = new QAction(tr("Export landmarks to csv"), parent);

    connect(export_landmarks, &QAction::triggered, [bilSeq] () {
        exportBilLandmarks(bilSeq);
    });

    QAction* view_files = new QAction(tr("View files on disk"), parent);

    connect(view_files, &QAction::triggered, [bilSeq] () {
        openBilSequenceFolder(bilSeq);
    });

    QAction* compute_orthophoto = new QAction(tr("Compute orthophoto"), parent);

    connect(compute_orthophoto, &QAction::triggered, [bilSeq] () {
        computeOrthophoto(bilSeq);
    });

    QAction* compute_corrMat = new QAction(tr("Show data correlation"), parent);

    connect(compute_corrMat, &QAction::triggered, [bilSeq] () {
        showCovariance(bilSeq);
    });

    QAction* assignTraj2Seq = new QAction(tr("Assign trajectory"), parent);
    connect(assignTraj2Seq, &QAction::triggered, [bilSeq] () {
        setBilSequenceTrajectory(bilSeq, -1);
    });

    QAction* estimateTime = new QAction(tr("Estimate time"), parent);
    connect(estimateTime, &QAction::triggered, [bilSeq] () {
        estimateTimeDeltaRough(bilSeq);
    });

    QAction* analyzeProj = new QAction(tr("Analyze reprojections"), parent);
    connect(analyzeProj, &QAction::triggered, [bilSeq] () {
        analyzeReprojections(bilSeq);
    });

    return {assignToCamera, assignTraj2Seq, view_bil_cube, view_files, export_landmarks, compute_orthophoto, compute_corrMat, estimateTime, analyzeProj};
}

QList<QAction*> BilSequenceActionManager::factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const {
    return StereoVisionApp::DatablockActionManager::factorizeMultiItemsContextActions(parent, p, projectIndex);
}

QAction* BilSequenceActionManager::createAssignToCameraAction(QObject* parent,
                                                              StereoVisionApp::Project* p,
                                                              QVector<BilSequenceAcquisitionData*> const& seqs) const {

    QAction* assignToCamera = new QAction(tr("Assign to camera"), parent);
    QMenu* camMenu = new QMenu();
    connect(assignToCamera, &QObject::destroyed, camMenu, &QObject::deleteLater);

    QVector<qint64> camsIds = p->getIdsByClass(StereoVisionApp::PushBroomPinholeCamera::staticMetaObject.className());

    for(qint64 camId : camsIds) {

        StereoVisionApp::PushBroomPinholeCamera* c = qobject_cast<StereoVisionApp::PushBroomPinholeCamera*>(p->getById(camId));

        if (c != nullptr) {
            QAction* toCam = new QAction(c->objectName(), assignToCamera);
            connect(toCam, &QAction::triggered, [camId, seqs] () {
                for (BilSequenceAcquisitionData* seq : seqs) {
                    seq->assignCamera(camId);
                }
            });
            camMenu->addAction(toCam);
        }
    }
    assignToCamera->setMenu(camMenu);

    return assignToCamera;

}

} // namespace PikaLTools
