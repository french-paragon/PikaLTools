#include "bilsequenceactionmanager.h"

#include "../datablocks/bilacquisitiondata.h"

#include "bilsequenceactions.h"

#include <QAction>

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

    return {add, generateSimulated};

}

QList<QAction*> BilSequenceActionManager::factorizeItemContextActions(QObject* parent, StereoVisionApp::DataBlock* d) const {

    BilSequenceAcquisitionData* bilSeq = qobject_cast<BilSequenceAcquisitionData*>(d);

    if (bilSeq == nullptr) {
        return {};
    }

    QAction* view_trajectory = new QAction(tr("View lcf trajectory"), parent);

    connect(view_trajectory, &QAction::triggered, [bilSeq] () {
        showLcfTrajectory(bilSeq);
    });

    QAction* view_bil_cube = new QAction(tr("View hyperspectral cube"), parent);

    connect(view_bil_cube, &QAction::triggered, [bilSeq] () {
        showBilImage(bilSeq);
    });

    QAction* export_landmarks = new QAction(tr("Export landmarks to csv"), parent);

    connect(export_landmarks, &QAction::triggered, [bilSeq] () {
        exportBilLandmarks(bilSeq);
    });

    QAction* compute_orthophoto = new QAction(tr("Compute orthophoto"), parent);

    connect(compute_orthophoto, &QAction::triggered, [bilSeq] () {
        computeOrthophoto(bilSeq);
    });

    QAction* compute_corrMat = new QAction(tr("Show data correlation"), parent);

    connect(compute_corrMat, &QAction::triggered, [bilSeq] () {
        showCovariance(bilSeq);
    });

    QAction* optimizeSeq = new QAction(tr("Optimize bil sequence"), parent);
    connect(optimizeSeq, &QAction::triggered, [bilSeq] () {
        refineTrajectoryUsingDn(bilSeq);
    });

    return {view_trajectory, view_bil_cube, export_landmarks, compute_orthophoto, compute_corrMat, optimizeSeq};
}

QList<QAction*> BilSequenceActionManager::factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const {
    return StereoVisionApp::DatablockActionManager::factorizeMultiItemsContextActions(parent, p, projectIndex);
}

} // namespace PikaLTools
