#include "bilsequenceactionmanager.h"

#include "../datablocks/bilacquisitiondata.h"

#include "bilsequenceactions.h"

#include <steviapp/datablocks/mounting.h>
#include <steviapp/datablocks/cameras/pushbroompinholecamera.h>
#include <steviapp/control/mainwindow.h>

#include <QAction>
#include <QMessageBox>
#include <QMenu>
#include <QDialog>
#include <QCheckBox>
#include <QLabel>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QComboBox>

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

    return {add, generateSimulated, initLandmarks};

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

    QAction* assignToMounting = createAssignToMountingAction(parent, bilSeq->getProject(), {bilSeq});

    QAction* export_landmarks = new QAction(tr("Export landmarks to csv"), parent);

    connect(export_landmarks, &QAction::triggered, [bilSeq] () {

        QDialog exportOptions;
        QFormLayout layout(&exportOptions);

        QComboBox exportBox(&exportOptions);
        exportBox.addItem(QObject::tr("Unrectified"), false);
        exportBox.addItem(QObject::tr("Rectified"), true);
        QCheckBox optimizedBox(&exportOptions);

        layout.addRow(QObject::tr("Export:"), &exportBox);
        layout.addRow(QObject::tr("Optimized:"), &optimizedBox);

        QDialogButtonBox buttonBox(&exportOptions);

        buttonBox.addButton(QDialogButtonBox::StandardButton::Ok);
        buttonBox.addButton(QDialogButtonBox::StandardButton::Cancel);

        QObject::connect(&buttonBox, &QDialogButtonBox::accepted,
                         &exportOptions, &QDialog::accept);
        QObject::connect(&buttonBox, &QDialogButtonBox::rejected,
                         &exportOptions, &QDialog::reject);

        layout.addRow(&buttonBox);

        int code = exportOptions.exec();

        if (code != QDialog::Accepted) {
            return;
        }

        bool rectified = exportBox.currentData().toBool();
        bool optimized = optimizedBox.isChecked();

        if (rectified) {
            exportBilRectifiedLandmarks(bilSeq, optimized);
        } else {
            exportBilLandmarks(bilSeq);
        }
    });

    QAction* view_files = new QAction(tr("View files on disk"), parent);

    connect(view_files, &QAction::triggered, [bilSeq] () {
        openBilSequenceFolder(bilSeq);
    });

    QAction* compute_orthophoto = new QAction(tr("Compute orthophoto"), parent);

    connect(compute_orthophoto, &QAction::triggered, [bilSeq] () {
        computeOrthophoto(bilSeq);
    });

    QAction* exportIgm = new QAction(tr("Export image geometry"), parent);
    connect(exportIgm, &QAction::triggered, [bilSeq] () {
        exportImageGeometry(bilSeq);
    });

    QAction* analyzeShift = new QAction(tr("Analyze horizontal shift"), parent);
    connect(analyzeShift, &QAction::triggered, [bilSeq] () {
        estimateBilShift(bilSeq);
    });

    QAction* analyzeVShift = new QAction(tr("Analyze vertical shift"), parent);
    connect(analyzeVShift, &QAction::triggered, [bilSeq] () {
        estimateBilShiftVertical(bilSeq);
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

    QAction* exportTraj = new QAction(tr("Export trajectory with mounting"), parent);
    connect(exportTraj, &QAction::triggered, [bilSeq] () {
        exportTrajectoryWithBilMounting(bilSeq);
    });

    QList<QAction*> ret{assignToCamera,
                         assignTraj2Seq,
                         assignToMounting,
                         view_bil_cube,
                         view_files,
                         export_landmarks,
                         compute_orthophoto,
                         exportIgm,
                         compute_corrMat,
                         analyzeShift,
                         analyzeVShift,
                         estimateTime,
                         analyzeProj,
                         exportTraj};

    ret.append(DatablockActionManager::factorizeItemContextActions(parent, d));

    return ret;
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

QAction* BilSequenceActionManager::createAssignToMountingAction(QObject* parent,
                                                                StereoVisionApp::Project* p,
                                                                QVector<BilSequenceAcquisitionData*> const& seqs) const {

    QAction* assignToMounting = new QAction(tr("Assign to mounting"), parent);
    QMenu* mountingMenu = new QMenu();
    connect(assignToMounting, &QObject::destroyed, mountingMenu, &QObject::deleteLater);

    QVector<qint64> mountingIds = p->getIdsByClass(StereoVisionApp::Mounting::staticMetaObject.className());

    for(qint64 mountingId : mountingIds) {

        StereoVisionApp::Mounting* m = qobject_cast<StereoVisionApp::Mounting*>(p->getById(mountingId));

        if (m != nullptr) {
            QAction* toMounting = new QAction(m->objectName(), assignToMounting);
            connect(toMounting, &QAction::triggered, [mountingId, seqs] () {
                for (BilSequenceAcquisitionData* seq : seqs) {
                    seq->assignMounting(mountingId);
                }
            });
            mountingMenu->addAction(toMounting);
        }
    }
    assignToMounting->setMenu(mountingMenu);

    return assignToMounting;

}

} // namespace PikaLTools
