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

    return {add};

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

    return {view_trajectory};
}

QList<QAction*> BilSequenceActionManager::factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const {
    return StereoVisionApp::DatablockActionManager::factorizeMultiItemsContextActions(parent, p, projectIndex);
}

} // namespace PikaLTools
