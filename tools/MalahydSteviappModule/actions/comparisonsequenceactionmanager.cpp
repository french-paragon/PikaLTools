#include "comparisonsequenceactionmanager.h"

#include "datablocks/comparisontrajectory.h"

#include <QAction>

#include "comparisonsequenceactions.h"

namespace PikaLTools {

ComparisonTrajectoryActionManager::ComparisonTrajectoryActionManager(QObject *parent) :
    StereoVisionApp::DatablockActionManager(parent)
{

}

QString ComparisonTrajectoryActionManager::ActionManagerClassName() const {
    return ComparisonTrajectoryActionManager::staticMetaObject.className();
}
QString ComparisonTrajectoryActionManager::itemClassName() const {
    return ComparisonTrajectory::staticMetaObject.className();
}

QList<QAction*> ComparisonTrajectoryActionManager::factorizeClassContextActions(QObject* parent, StereoVisionApp::Project* p) const {

    QAction* add = new QAction(tr("Add trajectory"), parent);
    connect(add, &QAction::triggered, [p] () {
        addComparisonSequence(p);
    });

    return {add};
}
QList<QAction*> ComparisonTrajectoryActionManager::factorizeItemContextActions(QObject* parent, StereoVisionApp::DataBlock* d) const {

    ComparisonTrajectory* seq = qobject_cast<ComparisonTrajectory*>(d);

    if (seq == nullptr) {
        return {};
    }

    QAction* view = new QAction(tr("View trajectory"), parent);
    connect(view, &QAction::triggered, [seq] () {
        viewComparisonTrajectory(seq);
    });

    return {view};
}
QList<QAction*> ComparisonTrajectoryActionManager::factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const {
    return StereoVisionApp::DatablockActionManager::factorizeMultiItemsContextActions(parent, p, projectIndex);
}

} // namespace PikaLTools
