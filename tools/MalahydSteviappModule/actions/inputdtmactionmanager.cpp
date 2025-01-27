#include "inputdtmactionmanager.h"

#include "datablocks/inputdtm.h"

#include "inputdtmactions.h"

#include <QAction>

namespace PikaLTools {

InputDtmActionManager::InputDtmActionManager(QObject *parent) :
    StereoVisionApp::DatablockActionManager(parent)
{

}

QString InputDtmActionManager::ActionManagerClassName() const {
    return InputDtmActionManager::staticMetaObject.className();
}
QString InputDtmActionManager::itemClassName() const {
    return InputDtm::staticMetaObject.className();
}

QList<QAction*> InputDtmActionManager::factorizeClassContextActions(QObject* parent,
                                                                    StereoVisionApp::Project* p) const {

    QAction* add = new QAction(tr("Add input dtm"), parent);
    connect(add, &QAction::triggered, [p] () {
        addInputDtm(p);
    });

    return {add};

}
QList<QAction*> InputDtmActionManager::factorizeItemContextActions(QObject* parent,
                                                                   StereoVisionApp::DataBlock* d) const {

    InputDtm* dtm = qobject_cast<InputDtm*>(d);

    if (dtm == nullptr) {
        return {};
    }

    QAction* view = new QAction(tr("View input dtm"), parent);
    connect(view, &QAction::triggered, [dtm] () {
        viewInputDtm(dtm);
    });

    QAction* view2d = new QAction(tr("View input dtm raster"), parent);
    connect(view2d, &QAction::triggered, [dtm] () {
        viewInputDtm2D(dtm);
    });

    return {view2d, view};

}
QList<QAction*> InputDtmActionManager::factorizeMultiItemsContextActions(QObject* parent,
                                                                         StereoVisionApp::Project* p,
                                                                         QModelIndexList const& projectIndex) const {
    return StereoVisionApp::DatablockActionManager::factorizeMultiItemsContextActions(parent, p, projectIndex);
}

} // namespace PikaLTools
