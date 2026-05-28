#include "inputdtmactionmanager.h"

#include "datablocks/inputdtm.h"

#include "inputdtmactions.h"

#include <steviapp/datablocks/landmark.h>

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

void InputDtmActionManager::registerAppHeadlessActions(StereoVisionApp::StereoVisionApplication* application) const {
    constexpr char const* InputDTMNamespace = "InputDTM";

    application->registerHeadlessAction(InputDTMNamespace,"importLandmarks", [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {

        bool ok = true;

        int infosStartId = 0;
        QString idStr = kwargs.value("id", "");

        if (idStr.isEmpty()) {
            idStr = argv[0];
            infosStartId++;
        }

        qint64 id = idStr.toInt(&ok);

        if (!ok) {
            return StereoVisionApp::StatusOptionalReturn<void>::error("Invalid inputDTM id provided, should be either the first argument, or named argument \"id\"!");
        }

        StereoVisionApp::StereoVisionApplication* app = StereoVisionApp::StereoVisionApplication::GetAppInstance();

        if (app == nullptr) {
            return StereoVisionApp::StatusOptionalReturn<void>::error("No active app instance!");
        }

        StereoVisionApp::Project* p = app->getCurrentProject();

        if (p == nullptr) {
            return StereoVisionApp::StatusOptionalReturn<void>::error("No active project!");
        }

        InputDtm* dtm = p->getDataBlock<InputDtm>(id);

        if (dtm == nullptr) {
            return StereoVisionApp::StatusOptionalReturn<void>::error("Invalid inputDTM id: does not correspond to a DTM in the project!");
        }

        for (int i = infosStartId; i < argv.size(); i++) {
            QString const& matchInfos = argv[i];
            QStringList splitted = matchInfos.split(" ", Qt::SkipEmptyParts);

            if (splitted.size() < 3) {
                continue;
            }

            QString name = splitted[0];

            StereoVisionApp::Landmark* lm = p->getDataBlockByName<StereoVisionApp::Landmark>(name);

            if (lm == nullptr) {
                continue;
            }

            bool ok = true;
            double u = splitted[1].toDouble(&ok);
            double v = splitted[2].toDouble(&ok);

            if (!ok) {
                continue;
            }

            dtm->addDtmLandmark(QPointF(u,v), lm->internalId());
        }

        if (!ok) {
            return StereoVisionApp::StatusOptionalReturn<void>::error("Unknwon error!");
        }

        return StereoVisionApp::StatusOptionalReturn<void>();
    });
}

} // namespace PikaLTools
