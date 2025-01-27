#ifndef PIKALTOOLS_INPUTDTMACTIONMANAGER_H
#define PIKALTOOLS_INPUTDTMACTIONMANAGER_H

#include <steviapp/control/actionmanager.h>


namespace PikaLTools {

class InputDtmActionManager : public StereoVisionApp::DatablockActionManager
{
    Q_OBJECT
public:
    InputDtmActionManager(QObject *parent = nullptr);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

    QList<QAction*> factorizeClassContextActions(QObject* parent, StereoVisionApp::Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, StereoVisionApp::DataBlock* d) const override;
    QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const override;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_INPUTDTMACTIONMANAGER_H
