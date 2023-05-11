#ifndef PIKALTOOLS_BILSEQUENCEACTIONMANAGER_H
#define PIKALTOOLS_BILSEQUENCEACTIONMANAGER_H

#include <steviapp/control/actionmanager.h>

namespace PikaLTools {

class BilSequenceActionManager : public StereoVisionApp::DatablockActionManager
{
    Q_OBJECT
public:
    explicit BilSequenceActionManager(QObject *parent = nullptr);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

    QList<QAction*> factorizeClassContextActions(QObject* parent, StereoVisionApp::Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, StereoVisionApp::DataBlock* p) const override;
    QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, StereoVisionApp::Project* p, QModelIndexList const& projectIndex) const override;

Q_SIGNALS:

};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILSEQUENCEACTIONMANAGER_H
