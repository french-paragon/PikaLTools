#ifndef PIKALTOOLS_PIKALSTEVIAPPMODULE_H
#define PIKALTOOLS_PIKALSTEVIAPPMODULE_H

#include <QObject>

#include <steviapp/control/stereoappplugininterface.h>

namespace PikaLTools {

class PikaLSteviappModule : public QObject, public StereoVisionApp::StereoAppPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID StereoVisionApp_PluginInterface_iid)
    Q_INTERFACES(StereoVisionApp::StereoAppPluginInterface)

public:
    explicit PikaLSteviappModule(QObject *parent = nullptr);

    virtual int loadModule(StereoVisionApp::StereoVisionApplication* app) const override;
    virtual int cleanupModule(StereoVisionApp::StereoVisionApplication* app) const override;

Q_SIGNALS:

};

} // namespace PikaLTools

#endif // PIKALTOOLS_PIKALSTEVIAPPMODULE_H
