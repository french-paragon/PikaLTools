#include "pikalsteviappmodule.h"

#include <QTextStream>

#include <steviapp/control/application.h>
#include <steviapp/control/actionmanager.h>
#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/project.h>

#include "datablocks/bilacquisitiondata.h"

#include "./gui/trajectoryvieweditor.h"
#include "./gui/bilcubevieweditor.h"

#include "actions/bilsequenceactionmanager.h"

namespace PikaLTools {

PikaLSteviappModule::PikaLSteviappModule(QObject *parent) :
    QObject(parent),
    StereoVisionApp::StereoAppPluginInterface()
{

}

int PikaLSteviappModule::loadModule(StereoVisionApp::StereoVisionApplication* app) const {

    QTextStream out(stdout);

    out << "Loading the PikaL Steviapp module... " << Qt::endl;

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();
    StereoVisionApp::ActionManagersLibrary& aml = StereoVisionApp::ActionManagersLibrary::defaultActionManagersLibrary();

    pF.addType(new BilSequenceAcquisitionDataFactory(app));

    aml.registerDatablockActionManager(new BilSequenceActionManager(&pF));

    StereoVisionApp::MainWindow* w = app->mainWindow();

    if (w != nullptr) {
        w->installEditor(new TrajectoryViewEditorFactory(app));
        w->installEditor(new BilCubeViewEditorFactory(app));
    }

    out << "PikaL Steviapp module loaded!" << Qt::endl;

    return 0;
}

int PikaLSteviappModule::cleanupModule(StereoVisionApp::StereoVisionApplication* app) const {
    return 0;
}

} // namespace PikaLTools