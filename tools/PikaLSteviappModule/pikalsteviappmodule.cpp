#include "pikalsteviappmodule.h"

#include <QTextStream>

#include <steviapp/control/application.h>
#include <steviapp/control/actionmanager.h>
#include <steviapp/control/mainwindow.h>

#include <steviapp/sparsesolver/modularsbasolver.h>

#include "solving/bilsequencesbamodule.h"
#include "solving/dtmtiepointsmodule.h"

#include <steviapp/datablocks/project.h>

#include "datablocks/bilacquisitiondata.h"
#include "datablocks/comparisontrajectory.h"
#include "datablocks/inputdtm.h"

#include "./gui/trajectoryvieweditor.h"
#include "./gui/bilcubevieweditor.h"
#include "./gui/dtmrastervieweditor.h"

#include "actions/bilsequenceactionmanager.h"
#include "actions/comparisonsequenceactionmanager.h"
#include "actions/inputdtmactionmanager.h"

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
    pF.addType(new ComparisonTrajectoryFactory(app));
    pF.addType(new InputDtmFactory(app));

    aml.registerDatablockActionManager(new BilSequenceActionManager(&pF));
    aml.registerDatablockActionManager(new ComparisonTrajectoryActionManager(&pF));
    aml.registerDatablockActionManager(new InputDtmActionManager(&pF));

    StereoVisionApp::MainWindow* w = app->mainWindow();

    if (w != nullptr) {
        w->installEditor(new TrajectoryViewEditorFactory(app));
        w->installEditor(new BilCubeViewEditorFactory(app));
        w->installEditor(new DTMRasterViewEditorFactory(app));
    }

    QObject* interface = app->getAdditionalInterface(StereoVisionApp::SBASolverModulesInterface::AppInterfaceName);

    StereoVisionApp::SBASolverModulesInterface* sbaModuleInterface =
            qobject_cast<StereoVisionApp::SBASolverModulesInterface*>(interface);

    if (sbaModuleInterface != nullptr) {

        sbaModuleInterface->registerSBAModule(BilSequenceSBAModule::ModuleName, [] (StereoVisionApp::ModularSBASolver* solver) -> StereoVisionApp::ModularSBASolver::SBAModule* {
            return new BilSequenceSBAModule();
        });

        sbaModuleInterface->registerSBAModule(DtmTiePointsModule::ModuleName, [] (StereoVisionApp::ModularSBASolver* solver) -> StereoVisionApp::ModularSBASolver::SBAModule* {
            return new DtmTiePointsModule();
        });
    }

    out << "PikaL Steviapp module loaded!" << Qt::endl;

    return 0;
}

int PikaLSteviappModule::cleanupModule(StereoVisionApp::StereoVisionApplication* app) const {
    return 0;
}

} // namespace PikaLTools
