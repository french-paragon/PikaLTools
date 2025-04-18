#ifndef PIKALTOOLS_BILSEQUENCESBAMODULE_H
#define PIKALTOOLS_BILSEQUENCESBAMODULE_H

#include <steviapp/sparsesolver/modularsbasolver.h>

#include <vector>
#include <QMap>

namespace PikaLTools {

class BilSequenceSBAModule : public StereoVisionApp::ModularSBASolver::SBAModule
{
public:

    static const char* ModuleName;
    BilSequenceSBAModule();

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(StereoVisionApp::Project *currentProject,
                                           StereoVisionApp::GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(StereoVisionApp::Project *currentProject,
                                              StereoVisionApp::GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(StereoVisionApp::ModularSBASolver* solver) override;
    virtual bool init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(StereoVisionApp::ModularSBASolver* solver) override;
    virtual bool writeUncertainty(StereoVisionApp::ModularSBASolver* solver) override;
    virtual void cleanup(StereoVisionApp::ModularSBASolver* solver) override;

protected:
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILSEQUENCESBAMODULE_H
