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

    struct BilCameraParameters {

        qint64 seqId;
        qint64 sensorId;

        std::array<double, 3> tLeverArm; // lever arm t
        std::array<double, 3> rLeverArm; // lever arm r
    };

    std::vector<BilCameraParameters> _sensorsParameters;
    QMap<qint64, int> _sensorParametersIndex;
    //for bil sequence with sensor index, indicate if the BilCameraParameters for a given sensor index (int) has been instance for a different camera already
    QMap<int, qint64> _sensorIndexMap;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILSEQUENCESBAMODULE_H
