#ifndef PIKALTOOLS_BILSEQUENCESBAMODULE_H
#define PIKALTOOLS_BILSEQUENCESBAMODULE_H

#include <steviapp/sparsesolver/modularsbasolver.h>

#include <vector>
#include <QMap>

namespace PikaLTools {

class BilSequenceSBAModule : public StereoVisionApp::ModularSBASolver::SBAModule
{
public:
    BilSequenceSBAModule();

    virtual bool addGraphReductorVariables(StereoVisionApp::Project *currentProject,
                                           StereoVisionApp::GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(StereoVisionApp::Project *currentProject,
                                              StereoVisionApp::GenericSBAGraphReductor* graphReductor) override;

    virtual bool init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(StereoVisionApp::ModularSBASolver* solver) override;
    virtual bool writeUncertainty(StereoVisionApp::ModularSBASolver* solver) override;
    virtual void cleanup(StereoVisionApp::ModularSBASolver* solver) override;

protected:

    struct BilCameraParameters {

        std::array<double, 3> tLeverArm; // lever arm t
        std::array<double, 3> rLeverArm; // lever arm r

        std::array<double, 1> fLen; //f
        std::array<double, 1> principalPoint; //pp
        std::array<double, 6> lateralDistortion; //as
        std::array<double, 6> frontalDistortion; //bs
    };

    std::vector<BilCameraParameters> _sensorsParameters;
    QMap<qint64, int> _sensorParametersIndex;
    //for bil sequence with sensor index, indicate if the BilCameraParameters for a given sensor index (int) has been instance for a different camera already
    QMap<int, qint64> _sensorIndexMap;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILSEQUENCESBAMODULE_H
