#ifndef PIKALTOOLS_DTMTIEPOINTSMODULE_H
#define PIKALTOOLS_DTMTIEPOINTSMODULE_H

#include <steviapp/sparsesolver/modularsbasolver.h>

#include <vector>
#include <QMap>


namespace PikaLTools {

class DtmTiePointsModule : public StereoVisionApp::ModularSBASolver::SBAModule
{
public:
    DtmTiePointsModule();

    virtual bool init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem);
    virtual bool writeResults(StereoVisionApp::ModularSBASolver* solver);
    virtual bool writeUncertainty(StereoVisionApp::ModularSBASolver* solver);
    virtual void cleanup(StereoVisionApp::ModularSBASolver* solver);

protected:

};

} // namespace PikaLTools

#endif // PIKALTOOLS_DTMTIEPOINTSMODULE_H
