#ifndef BILSEQUENCEACTIONS_H
#define BILSEQUENCEACTIONS_H

#include <QString>

namespace StereoVisionApp{
    class Project;
}

namespace PikaLTools {

class BilSequenceAcquisitionData;

int loadBilSequenceFromFolder(StereoVisionApp::Project* p, QString const& folder = "");

bool showLcfTrajectory(BilSequenceAcquisitionData *bilSequence);

bool showBilImage(BilSequenceAcquisitionData *bilSequence);

bool exportBilLandmarks(BilSequenceAcquisitionData *bilSequence, QString const& outFile = "");

} // namespace PikaLTools

#endif // BILSEQUENCEACTIONS_H
