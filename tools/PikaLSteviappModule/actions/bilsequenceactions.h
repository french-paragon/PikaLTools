#ifndef BILSEQUENCEACTIONS_H
#define BILSEQUENCEACTIONS_H

#include <QString>

namespace StereoVisionApp{
    class Project;
}

namespace PikaLTools {

class BilSequenceAcquisitionData;
class InputDtm;

int loadBilSequenceFromFolder(StereoVisionApp::Project* p, QString const& folder = "");

bool showLcfTrajectory(BilSequenceAcquisitionData *bilSequence);

bool showBilImage(BilSequenceAcquisitionData *bilSequence);

bool exportBilLandmarks(BilSequenceAcquisitionData *bilSequence, QString const& outFile = "");

bool computeOrthophoto(BilSequenceAcquisitionData *bilSequence, InputDtm* inputDtm = nullptr, const QString &outFile = "");

bool showCovariance(BilSequenceAcquisitionData *bilSequence);

bool setBilSequenceTrajectory(BilSequenceAcquisitionData *bilSequence, qint64 trajId);

bool initBilSequencesTiePoints();

bool simulatePseudoPushBroomData();
bool refineTrajectoryUsingDn();

} // namespace PikaLTools

#endif // BILSEQUENCEACTIONS_H
