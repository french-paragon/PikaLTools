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

bool assignCamera(BilSequenceAcquisitionData *bilSequence);

bool showLcfTrajectory(BilSequenceAcquisitionData *bilSequence);

bool showBilImage(BilSequenceAcquisitionData *bilSequence);

bool exportBilLandmarks(BilSequenceAcquisitionData *bilSequence, QString const& outFile = "");

bool computeOrthophoto(BilSequenceAcquisitionData *bilSequence, InputDtm* inputDtm = nullptr, const QString &outFile = "");

bool showCovariance(BilSequenceAcquisitionData *bilSequence);

bool setBilSequenceTrajectory(BilSequenceAcquisitionData *bilSequence, qint64 trajId);

bool initBilSequencesTiePoints();

/*!
 * \brief openBilSequenceFolder open the  folder where the bil files are located (if a GUI is active)
 * \param bilSequence the bil sequence
 * \return true on success
 */
bool openBilSequenceFolder(BilSequenceAcquisitionData *bilSequence);

bool simulatePseudoPushBroomData();
bool refineTrajectoryUsingDn();

/*!
 * \brief estimateTimeDeltaRough gives a rough estimate of the time the bil sequence was acquired at (in the form of affine coefficients a*y+b)
 * \param bilSequence the sequence to analyze.
 * \return true on succes, false otherwise.
 */
bool estimateTimeDeltaRough(BilSequenceAcquisitionData *bilSequence);

/*!
 * \brief analyzeReprojections print the errors for all landmarks in the sequences to stdout
 * \param bilSequence the sequence to analyze
 * \return true on success, false in case of error
 */
bool analyzeReprojections(BilSequenceAcquisitionData *bilSequence);

} // namespace PikaLTools

#endif // BILSEQUENCEACTIONS_H
