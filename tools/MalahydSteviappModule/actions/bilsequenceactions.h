#ifndef BILSEQUENCEACTIONS_H
#define BILSEQUENCEACTIONS_H

#include <QString>

#include <optional>

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
bool exportBilRectifiedLandmarks(BilSequenceAcquisitionData *bilSequence, bool optimized = true, QString const& outFile = "");

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

/*!
 * \brief exportTrajectoryWithBilMounting export the trajectory associated with a bil, with the mounting associated with the bil
 * \param bilSequence the bil sequence
 * \return true on success, false otherwise.
 */
bool exportTrajectoryWithBilMounting(BilSequenceAcquisitionData *bilSequence);

bool exportImageGeometry(BilSequenceAcquisitionData *bilSequence);

bool estimateBilShift(BilSequenceAcquisitionData *bilSequence);
bool estimateBilShiftVertical(BilSequenceAcquisitionData *bilSequence);

bool estimateBilShiftHorizontalAndVertical(BilSequenceAcquisitionData *bilSequence);

bool viewPreRectifiedBill(BilSequenceAcquisitionData *bilSequence);

/*!
 * \brief cornerMatchRawBill open a bill sequence in the corner matching editor
 * \param bilSequence the bill sequence to open
 * \param lineMin the start line of the range of lines to load (0 mean first line, negative numbers are wrapped via modulo, nullopt means query the user via a dialog).
 * \param lineMax the end line of the range of lines to load (-1 mean last line, negative numbers are wrapped via modulo, nullopt means query the user via a dialog).
 * \return true on success, false otherwise
 */
bool cornerMatchRawBill(BilSequenceAcquisitionData *bilSequence, std::optional<int> lineMin = std::nullopt, std::optional<int> lineMax = std::nullopt);
/*!
 * \brief cornerMatchPreRectifiedBill open a bill sequence in the corner matching editor, applying pre-rectification on the fly
 * \param bilSequence the bill sequence to open
 * \param lineMin the start line of the range of lines to load (0 mean first line, negative numbers are wrapped via modulo, nullopt means query the user via a dialog).
 * \param lineMax the end line of the range of lines to load (-1 mean last line, negative numbers are wrapped via modulo, nullopt means query the user via a dialog).
 * \return true on success, false otherwise
 */
bool cornerMatchPreRectifiedBill(BilSequenceAcquisitionData *bilSequence, std::optional<int> lineMin = std::nullopt, std::optional<int> lineMax = std::nullopt);

} // namespace PikaLTools

#endif // BILSEQUENCEACTIONS_H
