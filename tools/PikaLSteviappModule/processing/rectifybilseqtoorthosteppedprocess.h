#ifndef PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H
#define PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H

#include <steviapp/processing/steppedprocess.h>

#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/geometry/rotations.h>

#include "geo/georasterreader.h"

namespace PikaLTools {

class BilSequenceAcquisitionData;
class InputDtm;

class RectifyBilSeqToOrthoSteppedProcess : public StereoVisionApp::SteppedProcess
{
    Q_OBJECT
public:
    RectifyBilSeqToOrthoSteppedProcess(QObject* parent = nullptr);

    virtual int numberOfSteps() override;
    virtual QString currentStepName() override;

    inline void configure(BilSequenceAcquisitionData* bilSequence, InputDtm* inputDtm, QString const& outFile) {
        _bilSequence = bilSequence;
        _inputDtm = inputDtm;
        _outFile = outFile;
    }

    inline void setMinAndMaxLineId(int min, int max) {
        _minBilLine = min;
        _maxBilLine = max;
    }

    /*!
     * \brief getFinalPatchBand return a move reference to the final patch band raster
     * \return a r-value reference to the patch band raster
     */
    inline Multidim::Array<float,3>&& getFinalPatchBand() {
        return std::move(_patchBand);
    }
    /*!
     * \brief getFinalSamplesInPatch return a move reference to the samples in patch raster
     * \return a r-value refence to the samples in patch raster
     */
    inline Multidim::Array<float,2>&& getFinalSamplesInPatch() {
        return std::move(_samplesInPatch);
    }

    inline void setOutFile(QString const& outFile) {
        _outFile = outFile;
    }

protected:
    virtual bool doNextStep() override;

    virtual bool init() override;
    virtual void cleanup() override;

    InputDtm* _inputDtm;
    GeoRasterData<float, 2> _rasterData;
    float _maxHeight;
    float _minHeight;

    BilSequenceAcquisitionData* _bilSequence;
    QList<QString> _bilPaths;

    std::vector<StereoVision::Geometry::AffineTransform<float>> _ecefTrajectory;
    std::vector<double> _ecefTimes;

    StereoVision::Geometry::AffineTransform<float> _img2ecef_lin;
    StereoVision::Geometry::AffineTransform<float> _ecef2img_lin;

    int _bilStartIdx; //the start index in the bil sequence.
    int _lcfStartIdx; //the start index in the lcf sequence.

    int _bands;

    Multidim::Array<float,3> _patchBand;
    Multidim::Array<float,2> _samplesInPatch;
    Multidim::Array<float,3> _minVal;
    Multidim::Array<float,3> _maxVal;

    double _normalizedSensorSize;

    QString _outFile;

    int _minBilLine;
    int _maxBilLine;

    int _processedLines;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H
