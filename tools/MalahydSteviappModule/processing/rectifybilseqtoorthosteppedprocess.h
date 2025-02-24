#ifndef PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H
#define PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H

#include <steviapp/processing/steppedprocess.h>
#include <steviapp/geo/terrainProjector.h>
#include <steviapp/vision/indexed_timed_sequence.h>

#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/geometry/rotations.h>

#include "io/georasterreader.h"

#include <QTemporaryDir>

namespace PikaLTools {

class BilSequenceAcquisitionData;
class InputDtm;

class RectifyBilSeqToOrthoSteppedProcess : public StereoVisionApp::SteppedProcess
{
    Q_OBJECT
public:
    RectifyBilSeqToOrthoSteppedProcess(QObject* parent = nullptr);
    ~RectifyBilSeqToOrthoSteppedProcess();

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

    inline void useOptimizedTrajectory(bool useOptimized) {
        _useOptimzedTrajectory = useOptimized;
    }

    inline void useOptimizedCamera(bool useOptimized) {
        _useOptimzedCamera = useOptimized;
    }
    inline void useOptimizedLeverArm(bool useOptimized) {
        _useOptimzedLeverArm = useOptimized;
    }

    inline void setOutFile(QString const& outFile) {
        _outFile = outFile;
    }

    inline void setTargetGSD(double gsd) {
        _target_gsd = gsd;
    }

    inline int inPaintingRadius() const {
        return _inPaintingRadius;
    }
    inline void setInPaintingRadius(int newInPaintingRadius) {
        _inPaintingRadius = newInPaintingRadius;
    }

protected:

    template<typename CT>
    using Trajectory = StereoVisionApp::IndexedTimeSequence<StereoVision::Geometry::RigidBodyTransform<CT>, CT>;

    virtual bool doNextStep() override;

    virtual bool init() override;
    virtual void cleanup() override;

    bool computeBilProjection(int bilId);
    bool computeProjectedGrid();
    bool computeNextTile(int tileId);

    InputDtm* _inputDtm;
    StereoVisionApp::Geo::GeoRasterData<double,2> _terrain;
    StereoVisionApp::Geo::TerrainProjector<double>* _terrain_projector;

    double _map_scale; //scale between the map coordinates in the dsm and the reprojected grid

    BilSequenceAcquisitionData* _bilSequence;
    QList<QString> _bilPaths;

    struct BilROI {
        double minX;
        double maxX;
        double minY;
        double maxY;
    };
    std::vector<BilROI> _bilFilesROI;

    Trajectory<double> _bil_trajectory;

    struct GridBlock {
        int height; //height in pixel
        int width; //width in pixel

        //Index of the tile in the tile grid
        int pi;
        int pj;

        //min corner (in terrain raster pixels coordinates)
        double i0;
        double j0;

        //max corner (in terrain raster pixels coordinates)
        double in;
        double jn;
    };

    std::vector<GridBlock> _exportGrid;

    int _bilStartIdx; //the start index in the bil sequence.

    QTemporaryDir* _tmp_folder;
    QString _outFile;

    int _minBilLine;
    int _maxBilLine;

    int _nSamples; //number of pixels in a line.
    int _bands; //number of spectral bands in the sequence.

    double _target_gsd;
    double _max_tile_width;

    bool _useOptimzedTrajectory;
    bool _useOptimzedCamera;
    bool _useOptimzedLeverArm;

    int _redChannel;
    int _greenChannel;
    int _blueChannel;

    int _inPaintingRadius;

    double _f_len_pix;
    double _optical_center;

    int _processedLines;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_RECTIFYBILSEQTOORTHOSTEPPEDPROCESS_H
