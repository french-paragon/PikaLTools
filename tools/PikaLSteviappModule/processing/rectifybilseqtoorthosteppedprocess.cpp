#include "rectifybilseqtoorthosteppedprocess.h"

#include <StereoVision/io/image_io.h>
#include <StereoVision/imageProcessing/pixelsLines.h>

#include "../datablocks/bilacquisitiondata.h"
#include "../datablocks/inputdtm.h"

#include "processing/pushbroomprojections.h"

#include <QFileInfo>
#include <QDir>

#include <proj.h>

namespace PikaLTools {

RectifyBilSeqToOrthoSteppedProcess::RectifyBilSeqToOrthoSteppedProcess(QObject *parent) :
    StereoVisionApp::SteppedProcess(parent),
    _bilSequence(nullptr),
    _minBilLine(-1),
    _maxBilLine(-1)
{

}

int RectifyBilSeqToOrthoSteppedProcess::numberOfSteps() {

    if (_bilSequence == nullptr) {
        return 0;
    }

    return _bilSequence->getBilInfos().size();
}

QString RectifyBilSeqToOrthoSteppedProcess::currentStepName() {

    if (_bilSequence == nullptr) {
        return "";
    }

    int step = currentStep();

    if (step == -1) {
        return tr("Initialize");
    }

    if (step >= _bilSequence->getBilFiles().size()) {
        return tr("Clean up");
    }

    QString bilPath = _bilSequence->getBilFiles()[step];

    QFileInfo bilFileInfo(bilPath);

    return tr("Treating %1 (%2)").arg(bilFileInfo.baseName()).arg(currentStep());
}

bool RectifyBilSeqToOrthoSteppedProcess::doNextStep() {

    QString bilPath = _bilPaths[currentStep()];

    BilSequenceAcquisitionData::BilAcquisitionData data(bilPath);

    auto header = readHeaderData(data.bilFilePath().toStdString());

    std::map<std::string, std::string> headerData = header.value();

    int nLines = data.getNLines();
    int nLcfLines = data.getLcfNLines();

    if (_minBilLine >= 0 and _processedLines + nLines < _minBilLine) {

        _processedLines += nLines;

        _bilStartIdx += nLines;
        _lcfStartIdx += nLcfLines;

        return true;
    }

    if (_maxBilLine >= 0 and _processedLines + nLines > _maxBilLine) {

        _processedLines += nLines;

        return true;
    }

    Multidim::Array<float, 3> seq = read_envi_bil_to_float(data.bilFilePath().toStdString());

    int nPixels = seq.shape()[1];
    double fLenPix = static_cast<double>(nPixels)/_normalizedSensorSize;

    Eigen::Matrix3f Rcam2drone = Eigen::Matrix3f::Zero();
    Rcam2drone(0,0) = 1;
    Rcam2drone(1,1) = 1;
    Rcam2drone(2,2) = 1;

    for (int i = 0; i < nLines; i++) {

        if (_minBilLine >= 0 and _processedLines < _minBilLine) {
            _processedLines++;
            continue;
        }

        if (_maxBilLine >= 0 and _processedLines > _maxBilLine) {
            return true;
        }

        _processedLines++;

        float lcfIdx = _lcfStartIdx + i*nLcfLines/(nLines-1); //TODO: try to use better time interpolation

        int lowIdx = static_cast<int>(std::floor(lcfIdx));
        int highIdx = static_cast<int>(std::ceil(lcfIdx));

        if (lowIdx < 0) {
            lowIdx = 0;
        }

        if (highIdx < 0) {
            highIdx = 0;
        }

        if (lowIdx >= _ecefTrajectory.size()) {
            lowIdx = _ecefTrajectory.size()-1;
        }

        if (highIdx >= _ecefTrajectory.size()) {
            highIdx = _ecefTrajectory.size()-1;
        }

        StereoVision::Geometry::AffineTransform<float> pos1 = _ecefTrajectory[lowIdx];
        StereoVision::Geometry::AffineTransform<float> pos2 = _ecefTrajectory[highIdx];

        float w1 = static_cast<float>(highIdx) - lcfIdx;
        float w2 = lcfIdx - static_cast<float>(lowIdx);

        if (highIdx == lowIdx) {
            w1 = 0;
            w2 = 1;
        }

        Eigen::Vector3f r1 = StereoVision::Geometry::inverseRodriguezFormula(pos1.R);
        Eigen::Vector3f r2 = StereoVision::Geometry::inverseRodriguezFormula(pos2.R);

        Eigen::Vector3f r = w1*r1 + w2*r2;
        Eigen::Matrix3f R = StereoVision::Geometry::rodriguezFormula(r);

        StereoVision::Geometry::AffineTransform<float> pos(R*Rcam2drone, w1*pos1.t + w2*pos2.t);

        StereoVision::Geometry::AffineTransform<float> cam2ref = _ecef2img_lin*pos;

        Eigen::Matrix3f inv = cam2ref.R.inverse();
        StereoVision::Geometry::AffineTransform<float> ref2cam(inv, -inv*cam2ref.t);

        Eigen::Vector3f currentCamCenterInImageCoord = _ecef2img_lin*pos.t;

        Eigen::Vector3f viewDirectionLeftPix(-static_cast<float>(nPixels)/2.,0,fLenPix);
        Eigen::Vector3f viewDirectionRightPix(static_cast<float>(nPixels)/2.,0,fLenPix);

        Eigen::Vector3f tmpLeft = pos.R*viewDirectionLeftPix;
        Eigen::Vector3f tmpRight = pos.R*viewDirectionRightPix;

        Eigen::Vector3f currentViewDirectionLeftInImageCoord = _ecef2img_lin.R*tmpLeft;
        Eigen::Vector3f currentViewDirectionRightInImageCoord = _ecef2img_lin.R*tmpRight;

        Eigen::Vector2f intersectionPosLeft = findRayIntersection(_rasterData.raster, currentCamCenterInImageCoord, currentViewDirectionLeftInImageCoord);
        Eigen::Vector2f intersectionPosRight = findRayIntersection(_rasterData.raster, currentCamCenterInImageCoord, currentViewDirectionRightInImageCoord);

        Eigen::Array<float, 2, Eigen::Dynamic> points = StereoVision::ImageProcessing::listPixPointsOnLine(intersectionPosLeft, intersectionPosRight);

        for (int pixelId = 0; pixelId < points.cols(); pixelId++) {

            Eigen::Vector3f pos;
            pos.block<2,1>(0,0) = points.col(pixelId);

            Eigen::Vector2i coord(std::round(pos.x()), std::round(pos.y()));

            if (coord.x() < 0 or coord.x() >= _rasterData.raster.shape()[0]) {
                continue;
            }

            if (coord.y() < 0 or coord.y() >= _rasterData.raster.shape()[1]) {
                continue;
            }

            pos.z() = _rasterData.raster.atUnchecked(coord.x(), coord.y());

            PixReprojection reproj = findPixCoordinateInLine(pos, ref2cam, fLenPix, static_cast<float>(nPixels)/2.);

            if (std::abs(reproj.verticalError) > 0.1) {
                continue;
            }

            float weightGroundPix = (0.5 - std::abs(pos.x()-coord.x()))/0.5 * (0.5 - std::abs(pos.y()-coord.y()))/0.5;

            int pix1 = std::floor(reproj.pixCoord);
            int pix2 = pix1+1;

            if (pix1 < 0 or pix2 >= nPixels) {
                continue;
            }

            float wPix1 = 1-std::abs(pix1 - reproj.pixCoord);
            float wPix2 = 1-wPix1;

            std::array<int, 3> patch_idx;

            patch_idx[0] = coord.x();
            patch_idx[1] = coord.y();

            std::array<int, 3> hypercube_idx1;
            std::array<int, 3> hypercube_idx2;

            hypercube_idx1[LineAxis] = i;
            hypercube_idx2[LineAxis] = i;

            hypercube_idx1[SamplesAxis] = pix1;
            hypercube_idx2[SamplesAxis] = pix2;

            for (int b = 0; b < _bands; b++) {
                patch_idx[2] = b;
                hypercube_idx1[BandsAxis] = b;
                hypercube_idx2[BandsAxis] = b;

                float val1 = seq.valueUnchecked(hypercube_idx1);
                float val2 = seq.valueUnchecked(hypercube_idx2);

                float val = wPix1*val1 + wPix2*val2;

                _patchBand.atUnchecked(patch_idx) += weightGroundPix*val;

                if (_samplesInPatch.valueUnchecked(coord.x(), coord.y()) <= 0) {
                    _minVal.atUnchecked(patch_idx) = val;
                    _maxVal.atUnchecked(patch_idx) = val;
                } else {

                    if (_minVal.valueUnchecked(patch_idx) > val) {
                        _minVal.atUnchecked(patch_idx) = val;
                    }

                    if (_maxVal.valueUnchecked(patch_idx) < val) {
                        _maxVal.atUnchecked(patch_idx) = val;
                    }
                }
            }

            _samplesInPatch.atUnchecked(coord.x(), coord.y()) += weightGroundPix;

        }

    }

    _bilStartIdx += nLines;
    _lcfStartIdx += nLcfLines;

    return true;
}

bool RectifyBilSeqToOrthoSteppedProcess::init() {

    //load dtm data
    auto dtm = readGeoRasterData<float, 2>(_inputDtm->getDataSource().toStdString());

    if (!dtm.has_value()) {
        return false;
    }

    _rasterData = dtm.value();

    _minHeight = _rasterData.raster.valueUnchecked(0,0);
    _maxHeight = _rasterData.raster.valueUnchecked(0,0);

    for (int i = 0; i < _rasterData.raster.shape()[0]; i++) {
        for (int j = 0; j < _rasterData.raster.shape()[1]; j++) {

            float val = _rasterData.raster.valueUnchecked(i,j);

            if (val < _minHeight) {
                _minHeight = val;
            }

            if (val > _maxHeight) {
                _maxHeight = val;
            }

        }
    }

    OGRSpatialReference ogrSpatialRef(_rasterData.crsInfos.c_str());
    bool invertXY = ogrSpatialRef.EPSGTreatsAsLatLong(); //ogr will always treat coordinates as lon then lat, but proj will stick to the epsg order definition. This mean we might need to invert the order.

    Multidim::Array<double,2> corner_vecs({3, 3}, {3,1});

    for (int c = 0; c < 3; c++) {

        int i = 0;
        int j = 0;

        if (c == 1) {
            i = _rasterData.raster.shape()[0]-1;
        }

        if (c == 2) {
            j = _rasterData.raster.shape()[1]-1;
        }

        Eigen::Vector3d homogeneousImgCoord(j,i,1);
        Eigen::Vector2d geoCoord = _rasterData.geoTransform*homogeneousImgCoord;

        if (invertXY) {
            corner_vecs.atUnchecked(c,0) = geoCoord.y();
            corner_vecs.atUnchecked(c,1) = geoCoord.x();
        } else {
            corner_vecs.atUnchecked(c,0) = geoCoord.x();
            corner_vecs.atUnchecked(c,1) = geoCoord.y();
        }

        corner_vecs.atUnchecked(c,2) = 0;
    }


    PJ_CONTEXT* ctx = proj_context_create();


    const char* wgs84_ecef = "EPSG:4978";


    PJ* reprojector = proj_create_crs_to_crs(ctx, _rasterData.crsInfos.c_str(), wgs84_ecef, nullptr);

    if (reprojector == 0) { //in case of error
        return false;
    }


    proj_trans_generic(reprojector, PJ_FWD,
                       &corner_vecs.atUnchecked(0,0), 3*sizeof(double), 3,
                       &corner_vecs.atUnchecked(0,1), 3*sizeof(double), 3,
                       &corner_vecs.atUnchecked(0,2), 3*sizeof(double), 3,
                       nullptr,0,0); //reproject to ecef coordinates


    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    Eigen::Vector3f origin;
    origin << corner_vecs.valueUnchecked(0,0), corner_vecs.valueUnchecked(0,1), corner_vecs.valueUnchecked(0,2);

    Eigen::Vector3f local_frame_x_axis;
    local_frame_x_axis << corner_vecs.valueUnchecked(1,0), corner_vecs.valueUnchecked(1,1), corner_vecs.valueUnchecked(1,2);
    local_frame_x_axis -= origin;
    local_frame_x_axis /= _rasterData.raster.shape()[0]-1;


    Eigen::Vector3f local_frame_y_axis;
    local_frame_y_axis << corner_vecs.valueUnchecked(2,0), corner_vecs.valueUnchecked(2,1), corner_vecs.valueUnchecked(2,2);
    local_frame_y_axis -= origin;
    local_frame_y_axis /= _rasterData.raster.shape()[1]-1;

    Eigen::Vector3f local_frame_z_axis = origin.normalized();

    Eigen::Matrix3f A;
    A.col(0) = local_frame_x_axis;
    A.col(1) = local_frame_y_axis;
    A.col(2) = local_frame_z_axis;

    StereoVision::Geometry::AffineTransform<float> img2ecef_lin(A, origin); //approximate the transform from the image to ecef.

    Eigen::Matrix3f inv = img2ecef_lin.R.inverse();
    StereoVision::Geometry::AffineTransform<float> ecef2img_lin(inv, -inv*origin); //this can map points from ecef to img+alt coordinates

    _ecef2img_lin = ecef2img_lin;

    //load bill data

    _ecefTrajectory = _bilSequence->ecefTrajectory();
    _ecefTimes = _bilSequence->ecefTimes();
    _bilPaths = _bilSequence->getBilFiles();

    _processedLines = 0;

    auto header = readHeaderData(_bilPaths.first().toStdString());

    if (!header.has_value()) {
        return false;
    }

    if (header->count("bands") <= 0) {
        return false;
    }

    try {
        _bands = std::stoi(header.value()["bands"]);
    }
    catch(std::invalid_argument const& e) {
        return false;
    }

    float fieldOfView;

    if (header->count("field of view") <= 0) {
        return false;
    }

    try {
        fieldOfView = std::stof(header.value()["field of view"]);
    }
    catch(std::invalid_argument const& e) {
        return false;
    }

    float fov_rad = fieldOfView/180*M_PI;
    _normalizedSensorSize = 2*std::tan(fov_rad/2);

    _patchBand = Multidim::Array<float,3>(_rasterData.raster.shape()[0], _rasterData.raster.shape()[1], _bands);

    _samplesInPatch = Multidim::Array<float,2>(_rasterData.raster.shape()[0], _rasterData.raster.shape()[1]);

    _minVal = Multidim::Array<float,3>(_rasterData.raster.shape()[0], _rasterData.raster.shape()[1], _bands);
    _maxVal = Multidim::Array<float,3>(_rasterData.raster.shape()[0], _rasterData.raster.shape()[1], _bands);

    for (int i = 0; i < _rasterData.raster.shape()[0]; i++) {
        for (int j = 0; j < _rasterData.raster.shape()[1]; j++) {

            _samplesInPatch.atUnchecked(i,j) = 0;

            for (int b = 0; b < _bands; b++) {
                _patchBand.atUnchecked(i,j,b) = 0;
                _minVal.atUnchecked(i,j,b) = 0;
                _maxVal.atUnchecked(i,j,b) = 0;
            }

        }
    }

    _bilStartIdx = 0;
    _lcfStartIdx = 0;

    return true;
}

void RectifyBilSeqToOrthoSteppedProcess::cleanup() {

    for (int i = 0; i < _patchBand.shape()[0]; i++) {
        for (int s = 0; s < _patchBand.shape()[1]; s++) {

            double coeff = _samplesInPatch.atUnchecked(i,s);

            if (coeff <= 0) {
                continue;
            }

            for (int b = 0; b < _patchBand.shape()[2]; b++) {
                _patchBand.atUnchecked(i,s,b) /= coeff;

                if (std::isinf(_patchBand.atUnchecked(i,s,b)) or
                        std::isnan(_patchBand.atUnchecked(i,s,b))) {
                    _patchBand.atUnchecked(i,s,b) = 0;
                }
            }
        }
    }

    QFileInfo infos(_outFile);
    QDir dir = infos.dir();

    std::cout << "Writing data to: " << dir.absolutePath().toStdString() << std::endl;

    StereoVision::IO::writeStevimg<float>(dir.absoluteFilePath(infos.fileName()).toStdString(), _patchBand);
    StereoVision::IO::writeStevimg<float>(dir.absoluteFilePath(infos.baseName() + "_min." + infos.suffix()).toStdString(), _minVal);
    StereoVision::IO::writeStevimg<float>(dir.absoluteFilePath(infos.baseName() + "_max." + infos.suffix()).toStdString(), _maxVal);
    StereoVision::IO::writeStevimg<float>(dir.absoluteFilePath(infos.baseName() + "_samples." + infos.suffix()).toStdString(), _samplesInPatch);
}

} // namespace PikaLTools
