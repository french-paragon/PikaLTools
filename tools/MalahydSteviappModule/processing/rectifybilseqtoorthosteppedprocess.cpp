#include "rectifybilseqtoorthosteppedprocess.h"

#include <StereoVision/io/image_io.h>
#include <StereoVision/imageProcessing/pixelsLines.h>
#include <StereoVision/imageProcessing/convolutions.h>
#include <StereoVision/imageProcessing/standardConvolutionFilters.h>
#include <StereoVision/imageProcessing/inpainting.h>
#include <StereoVision/imageProcessing/morphologicalOperators.h>
#include <StereoVision/interpolation/interpolation.h>

#include <steviapp/datablocks/trajectory.h>
#include <steviapp/datablocks/mounting.h>
#include <steviapp/datablocks/cameras/pushbroompinholecamera.h>

#include "../datablocks/bilacquisitiondata.h"
#include "../datablocks/inputdtm.h"

#include "processing/pushbroomprojections.h"

#include <QFileInfo>
#include <QDir>
#include <QRectF>
#include <QPointF>

#include <proj.h>

namespace PikaLTools {

RectifyBilSeqToOrthoSteppedProcess::RectifyBilSeqToOrthoSteppedProcess(QObject *parent) :
    StereoVisionApp::SteppedProcess(parent),
    _mode(ExportOrtho),
    _bilSequence(nullptr),
    _minBilLine(-1),
    _maxBilLine(-1),
    _useOptimzedTrajectory(true),
    _useOptimzedCamera(true),
    _useOptimzedLeverArm(true),
    _terrain_projector(nullptr),
    _tmp_folder(nullptr),
    _outFile(""),
    _outCrs("")
{

    _target_gsd = 0.5;
    _max_tile_width = 2000;
    _inPaintingRadius = 4;

    _redChannel = 43;
    _greenChannel = 27;
    _blueChannel = 12;

}
RectifyBilSeqToOrthoSteppedProcess::RectifyBilSeqToOrthoSteppedProcess(Mode mode, QObject* parent) :
    RectifyBilSeqToOrthoSteppedProcess(parent)
{
    _mode = mode;
}

RectifyBilSeqToOrthoSteppedProcess::~RectifyBilSeqToOrthoSteppedProcess() {
    RectifyBilSeqToOrthoSteppedProcess::cleanup();
}

int RectifyBilSeqToOrthoSteppedProcess::numberOfSteps() {

    if (_bilSequence == nullptr) {
        return 0;
    }

    //one projection step for each bil file.
    int nSteps = _bilSequence->getBilInfos().size();

    if (_mode == ExportImageGeometry) {
        return 2*nSteps; //one step to project, one step to write
    }

    //one step to configure the grid.
    nSteps += 1;

    //one step per grid cell (min 1 to set error if empty grid
    //and to ensure the process do not stop before starting export)
    nSteps += std::max<int>(_exportGrid.size(),1);

    return nSteps;
}

QString RectifyBilSeqToOrthoSteppedProcess::currentStepName() {

    if (_bilSequence == nullptr) {
        return "";
    }

    int step = currentStep();

    if (step == -1) {
        return tr("Initialize");
    }

    int nBils = _bilSequence->getBilInfos().size();

    if (step < nBils) {

        QString bilPath = _bilSequence->getBilFiles()[step];

        QFileInfo bilFileInfo(bilPath);

        return tr("Treating bil %1 (%2/%3)").arg(bilFileInfo.baseName()).arg(step+1).arg(nBils);
    }

    if (_mode == ExportImageGeometry) {

        if (step - nBils < nBils) {

            QString bilPath = _bilSequence->getBilFiles()[step - nBils];

            QFileInfo bilFileInfo(bilPath);

            return tr("Exporting bil %1 (%2/%3)").arg(bilFileInfo.baseName()).arg(step+1-nBils).arg(nBils);
        }

        return tr("Cleaning up");
    }


    if (step == nBils) {
        return tr("Building grid");
    }

    if (step > nBils and step <= _exportGrid.size() + nBils) {

        int gridBlockId = step - nBils - 1;

        return tr("Treating grid block (%1/%2)").arg(gridBlockId+1).arg(_exportGrid.size());

    }

    return tr("Cleaning up");
}
bool RectifyBilSeqToOrthoSteppedProcess::doNextStep() {

    QTextStream out(stdout);

    if (_bilSequence == nullptr) {
        return false;
    }

    int step = currentStep();

    if (step == -1) {
        return false;
    }

    out << "Running step " << (step+1) << "/" << numberOfSteps() << Qt::endl;

    int nBils = _bilSequence->getBilInfos().size();

    if (step < nBils) {
        int bilId = step;
        out << "\tComputing projection " << bilId << Qt::endl;
        return computeBilProjection(bilId);
    }

    if (_mode == ExportImageGeometry) {

        int bilId = step - nBils;
        out << "\tExporting projection " << bilId << Qt::endl;
        return exportBilProjection(bilId, _outFile, _outCrs);
    }

    if (step == nBils) {

        if (_terrain_projector != nullptr) {
            delete _terrain_projector; //clean up memory
            _terrain_projector = nullptr;
        }

        out << "\tComputing projection grid" << Qt::endl;
        return computeProjectedGrid();
    }

    if (step > nBils and step <= _exportGrid.size() + nBils) {

        int tileId = step - nBils - 1;

        out << "\tComputing tile " << tileId << Qt::endl;

        return computeNextTile(tileId);
    }

    return false;
}

bool RectifyBilSeqToOrthoSteppedProcess::computeBilProjection(int bilId) {

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();

    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    QTextStream out(stdout);

    QString bil_file_path = _bilSequence->getBilInfos()[bilId].bilFilePath();

    QFileInfo bil_file_info(bil_file_path);

    if (!bil_file_info.exists()) {
        return false;
    }

    StereoVisionApp::Mounting* leverArm = _bilSequence->getAssignedMounting();

    if (leverArm == nullptr) {
        return false;
    }

    std::vector<std::array<double, 3>> viewDirectionsSensor = _bilSequence->getSensorViewDirections(_useOptimzedCamera);

    std::vector<double> times = get_envi_bil_lines_times(bil_file_path.toStdString());

    int nLines = times.size();

    if (nLines <= 0) {
        return false;
    }

    for (double& t : times) {
        t = _bilSequence->timeScale()*t + _bilSequence->timeDelta(); //transform the times.
    }

    QString tmpFileName = QString("bilProj%1.bin").arg(bilId);

    QString tmpFilePath = _tmp_folder->filePath(tmpFileName);

    QFile tmpProjections(tmpFilePath);

    bool ok = tmpProjections.open(QFile::WriteOnly);

    if (!ok) {
        return false;
    }

    //Boresight is body2sensor
    StereoVision::Geometry::RigidBodyTransform<double> body2sensor(Eigen::Vector3d::Zero(),
                                                                 Eigen::Vector3d::Zero());

    if (_useOptimzedLeverArm) {

        if (leverArm->optRot().isSet()) {
            body2sensor.r[0] = leverArm->optRot().value(0);
            body2sensor.r[1] = leverArm->optRot().value(1);
            body2sensor.r[2] = leverArm->optRot().value(2);
        }

        if (leverArm->optPos().isSet()) {
            body2sensor.t[0] = leverArm->optPos().value(0);
            body2sensor.t[1] = leverArm->optPos().value(1);
            body2sensor.t[2] = leverArm->optPos().value(2);
        }

    } else {

        if (leverArm->xRot().isSet() and leverArm->yRot().isSet() and leverArm->zRot().isSet()) {
            body2sensor.r[0] = leverArm->xRot().value();
            body2sensor.r[1] = leverArm->yRot().value();
            body2sensor.r[2] = leverArm->zRot().value();
        }

        if (leverArm->xCoord().isSet() and leverArm->yCoord().isSet() and leverArm->zCoord().isSet()) {
            body2sensor.t[0] = leverArm->xCoord().value();
            body2sensor.t[1] = leverArm->yCoord().value();
            body2sensor.t[2] = leverArm->zCoord().value();
        }
    }

    StereoVision::Geometry::RigidBodyTransform<double> sensor2body = body2sensor.inverse();

    //#pragma omp parallel for
    for (int i = 0; i < nLines; i++) {

    //#ifndef NDEBUG
        out << "\r\t" << "Treating line " << (i+1) << "/" << nLines;
        out.flush();
    //#endif

        double targetTime = times[i];

        Trajectory<double>::TimeInterpolableVals vals = _bil_trajectory.getValueAtTime(targetTime);
        StereoVision::Geometry::RigidBodyTransform<double> delta = vals.valLower.inverse()*vals.valUpper;

        double w = vals.weigthUpper;
        StereoVision::Geometry::RigidBodyTransform<double> body2ecef = vals.valLower*(w*delta);

        StereoVision::Geometry::RigidBodyTransform<double> sensor2ecef = body2ecef*sensor2body;

        std::array<double, 3> ecefOrigin{sensor2ecef.t.x(), sensor2ecef.t.y(), sensor2ecef.t.z()};

        std::vector<std::array<float, 3>> viewDirectionsECEF(_nSamples);

        for (int i = 0; i < _nSamples; i++) {
            Eigen::Vector3d vec(viewDirectionsSensor[i][0], viewDirectionsSensor[i][1], viewDirectionsSensor[i][2]);
            Eigen::Vector3d transformed = StereoVision::Geometry::angleAxisRotate(sensor2ecef.r, vec);

            viewDirectionsECEF[i] = std::array<float, 3>{float(transformed.x()), float(transformed.y()), float(transformed.z())};
        }

        auto projResOpt = _terrain_projector->projectVectors(ecefOrigin, viewDirectionsECEF);

        Multidim::Array<float,2> projectedCoordinates({_nSamples,2}, {2,1});

        for (int i = 0; i < _nSamples; i++) {
            projectedCoordinates.atUnchecked(i,0) = std::nanf("");
            projectedCoordinates.atUnchecked(i,1) = std::nanf("");
        }

        if (projResOpt.has_value()) {

            StereoVisionApp::Geo::TerrainProjector<double>::ProjectionResults& results = projResOpt.value();

            int nToTreat = std::min<int>(results.projectedPoints.size(), _nSamples);

            for (int i = 0; i < nToTreat; i++) {
                std::array<float,2> proj = results.projectedPoints[i];

                if (std::isfinite(proj[0]) or std::isfinite(proj[1])) {

                    if (proj[0] < min_x) {
                        min_x = proj[0];
                    }

                    if (proj[0] > max_x) {
                        max_x = proj[0];
                    }

                    if (proj[1] < min_y) {
                        min_y = proj[1];
                    }

                    if (proj[1] > max_y) {
                        max_y = proj[1];
                    }

                }

                projectedCoordinates.atUnchecked(i,0) = proj[0];
                projectedCoordinates.atUnchecked(i,1) = proj[1];
            }
        }

        tmpProjections.write(reinterpret_cast<char*>(&projectedCoordinates.atUnchecked(0,0)),
                             projectedCoordinates.flatLenght()*sizeof (float));

    }
    //#ifndef NDEBUG
    out << Qt::endl;
    //#endif

    out << "\t" << "min_x = " << min_x << " max_x = " << max_x << " min_y = " << min_y << " max_y = " << max_y << Qt::endl;

    if (_mode == ExportOrtho) {
        BilROI regionOfInterest;
        regionOfInterest.minX = min_x;
        regionOfInterest.maxX = max_x;
        regionOfInterest.minY = min_y;
        regionOfInterest.maxY = max_y;
        _bilFilesROI[bilId] = regionOfInterest;
    }

    return true;

}

bool RectifyBilSeqToOrthoSteppedProcess::exportBilProjection(int bilId, QString exportPath, QString exportCRS) {

    constexpr int enviDataTypeDouble = 5;

    QTextStream out(stdout);

    out << "Export bil projection for bil: " << bilId << Qt::endl;

    QFileInfo outPathInfos(exportPath);

    if (!outPathInfos.isDir()) {
        out << "Trying to export to a non directory: " << exportPath << ", aborting!" << Qt::endl;
        return false;
    }

    QString bil_file_path = _bilSequence->getBilInfos()[bilId].bilFilePath();

    QFileInfo bil_file_info(bil_file_path);

    if (!bil_file_info.exists()) {
        out << "File for bil id: " << bilId << " (" << bil_file_path << ") does not exist" << Qt::endl;
        return false;
    }

    QString tmpFileName = QString("bilProj%1.bin").arg(bilId);

    QString tmpFilePath = _tmp_folder->filePath(tmpFileName);

    QFile tmpProjections(tmpFilePath);

    bool ok = tmpProjections.open(QFile::ReadOnly);

    if (!ok) {
        out << "Could not open bil id: " << bilId << " projection data in file " << tmpFilePath << Qt::endl;
        return false;
    }

    StereoVisionApp::Mounting* leverArm = _bilSequence->getAssignedMounting();

    if (leverArm == nullptr) {
        return false;
    }

    //Boresight is body2sensor
    StereoVision::Geometry::RigidBodyTransform<double> body2sensor(Eigen::Vector3d::Zero(),
                                                                 Eigen::Vector3d::Zero());

    if (_useOptimzedLeverArm) {

        if (leverArm->optRot().isSet()) {
            body2sensor.r[0] = leverArm->optRot().value(0);
            body2sensor.r[1] = leverArm->optRot().value(1);
            body2sensor.r[2] = leverArm->optRot().value(2);
        }

        if (leverArm->optPos().isSet()) {
            body2sensor.t[0] = leverArm->optPos().value(0);
            body2sensor.t[1] = leverArm->optPos().value(1);
            body2sensor.t[2] = leverArm->optPos().value(2);
        }

    } else {

        if (leverArm->xRot().isSet() and leverArm->yRot().isSet() and leverArm->zRot().isSet()) {
            body2sensor.r[0] = leverArm->xRot().value();
            body2sensor.r[1] = leverArm->yRot().value();
            body2sensor.r[2] = leverArm->zRot().value();
        }

        if (leverArm->xCoord().isSet() and leverArm->yCoord().isSet() and leverArm->zCoord().isSet()) {
            body2sensor.t[0] = leverArm->xCoord().value();
            body2sensor.t[1] = leverArm->yCoord().value();
            body2sensor.t[2] = leverArm->zCoord().value();
        }
    }

    StereoVision::Geometry::RigidBodyTransform<double> sensor2body = body2sensor.inverse();


    PJ_CONTEXT* ctx = proj_context_create();

    if (ctx == 0) {
        return false;
    }


    const char* wgs84_ecef = "EPSG:4978";
    const char* wgs84_geo = "EPSG:4979";


    PJ* ecef2terrain = proj_create_crs_to_crs(ctx, wgs84_ecef, _terrain.crsInfos.c_str(), nullptr);

    if (ecef2terrain == 0) { //in case of error
        proj_context_destroy(ctx);
        return false;
    }


    PJ* geo2ecef = proj_create_crs_to_crs(ctx, wgs84_geo, wgs84_ecef, nullptr);

    if (geo2ecef == 0) { //in case of error
        proj_destroy(ecef2terrain);
        proj_context_destroy(ctx);
        return false;
    }


    PJ* ecef2geo = proj_create_crs_to_crs(ctx, wgs84_ecef, wgs84_geo, nullptr);

    if (ecef2geo == 0) { //in case of error
        proj_destroy(ecef2terrain);
        proj_destroy(geo2ecef);
        proj_context_destroy(ctx);
        return false;
    }

    std::vector<std::array<double, 3>> viewDirectionsSensor = _bilSequence->getSensorViewDirections(_useOptimzedCamera);

    Eigen::Matrix<double,3,3> dsmToMap;
    dsmToMap.block<2,3>(0,0) = _terrain.geoTransform;
    dsmToMap(2,0) = 0;
    dsmToMap(2,1) = 0;
    dsmToMap(2,2) = 1;

    std::vector<double> times = get_envi_bil_lines_times(bil_file_path.toStdString());

    int nLines = times.size();

    Multidim::Array<float,2> projectedCoordinates({_nSamples,2}, {2,1});

    Multidim::Array<double,3> data({_nSamples, nLines, 3},{nLines,1,_nSamples*nLines});
    Multidim::Array<int32_t,3> angles({_nSamples, nLines, 3},{nLines,1,_nSamples*nLines});

    double avgHeight = 0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < nLines; i++) { //lines

        double targetTime = times[i];

        Trajectory<double>::TimeInterpolableVals vals = _bil_trajectory.getValueAtTime(targetTime);
        StereoVision::Geometry::RigidBodyTransform<double> delta = vals.valLower.inverse()*vals.valUpper;

        double w = vals.weigthUpper;
        StereoVision::Geometry::RigidBodyTransform<double> body2ecef = vals.valLower*(w*delta);

        StereoVision::Geometry::RigidBodyTransform<double> sensor2ecef = body2ecef*sensor2body;

        std::array<double, 3> ecefOrigin{sensor2ecef.t.x(), sensor2ecef.t.y(), sensor2ecef.t.z()};

        PJ_COORD ecef{ecefOrigin[0], ecefOrigin[1], ecefOrigin[2]};
        PJ_COORD geo = proj_trans(ecef2geo, PJ_FWD, ecef);
        PJ_COORD down = geo;
        PJ_COORD north = geo;

        down.xyz.z -= 10; //10 meters down;
        north.xyz.x += 1./3600; //one second higher north

        PJ_COORD downEcef = proj_trans(geo2ecef, PJ_FWD, down);
        PJ_COORD northEcef = proj_trans(geo2ecef, PJ_FWD, north);

        PJ_COORD originInTerrain = proj_trans(ecef2terrain, PJ_FWD, ecef);

        Eigen::Vector3d vOrigin(ecefOrigin[0],ecefOrigin[1],ecefOrigin[2]);

        Eigen::Vector3d vDown(downEcef.xyz.x, downEcef.xyz.y,downEcef.xyz.z);
        Eigen::Vector3d vNorth(northEcef.xyz.x, northEcef.xyz.y,northEcef.xyz.z);

        vDown -= vOrigin;
        vNorth -= vOrigin;

        vDown.normalize();
        vNorth.normalize();

        std::vector<std::array<float, 3>> viewDirectionsECEF(_nSamples);

        for (int i = 0; i < _nSamples; i++) {
            Eigen::Vector3d vec(viewDirectionsSensor[i][0], viewDirectionsSensor[i][1], viewDirectionsSensor[i][2]);
            Eigen::Vector3d transformed = StereoVision::Geometry::angleAxisRotate(sensor2ecef.r, vec);

            viewDirectionsECEF[i] = std::array<float, 3>{float(transformed.x()), float(transformed.y()), float(transformed.z())};
        }

        qint64 read = tmpProjections.read(reinterpret_cast<char*>(&projectedCoordinates.atUnchecked(0,0)),
                                          projectedCoordinates.flatLenght()*sizeof (float));

        if (read != projectedCoordinates.flatLenght()*sizeof (float)) {
            out << "Projection read error" << Qt::endl;
            return false;
        }

        for (int pt = 0; pt < _nSamples; pt++) {

            float dsm_i = projectedCoordinates.valueUnchecked(pt,0);
            float dsm_j = projectedCoordinates.valueUnchecked(pt,1);

            Eigen::Vector3d dmsCoord(dsm_j, dsm_i,1);
            Eigen::Vector3d mapCoord = dsmToMap*dmsCoord;

            constexpr auto Kernel = StereoVision::Interpolation::pyramidFunction<double,2>;
            constexpr int radius = 1;

            double z = StereoVision::Interpolation::interpolateValue<2,double, Kernel, radius>
                    (_terrain.raster, {dsm_i, dsm_j});

            data.atUnchecked(pt,i,0) = mapCoord.x();
            data.atUnchecked(pt,i,1) = mapCoord.y();
            data.atUnchecked(pt,i,2) = z;
            avgHeight += z;

            min_x = std::min(mapCoord.x(), min_x);
            max_x = std::max(mapCoord.x(), max_x);
            min_y = std::min(mapCoord.y(), min_y);
            max_y = std::max(mapCoord.y(), max_y);
            min_z = std::min(z, min_z);
            max_z = std::max(z, max_z);

            double planeHeight = originInTerrain.xyz.z - z;

            Eigen::Vector3d viewDir(viewDirectionsECEF[i][0],viewDirectionsECEF[i][1],viewDirectionsECEF[i][2]);
            viewDir.normalize();
            Eigen::Vector3d horizontal = viewDir - viewDir.dot(vDown)*vDown;

            double dot = horizontal.dot(vNorth);
            double det = horizontal.cross(vNorth).dot(-vDown);

            double zenith = std::acos(viewDir.dot(vDown));
            double heading = std::atan2(det, dot);

            if (heading < 0) {
                heading += 2*M_PI;
            }

            angles.atUnchecked(pt,i,0) = std::round(zenith/M_PI *180*100);
            angles.atUnchecked(pt,i,1) = std::round(heading/M_PI *180*10);
            angles.atUnchecked(pt,i,2) = std::round(planeHeight);


        }
    }
    out << "\tComputed the data in dsm frame!" << Qt::endl;

    QString outCrs = QString::fromStdString(_terrain.crsInfos);


    proj_destroy(ecef2terrain);
    proj_destroy(geo2ecef);
    proj_destroy(ecef2geo);
    proj_context_destroy(ctx);

    avgHeight /= (nLines*_nSamples);

    QString headerFileName = QString("bilProj%1.hdr").arg(bilId);
    QString bsqFileName = QString("bilProj%1.bsq").arg(bilId);

    QString anglesHeaderFileName = QString("bilProj%1_sca.hdr").arg(bilId);
    QString anglesBsqFileName = QString("bilProj%1_sca.bsq").arg(bilId);

    QDir outDir(exportPath);

    QString headerFilePath = outDir.filePath(headerFileName);
    QString bsqFilePath = outDir.filePath(bsqFileName);

    QString anglesHeaderFilePath = outDir.filePath(anglesHeaderFileName);
    QString anglesBsqFilePath = outDir.filePath(anglesBsqFileName);

    QFile headerFile(headerFilePath);

    ok = headerFile.open(QFile::WriteOnly|QFile::Text);

    if (!ok) {
        out << "Could not write header file: " << headerFilePath << Qt::endl;
        return false;
    }

    QTextStream hdrOut(&headerFile);

    hdrOut << "ENVI" << "\n";
    hdrOut << "description = {Steviapp image projection geometry file, average scene elevation = " << QString("%1").arg(avgHeight, 0, 'f', 2) << "}" << "\n";
    hdrOut << "samples = " << _nSamples << "\n";
    hdrOut << "lines   = " << nLines << "\n";
    hdrOut << "bands   = 3" << "\n";
    hdrOut << "header offset = 0" << "\n";
    hdrOut << "file type = ENVI Standard" << "\n";
    hdrOut << "data type = 5" << "\n";
    hdrOut << "interleave = BSQ" << "\n";
    hdrOut << "sensor type = " << "\n";
    hdrOut << "byte order = 0" << "\n";
    hdrOut << "x start =  1" << "\n";
    hdrOut << "y start =  1" << "\n";
    hdrOut << "band names = {IGM X Map, IGM Y Map, IGM Z Map}" << "\n";

    hdrOut << "xrange = {" << min_x << ", " << max_x << "}" << "\n";
    hdrOut << "yrange = {" << min_y << ", " << max_y << "}" << "\n";
    hdrOut << "zrange = {" << min_z << ", " << max_z << "}" << "\n";
    hdrOut << "coordinate_system = " << outCrs << Qt::flush;

    QFile anglesHeaderFile(anglesHeaderFilePath);

    ok = anglesHeaderFile.open(QFile::WriteOnly|QFile::Text);

    if (!ok) {
        out << "Could not write header file: " << anglesHeaderFilePath << Qt::endl;
        return false;
    }

    QTextStream anglesHdrOut(&anglesHeaderFile);

    anglesHdrOut << "ENVI" << "\n";
    anglesHdrOut << "description = {Steviapp image projection viewpoint file}" << "\n";
    anglesHdrOut << "samples = " << _nSamples << "\n";
    anglesHdrOut << "lines   = " << nLines << "\n";
    anglesHdrOut << "bands   = 3" << "\n";
    anglesHdrOut << "header offset = 0" << "\n";
    anglesHdrOut << "file type = ENVI Standard" << "\n";
    anglesHdrOut << "data type = 3" << "\n";
    anglesHdrOut << "interleave = BSQ" << "\n";
    anglesHdrOut << "sensor type = " << "\n";
    anglesHdrOut << "byte order = 0" << "\n";
    anglesHdrOut << "band names = {Zenithx100, Azimuthx10, HeightM}" << Qt::flush;


    QFile bsqFile(bsqFilePath);

    ok = bsqFile.open(QFile::WriteOnly);

    if (!ok) {
        out << "Could not write bsq file: " << bsqFilePath << Qt::endl;
        return false;
    }

    bsqFile.write(reinterpret_cast<char*>(&data.atUnchecked(0,0,0)),
                         data.flatLenght()*sizeof (double));


    QFile anglesBsqFile(anglesBsqFilePath);

    ok = anglesBsqFile.open(QFile::WriteOnly);

    if (!ok) {
        out << "Could not write bsq file: " << anglesBsqFilePath << Qt::endl;
        return false;
    }

    anglesBsqFile.write(reinterpret_cast<char*>(&angles.atUnchecked(0,0,0)),
                         angles.flatLenght()*sizeof (double));


    return true;
}
bool RectifyBilSeqToOrthoSteppedProcess::computeProjectedGrid() {

    QTextStream out(stdout);

    out << "Computing project grid:\n";

    _exportGrid.clear();

    //Note that local image coordinate system has x as vertical axis.
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();

    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < _bilFilesROI.size(); i++) {

        if (_bilFilesROI[i].minX < min_x) {
            min_x = _bilFilesROI[i].minX;
        }

        if (_bilFilesROI[i].maxX > max_x) {
            max_x = _bilFilesROI[i].maxX;
        }

        if (_bilFilesROI[i].minY < min_y) {
            min_y = _bilFilesROI[i].minY;
        }

        if (_bilFilesROI[i].maxY > max_y) {
            max_y = _bilFilesROI[i].maxY;
        }

    }

    out << "\tmin_x = " << min_x << " max_x = " << max_x << " min_y = " << min_y << " max_y = " << max_y << Qt::endl;
    out << "\t" << "Target gsd = " << _target_gsd <<
           " Map scale = " << _map_scale <<
           " Max tile width = " << _max_tile_width << Qt::endl;

    //maximal side lenghts of a tile, in terrain pixels
    double max_len_x = _max_tile_width/_map_scale;
    double max_len_y = _max_tile_width/_map_scale;

    double area_x = max_x - min_x;
    double area_y = max_y - min_y;

    int nCutx = std::ceil(area_x/max_len_x);
    int nCuty = std::ceil(area_y/max_len_y);

    double delta_x = area_x/nCutx;
    double delta_y = area_y/nCuty;

    double len_x = _map_scale*delta_x;
    double len_y = _map_scale*delta_y;

    out << "\tmax_len_x = " << max_len_x << " max_len_y = " << max_len_y << Qt::endl;
    out << "\tarea_height = " << area_x << " area_width = " << area_y << Qt::endl;
    out << "\tnCutHorz = " << nCuty << " nCutVert = " << nCutx << Qt::endl;

    for (int i = 0; i < nCutx; i++) {
        for (int j = 0; j < nCuty; j++) {

            bool anyIntersect = false;

            for (BilROI const& roi : _bilFilesROI) {
                if ((roi.maxX > min_x + i*delta_x and
                        roi.maxY > min_y + j*delta_y)
                        and
                        (roi.minX < min_x + (i+1)*delta_x) and
                         roi.minY < min_y + (j+1)*delta_y) {
                    anyIntersect = true;
                    break;
                }
            }

            if (!anyIntersect) {
                continue;
            }

            GridBlock tile;

            tile.pi = i;
            tile.pj = j;

            tile.height = std::ceil(len_x);
            tile.width = std::ceil(len_y);

            tile.i0 = min_x + i*delta_x;
            tile.j0 = min_y + j*delta_y;

            tile.in = tile.i0 + delta_x;
            tile.jn = tile.j0 + delta_y;

            _exportGrid.push_back(tile);

        }
    }

    for (GridBlock const& block : _exportGrid) {
        out << "\t" << block.i0 << " " << block.j0 << " -> " << block.in << " " << block.jn << " (" << block.pi << " " << block.pj << " " << block.height << " " << block.width << ")\n";
    }

    return true;

}
bool RectifyBilSeqToOrthoSteppedProcess::computeNextTile(int tileId) {

    QTextStream out(stdout);

    GridBlock const& tile =  _exportGrid[tileId];

    out << "\tExporting tile: " << tile.i0 << " " << tile.j0 << " -> " << tile.in << " " << tile.jn << " (" << tile.pi << " " << tile.pj << " " << tile.height << " " << tile.width << ")" << Qt::endl;

    Eigen::Matrix<double,3,3> dsmToMap;
    dsmToMap.block<2,3>(0,0) = _terrain.geoTransform;
    dsmToMap(2,0) = 0;
    dsmToMap(2,1) = 0;
    dsmToMap(2,2) = 1;

    Eigen::Matrix<double,3,3> tileTodsm = Eigen::Matrix<double,3,3>::Identity();
    tileTodsm(0,0) = 1/_map_scale; //scale of one tile pixel in dsm space
    tileTodsm(1,1) = 1/_map_scale; //scale of one tile pixel in dsm space
    tileTodsm(0,2) = tile.j0;
    tileTodsm(1,2) = tile.i0;

    Eigen::Matrix<double,3,3> dsmToTile = tileTodsm.inverse();

    Eigen::Matrix<double,3,3> tileToMap = dsmToMap*tileTodsm;

    Eigen::Matrix<double,2,3> modifiedGeoTransform = tileToMap.block<2,3>(0,0);

    Multidim::Array<float,2> nSamples(tile.height, tile.width);
    Multidim::Array<float,3> samples(tile.height, tile.width, _bands);
    Multidim::Array<float, 3> preview(tile.height, tile.width, 3);

    #pragma omp parallel for
    for (int i = 0; i < samples.shape()[0]; i++) {
        for (int j = 0; j < samples.shape()[1]; j++) {

            for (int b = 0; b < samples.shape()[2]; b++) {
                samples.atUnchecked(i,j,b) = 0;
            }

            nSamples.atUnchecked(i,j) = 0;
        }
    }

    out << "\t" << "Memory reserved!" << Qt::endl;

    bool anyWritten = false;

    for (int i = 0; i < _bilFilesROI.size(); i++) {

        int bilId = i;

        out << "\t\t" << "Considering bil " << bilId << Qt::endl;

        QString tmpFileName = QString("bilProj%1.bin").arg(bilId);

        QString tmpFilePath = _tmp_folder->filePath(tmpFileName);

        QFile tmpProjections(tmpFilePath);

        bool ok = tmpProjections.open(QFile::ReadOnly);

        if (!ok) {
            return false;
        }

        out << "\t\t" << "Reading bil " << _bilPaths[i] << Qt::endl;

        Multidim::Array<float, 3> bilFile = read_envi_bil_to_float(_bilPaths[i].toStdString());

        out << "\t\t" << "Read bil " << _bilPaths[i] << " Shape: " << bilFile.shape()[0] << " " << bilFile.shape()[1] << " " << bilFile.shape()[2] << Qt::endl;

        if (bilFile.empty()) {
            continue;
        }

        Multidim::Array<float,2> projectedCoordinates({_nSamples,2}, {2,1});

        for (int i = 0; i < bilFile.shape()[0]; i++) { //lines

            qint64 read = tmpProjections.read(reinterpret_cast<char*>(&projectedCoordinates.atUnchecked(0,0)),
                                              projectedCoordinates.flatLenght()*sizeof (float));

            if (read != projectedCoordinates.flatLenght()*sizeof (float)) {
                out << "Projection read error" << Qt::endl;
                continue;
            }

            out << "\r\t\t" << "Read bil coordinates projections for line " << i << "  in " << tmpFilePath << "!" << Qt::flush;

            for (int j = 0; j < bilFile.shape()[1]; j++) { //samples

                float dsm_i = projectedCoordinates.valueUnchecked(j,0);
                float dsm_j = projectedCoordinates.valueUnchecked(j,1);

                Eigen::Vector3d dmsCoord(dsm_j, dsm_i,1);
                Eigen::Vector3d tileCoord = dsmToTile*dmsCoord;

                //pixel coordinate of geotiff still have x horizontal
                float tile_i = tileCoord.y();
                float tile_j = tileCoord.x();

                int pIfloor = std::floor(tile_i);
                int pIceil = std::ceil(tile_i);

                int pJfloor = std::floor(tile_j);
                int pJceil = std::ceil(tile_j);

                float wI = pIceil - tile_i;
                float wJ = pJceil - tile_j;

                if (pIfloor >= samples.shape()[0] or pIceil < 0) {
                    continue;
                }

                if (pJfloor >= samples.shape()[1] or pJceil < 0) {
                    continue;
                }

                if (pIceil >= samples.shape()[0]) {
                    pIceil = pIfloor;
                    wI = 1;
                }

                if (pIfloor < 0) {
                    pIfloor = pIceil;
                    wI = 1;
                }

                if (pJceil >= samples.shape()[1]) {
                    pJceil = pJfloor;
                    wJ = 1;
                }

                if (pJfloor < 0) {
                    pJfloor = pJceil;
                    wJ = 1;
                }

                #pragma omp parallel for
                for (int c = 0; c < bilFile.shape()[2]; c++) { //paralelize here to avoid race conditions.

                    float val = bilFile.valueUnchecked(i,j,c);

                    samples.atUnchecked(pIfloor, pJfloor, c) += wI*wJ*val;
                    samples.atUnchecked(pIfloor, pJceil, c) += wI*(1-wJ)*val;
                    samples.atUnchecked(pIceil, pJfloor, c) += (1-wI)*wJ*val;
                    samples.atUnchecked(pIceil, pJceil, c) += (1-wI)*(1-wJ)*val;
                }

                nSamples.atUnchecked(pIfloor, pJfloor) += wI*wJ;
                nSamples.atUnchecked(pIfloor, pJceil) += wI*(1-wJ);
                nSamples.atUnchecked(pIceil, pJfloor) += (1-wI)*wJ;
                nSamples.atUnchecked(pIceil, pJceil) += (1-wI)*(1-wJ);

                anyWritten = true;


            }
        }

    }

    out << "\n";

    if (!anyWritten) {
        out << "\tNo pixels written for tile" << Qt::endl;
        return true;
    } else {
        out << "\t" << "Finished integrating the file" << Qt::endl;
    }

    for (int i = 0; i < samples.shape()[0]; i++) {
        for (int j = 0; j < samples.shape()[1]; j++) {

            float weight = nSamples.valueUnchecked(i,j);

            if (weight <= 0) {
                continue;
            }

            for (int c = 0; c < samples.shape()[2]; c++) {
                samples.atUnchecked(i,j,c) /= weight;
            }

        }
    }
    out << "\t" << "Finished computing the file" << Qt::endl;

    int redChannel = _redChannel;
    int greenChannel = _greenChannel;
    int blueChannel = _blueChannel;

    if (redChannel >= samples.shape()[2]) {
        redChannel = 0;
    }

    if (greenChannel >= samples.shape()[2]) {
        greenChannel = samples.shape()[2]/2;
    }

    if (blueChannel >= samples.shape()[2]) {
        blueChannel = samples.shape()[2]-1;
    }

    if (_inPaintingRadius > 0) {

        int iPRadius = _inPaintingRadius;

        out << "\t" << "Start inpainting" << Qt::endl;

        using AxisT = StereoVision::ImageProcessing::Convolution::MovingWindowAxis;
        using PaddingT = StereoVision::ImageProcessing::Convolution::PaddingInfos;

        auto filter = StereoVision::ImageProcessing::Convolution::constantFilter<float>
                (1, iPRadius, AxisT(PaddingT(iPRadius)), AxisT(PaddingT(iPRadius)));
        filter.setPaddingConstant(1);

        Multidim::Array<float,2> extended = filter.convolve(nSamples);
        Multidim::Array<bool,2> toPaint(nSamples.shape());
        Multidim::Array<bool,2> pixels(nSamples.shape());

        for (int i = 0; i < nSamples.shape()[0]; i++) {
            for (int j = 0; j < nSamples.shape()[1]; j++) {

                pixels.atUnchecked(i,j) = nSamples.valueUnchecked(i,j) > 0;

                if (nSamples.valueUnchecked(i,j) <= 0 and extended.valueUnchecked(i,j) > 0) {
                    toPaint.atUnchecked(i,j) = true;
                } else {
                    toPaint.atUnchecked(i,j) = false;
                }
            }
        }

        Multidim::Array<bool,2> filteredPixels = StereoVision::ImageProcessing::erosion(2*iPRadius+1,2*iPRadius+1,
                                                                                        StereoVision::ImageProcessing::dilation(2*iPRadius, 2*iPRadius,
                                                                                                                                pixels));

        samples = StereoVision::ImageProcessing::nearestInPaintingBatched<float,3,1>(samples, toPaint, {2});

        for (int i = 0; i < nSamples.shape()[0]; i++) {
            for (int j = 0; j < nSamples.shape()[1]; j++) {

                if (!toPaint.valueUnchecked(i,j) or filteredPixels.valueUnchecked(i,j)) {
                    continue;
                }

                for (int c = 0; c < samples.shape()[2]; c++) {
                    samples.atUnchecked(i,j,c) = 0;
                }
            }
        }

        out << "\t" << "End inpainting" << Qt::endl;
    }

    out << "\t" << "Start computing preview image with channels r = " << redChannel << " g = " << greenChannel << " b = " << blueChannel << Qt::endl;

    float maxPreviewRed = 0;
    float maxPreviewGreen = 0;
    float maxPreviewBlue = 0;

    for (int i = 0; i < samples.shape()[0]; i++) {
        for (int j = 0; j < samples.shape()[1]; j++) {

            float red = samples.atUnchecked(i,j,redChannel);
            float green = samples.atUnchecked(i,j,greenChannel);
            float blue = samples.atUnchecked(i,j,blueChannel);

            maxPreviewRed = std::max(maxPreviewRed, red);
            maxPreviewGreen = std::max(maxPreviewGreen, green);
            maxPreviewBlue = std::max(maxPreviewBlue, blue);

            preview.atUnchecked(i,j,0) = red;
            preview.atUnchecked(i,j,1) = green;
            preview.atUnchecked(i,j,2) = blue;
        }
    }

    float scalingRed = 1/maxPreviewRed;
    float scalingGreen = 1/maxPreviewGreen;
    float scalingBlue = 1/maxPreviewBlue;

    for (int i = 0; i < samples.shape()[0]; i++) {
        for (int j = 0; j < samples.shape()[1]; j++) {

            preview.atUnchecked(i,j,0) *= scalingRed;
            preview.atUnchecked(i,j,1) *= scalingGreen;
            preview.atUnchecked(i,j,2) *= scalingBlue;

            preview.atUnchecked(i,j,0) = std::pow(preview.atUnchecked(i,j,0),1/2.2);
            preview.atUnchecked(i,j,1) = std::pow(preview.atUnchecked(i,j,1),1/2.2);
            preview.atUnchecked(i,j,2) = std::pow(preview.atUnchecked(i,j,2),1/2.2);

            preview.atUnchecked(i,j,0) *= 255;
            preview.atUnchecked(i,j,1) *= 255;
            preview.atUnchecked(i,j,2) *= 255;
        }
    }

    out << "\t" << "Finished computing the preview" << Qt::endl;

    out << "\t" << "Start writing the data" << Qt::endl;

    std::stringstream streamName;
    streamName << "_tile_" << tile.pi << "_" << tile.pj << ".tiff";

    std::stringstream streamWld;
    streamWld << "_tile_" << tile.pi << "_" << tile.pj << ".wld";

    std::stringstream streamPreview;
    streamPreview << "_tile_" << tile.pi << "_" << tile.pj << ".jpg";

    bool ok = StereoVision::IO::writeImage<float>(_outFile.toStdString() + streamName.str(), samples);
    if (ok) {
        out << "\t" << "Written" << Qt::endl;
    } else {
        out << "\t" << "Failed to write" << Qt::endl;
    }

    ok = StereoVision::IO::writeImage<uint8_t>(_outFile.toStdString() + streamPreview.str(), preview);
    if (ok) {
        out << "\t" << "Preview file Written" << Qt::endl;
    } else {
        out << "\t" << "Failed to write preview" << Qt::endl;
    }

    QFile wldFile(QString::fromStdString(_outFile.toStdString() + streamWld.str()));
    if (wldFile.open(QFile::WriteOnly)) {

        QTextStream wldOut(&wldFile);
        wldOut.setRealNumberPrecision(16);
        wldOut.setRealNumberNotation(QTextStream::FixedNotation);
        wldOut << modifiedGeoTransform(1,0) << "\n";
        wldOut << modifiedGeoTransform(1,1) << "\n";
        wldOut << modifiedGeoTransform(0,0) << "\n";
        wldOut << modifiedGeoTransform(0,1) << "\n";
        wldOut << modifiedGeoTransform(1,2) << "\n";
        wldOut << modifiedGeoTransform(0,2);

        wldOut.flush();

        wldFile.close();
        out << "\t" << "Wld file Written" << Qt::endl;

    } else {
        out << "\t" << "Failed to write wld file" << Qt::endl;
    }

    return true;

}

bool RectifyBilSeqToOrthoSteppedProcess::init() {

    if (_bilSequence == nullptr) {
        return false;
    }

    StereoVisionApp::PushBroomPinholeCamera* cam = _bilSequence->getAssignedCamera();

    if (cam == nullptr) {
        return false;
    }

    StereoVisionApp::Project* project = _bilSequence->getProject();

    if (project == nullptr) {
        return false;
    }

    qint64 trajId = _bilSequence->assignedTrajectory();

    StereoVisionApp::Trajectory* trajectory = project->getDataBlock<StereoVisionApp::Trajectory>(trajId);

    if (trajectory == nullptr) {
        return false;
    }

    //setup temporary directory
    _tmp_folder = new QTemporaryDir();

    //load dtm data
    auto dtm = readGeoRasterData<double, 2>(_inputDtm->getDataSource().toStdString());

    if (!dtm.has_value()) {
        return false;
    }

    _terrain = dtm.value();

    Multidim::Array<double,2> corner_vecs({3, 3}, {3,1});

    for (int c = 0; c < 3; c++) {

        int i = 0;
        int j = 0;

        if (c == 1) {
            i = _terrain.raster.shape()[0]-1;
        }

        if (c == 2) {
            j = _terrain.raster.shape()[1]-1;
        }

        Eigen::Vector3d homogeneousImgCoord(j,i,1);
        Eigen::Vector2d geoCoord = _terrain.geoTransform*homogeneousImgCoord;

        corner_vecs.atUnchecked(c,0) = geoCoord.x();
        corner_vecs.atUnchecked(c,1) = geoCoord.y();

        corner_vecs.atUnchecked(c,2) = 0;
    }


    PJ_CONTEXT* ctx = proj_context_create();


    const char* wgs84_ecef = "EPSG:4978";


    PJ* reprojector = proj_create_crs_to_crs(ctx, _terrain.crsInfos.c_str(), wgs84_ecef, nullptr);

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

    Eigen::Vector3d origin;
    origin << corner_vecs.valueUnchecked(0,0), corner_vecs.valueUnchecked(0,1), corner_vecs.valueUnchecked(0,2);

    Eigen::Vector3d local_frame_x_axis;
    local_frame_x_axis << corner_vecs.valueUnchecked(1,0), corner_vecs.valueUnchecked(1,1), corner_vecs.valueUnchecked(1,2);
    local_frame_x_axis -= origin;
    local_frame_x_axis /= _terrain.raster.shape()[0]-1;

    double terrain_vert_gsd = local_frame_x_axis.norm();


    Eigen::Vector3d local_frame_y_axis;
    local_frame_y_axis << corner_vecs.valueUnchecked(2,0), corner_vecs.valueUnchecked(2,1), corner_vecs.valueUnchecked(2,2);
    local_frame_y_axis -= origin;
    local_frame_y_axis /= _terrain.raster.shape()[1]-1;

    double terrain_horz_gsd = local_frame_y_axis.norm();

    _map_scale = std::max(terrain_horz_gsd, terrain_vert_gsd)/_target_gsd;

    if (_terrain_projector != nullptr) {
        delete _terrain_projector;
    }

    _terrain_projector = new StereoVisionApp::Geo::TerrainProjector<double>(_terrain);

    //load bill data

    if (_useOptimzedTrajectory) {

        constexpr bool resample = true;

        StereoVisionApp::StatusOptionalReturn<Trajectory<double>> trajOpt =
                trajectory->optimizedTrajectoryECEF(resample);

        if (!trajOpt.isValid()) {
            return false;
        }

        _bil_trajectory = trajOpt.value();

    } else {

        StereoVisionApp::StatusOptionalReturn<Trajectory<double>> trajOpt =
                trajectory->loadTrajectorySequence();

        if (!trajOpt.isValid()) {
            return false;
        }

        _bil_trajectory = trajOpt.value();

    }

    _bilPaths = _bilSequence->getBilFiles();

    _bilFilesROI.clear();
    _bilFilesROI.resize(_bilPaths.size());

    _processedLines = 0;

    auto header = readBilHeaderData(_bilPaths.first().toStdString());

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

    if (header->count("samples") <= 0) {
        return false;
    }

    try {
        _nSamples = std::stoi(header.value()["samples"]);
    }
    catch(std::invalid_argument const& e) {
        return false;
    }

    if (cam->optimizedFLen().isSet() and cam->optimizedOpticalCenterX().isSet()) {

        _f_len_pix = cam->optimizedFLen().value();
        _optical_center = cam->optimizedOpticalCenterX().value();

    } else {

        _f_len_pix = cam->fLen().value();
        _optical_center = cam->opticalCenterX().value();
    }

    return true;
}

void RectifyBilSeqToOrthoSteppedProcess::cleanup() {

    if (_tmp_folder != nullptr) {
        delete _tmp_folder; //clean up temporary files;
        _tmp_folder = nullptr;
    }

    if (_terrain_projector != nullptr) {
        delete _terrain_projector;
        _terrain_projector = nullptr;
    }

    _bilFilesROI.clear();
    _exportGrid.clear();
}

} // namespace PikaLTools
