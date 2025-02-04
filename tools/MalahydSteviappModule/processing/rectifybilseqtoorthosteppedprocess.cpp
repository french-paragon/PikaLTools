#include "rectifybilseqtoorthosteppedprocess.h"

#include <StereoVision/io/image_io.h>
#include <StereoVision/imageProcessing/pixelsLines.h>
#include <StereoVision/imageProcessing/convolutions.h>
#include <StereoVision/imageProcessing/standardConvolutionFilters.h>
#include <StereoVision/imageProcessing/inpainting.h>

#include <steviapp/datablocks/trajectory.h>
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
    _bilSequence(nullptr),
    _minBilLine(-1),
    _maxBilLine(-1),
    _useOptimzedTrajectory(true),
    _terrain_projector(nullptr),
    _tmp_folder(nullptr)
{
    _target_gsd = 0.5;
    _max_tile_width = 2000;

    _redChannel = 43;
    _greenChannel = 27;
    _blueChannel = 12;

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

    constexpr bool optimized = true;
    std::vector<std::array<double, 3>> viewDirectionsSensor = _bilSequence->getSensorViewDirections(optimized);

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

    if (_bilSequence->optRot().isSet()) {
        body2sensor.r[0] = _bilSequence->optRot().value(0);
        body2sensor.r[1] = _bilSequence->optRot().value(1);
        body2sensor.r[2] = _bilSequence->optRot().value(2);
    }

    if (_bilSequence->optPos().isSet()) {
        body2sensor.t[0] = _bilSequence->optPos().value(0);
        body2sensor.t[1] = _bilSequence->optPos().value(1);
        body2sensor.t[2] = _bilSequence->optPos().value(2);
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

            for (int i = 0; i < _nSamples; i++) {
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

    BilROI regionOfInterest;
    regionOfInterest.minX = min_x;
    regionOfInterest.maxX = max_x;
    regionOfInterest.minY = min_y;
    regionOfInterest.maxY = max_y;
    _bilFilesROI[bilId] = regionOfInterest;

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
        }
    }
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
