#include "libs/io/read_envi_bil.h"
#include "libs/io/georasterreader.h"

#include "libs/geo/coordinate_conversions.h"

#include <StereoVision/io/image_io.h>

#include <steviapp/geo/geoRaster.h>
#include <steviapp/geo/terrainProjector.h>
#include <steviapp/datablocks/trajectory.h>

#include <QTextStream>
#include <QVector>
#include <QRectF>
#include <QFileInfo>
#include <QDir>

#include <QJsonDocument>
#include <QJsonObject>

#include <iostream>

#include <tclap/CmdLine.h>

template<typename T>
int projectBilSequence(std::vector<std::string> const& filesList, int argc, char** argv) {

    QTextStream out(stdout);

    std::string bil_folderpath;
    std::string dtm_filepath;
    float mapScale;
    int maxTilePixels;
    bool useCached;
    std::string output_folder;
    std::string trajectoryConfig_filePath;

    try {

        TCLAP::CmdLine cmd("Project a bil sequence to a map", '=', "0.0");

        TCLAP::UnlabeledValueArg<std::string> bilFolderPathArg("bildFolderPath", "Path where the bil sequence is stored", true, "", "local path to folder");
        TCLAP::UnlabeledValueArg<std::string> dtmPathArg("dtmPath", "path to the dtm to use", true, "", "path to geotiff dtm");

        cmd.add(bilFolderPathArg);
        cmd.add(dtmPathArg);

        TCLAP::ValueArg<float> scaleArg("s","scale", "Scale at which the dtm should be used", false, 1., "float");
        TCLAP::ValueArg<int> tileArg("w","tileWidth", "Number of pixels in a tile", false, 2000, "int");
        TCLAP::SwitchArg useCacheArg("c", "useCached", "Used cached projection");
        TCLAP::ValueArg<std::string> outputPathArg("o","output", "Output path", false, "./rectified", "local path to folder");
        TCLAP::ValueArg<std::string> trajectoryConfigArg("t","trajectory", "Output path", false, "", "local path to json file of trajectory");

        cmd.add(scaleArg);
        cmd.add(tileArg);
        cmd.add(useCacheArg);
        cmd.add(outputPathArg);

        cmd.parse(argc, argv);

        bil_folderpath = bilFolderPathArg.getValue();
        dtm_filepath = dtmPathArg.getValue();

        mapScale = scaleArg.getValue();

        if (mapScale < 1) {
            out << "Invalid map scale!" << Qt::endl;
            return 1;
        }

        maxTilePixels = tileArg.getValue();

        if (maxTilePixels <= 0) {
            out << "max tile size smaller or equal 0 is invalid!" << Qt::endl;
            return 1;
        }

        useCached = useCacheArg.getValue();

        output_folder = outputPathArg.getValue();

        if (output_folder.back() != '/') {
            output_folder += '/';
        }

        trajectoryConfig_filePath = trajectoryConfigArg.getValue();

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        return 1;
    }

    QDir outputDir(QString::fromStdString(output_folder));
    if (!outputDir.exists()) {
        bool ok = outputDir.mkpath(".");

        if (!ok) {
            std::cerr << "Cannot create ouput path: " << output_folder << std::endl;
            return 1;
        }
    }

    if (filesList.empty()) {
        out << "Empty list of bil files" << Qt::endl;
        return 1;
    }

    auto terrainOpt = PikaLTools::readGeoRasterData<double, 2>(dtm_filepath);

    if (!terrainOpt.has_value()) {
        out << "Could not read terrain file \"" << QString::fromStdString(dtm_filepath) << "\"" << Qt::endl;
        return 1;
    }

    StereoVisionApp::Geo::GeoRasterData<double, 2>& terrain = terrainOpt.value();

    StereoVisionApp::Geo::TerrainProjector<double> projector(terrain);

    std::optional<Trajectory<double>> trajOpt = std::nullopt;
    QFileInfo trajConfigFileInfos(QString::fromStdString(trajectoryConfig_filePath));

    if (trajConfigFileInfos.exists()) {
        QFile trajConfig(QString::fromStdString(trajectoryConfig_filePath));
        QByteArray content = trajConfig.readAll();

        QJsonParseError parseStatus;
        QJsonDocument doc = QJsonDocument::fromJson(content, &parseStatus);

        if (parseStatus.error == QJsonParseError::NoError) {
            QJsonObject obj = doc.object();
            StereoVisionApp::Trajectory trajBlock;
            trajBlock.setParametersFromJsonRepresentation(obj);

            trajOpt = trajBlock.loadTrajectorySequence();
        }
    }

    QVector<QRectF> bilFilesROI(filesList.size()); //keep track of the regions of the terrain (in pixels coordinates) which have already been covered

    int nBands = 0;

    for (int fileId = 0; fileId < filesList.size(); fileId++) {

        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -1;

        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -1;

        std::string const& file = filesList[fileId];

        out << "Started processing \"" << QString::fromStdString(file) << "\"" << Qt::endl;

        //Sensor infos
        std::optional<std::map<std::string, std::string>> headerOpt = readHeaderData(file);

        if (!headerOpt.has_value()) {
            out << "Failed to extract header infos for file \"" << QString::fromStdString(file) << "\"" << Qt::endl;
            continue;
        }

        std::map<std::string, std::string>& header = headerOpt.value();

        if (header.count("field of view") <= 0 or
                header.count("samples") <= 0 or
                header.count("lines") <= 0 or
                header.count("framerate") <= 0 or
                header.count("bands") <= 0) {
            out << "Failed to extract field of view and samples for file \"" << QString::fromStdString(file) << "\"" << Qt::endl;
            continue;
        }

        double fieldOfViewDeg = stod(header["field of view"]);
        double fieldOfViewRad = fieldOfViewDeg/180 * M_PI;
        int nSamples = stoi(header["samples"]);
        int nLines = stoi(header["lines"]);
        double frameRate = stod(header["framerate"]);
        double frameTime = 1/frameRate;

        nBands = std::max(nBands, stoi(header["bands"])); //needs to load that data at least

        if (useCached) {
            QFileInfo infos(QString::fromStdString(file + "_projected_coords.stevimg"));

            if (infos.exists()) {

                Multidim::Array<float, 3> projectedCoordinates = StereoVision::IO::readStevimg<float, 3>(file + "_projected_coords.stevimg");

                float min_x = std::numeric_limits<float>::infinity();
                float max_x = -std::numeric_limits<float>::infinity();

                float min_y = std::numeric_limits<float>::infinity();
                float max_y = -std::numeric_limits<float>::infinity();

                for (int i = 0; i < projectedCoordinates.shape()[0]; i++) {
                    for (int j = 0; j < projectedCoordinates.shape()[1]; j++) {
                        float p_x = projectedCoordinates.valueUnchecked(i,j,0);
                        float p_y = projectedCoordinates.valueUnchecked(i,j,1);

                        if (std::isfinite(p_x) or std::isfinite(p_y)) {

                            if (p_x < min_x) {
                                min_x = p_x;
                            }

                            if (p_x > min_x) {
                                max_x = p_x;
                            }

                            if (p_y < min_y) {
                                min_y = p_y;
                            }

                            if (p_y > min_y) {
                                max_y = p_y;
                            }

                        }
                    }
                }

                bilFilesROI[fileId] = QRectF(QPointF(min_x, min_y), QPointF(max_x, max_y));

                out << "Coordinates already processed for \"" << QString::fromStdString(file) << "\"" << Qt::endl;
                continue;
            }
        }

        double midPoint = static_cast<double>(nSamples-1)/2;
        double fLenPix = static_cast<double>(nSamples-1)/2 * std::tan(M_PI_2 - fieldOfViewRad/2);

        std::vector<std::array<double, 3>> viewDirectionsSensor(nSamples);

        for (int i = 0; i < nSamples; i++) {
            viewDirectionsSensor[i] = std::array<double, 3>{i - midPoint, 0, fLenPix};
        }

        //Trajectory
        std::vector<EnviBilLcfLine> rawTrajectory = read_envi_bil_lcf_data(file);

        int nPos = rawTrajectory.size();

        Multidim::Array<float,3> projectedCoordinates(nSamples, nLines, 2);

        //Sequence of body2ecef transforms
        std::optional<Trajectory<double>> lcfTrajOpt = convertLcfSequenceToTrajectory(rawTrajectory);

        if (!lcfTrajOpt.has_value()) {
            out << "Failed to extract trajectory for file \"" << QString::fromStdString(file) << "\"" << Qt::endl;
            continue;
        }

        Trajectory<double>& lcfTrajectory = lcfTrajOpt.value();
        std::vector<double> times = get_envi_bil_lines_times(file);

        Trajectory<double>& trajectory = (trajOpt.has_value()) ? trajOpt.value() : lcfTrajOpt.value();

        int currentPoseIndex = 0;

        for (int i = 0; i < nLines; i++) {

            out << "\r\t" << "Treating line " << (i+1) << "/" << nLines;
            out.flush();

            double targetTime = times[i];

            Trajectory<double>::TimeInterpolableVals vals = trajectory.getValueAtTime(targetTime);
            StereoVision::Geometry::RigidBodyTransform<double> pose = vals.weigthLower*vals.valLower + vals.weigthUpper*vals.valUpper;

            std::array<double, 3> ecefOrigin{pose.t.x(), pose.t.y(), pose.t.z()};

            std::vector<std::array<float, 3>> viewDirectionsECEF(nSamples);

            for (int i = 0; i < nSamples; i++) {
                Eigen::Vector3d vec(viewDirectionsSensor[i][0], viewDirectionsSensor[i][1], viewDirectionsSensor[i][2]);
                Eigen::Vector3d transformed = StereoVision::Geometry::angleAxisRotate(pose.r, vec);

                viewDirectionsECEF[i] = std::array<float, 3>{float(transformed.x()), float(transformed.y()), float(transformed.z())};
            }

            auto projResOpt = projector.projectVectorsOptimized(ecefOrigin, viewDirectionsECEF);

            if (!projResOpt.has_value()) {
                continue;
            }

            StereoVisionApp::Geo::TerrainProjector<double>::ProjectionResults& results = projResOpt.value();

            for (int j = 0; j < results.projectedPoints.size(); j++) {
                std::array<float,2> proj = results.projectedPoints[j];

                if (std::isfinite(proj[0]) or std::isfinite(proj[1])) {

                    if (proj[0] < min_x) {
                        min_x = proj[0];
                    }

                    if (proj[0] > min_x) {
                        max_x = proj[0];
                    }

                    if (proj[1] < min_y) {
                        min_y = proj[1];
                    }

                    if (proj[1] > min_y) {
                        max_y = proj[1];
                    }

                }

                projectedCoordinates.atUnchecked(j,i,0) = proj[0];
                projectedCoordinates.atUnchecked(j,i,1) = proj[1];

            }

        }
        out << Qt::endl;

        bilFilesROI[fileId] = QRectF(QPointF(min_x, min_y), QPointF(max_x, max_y));

        StereoVision::IO::writeStevimg<float>(file + "_projected_coords.stevimg", projectedCoordinates);
    }

    out << "Reprojection done, starting rendering!" << Qt::endl;

    //export reprojected mosaïc
    int newGridHeight = mapScale*terrain.raster.shape()[0];
    int newGridWidth = mapScale*terrain.raster.shape()[1];

    int gridVSplits = std::ceil(float(newGridHeight)/maxTilePixels);
    int vPixsPerSplit = maxTilePixels;

    if (vPixsPerSplit*gridVSplits != newGridHeight) {
        vPixsPerSplit -= (maxTilePixels - newGridHeight%maxTilePixels)/(gridVSplits-1);
    }

    int gridHSplits = std::ceil(float(newGridWidth)/maxTilePixels);
    int hPixsPerSplit = maxTilePixels;

    if (hPixsPerSplit*gridHSplits != newGridWidth) {
        hPixsPerSplit -= (maxTilePixels - newGridWidth%maxTilePixels)/(gridHSplits-1);
    }

    for (int v = 0; v < gridVSplits; v++) {
        for (int h = 0; h < gridHSplits; h++) {

            out << "\t" << "processing tile " << (v+1) << "-" << (h+1) << " over " << gridVSplits << "-" << gridHSplits << Qt::endl;

            int i0 = v*vPixsPerSplit;
            int j0 = h*hPixsPerSplit;

            int split_height = (v == gridVSplits-1) ? newGridHeight - i0 : vPixsPerSplit;
            int split_width = (h == gridHSplits-1) ? newGridWidth - j0 : hPixsPerSplit;

            QRectF currentAOI(float(j0)/mapScale,
                              float(i0)/mapScale,
                              float(split_width)/mapScale,
                              float(split_height)/mapScale);

            Multidim::Array<float,2> nSamples(split_height, split_width);
            std::fill(&nSamples.at(0,0), &nSamples.at(split_height-1,split_width-1), 0);

            Multidim::Array<float,3> samples(split_height, split_width, nBands);
            std::fill(&samples.at(0,0,0), &samples.at(split_height-1,split_width-1, nBands-1), 0);

            bool anyWritten = false;

            for (int f = 0; f < bilFilesROI.size(); f++) {

                if (!bilFilesROI[f].intersects(currentAOI)) {
                    continue;
                }

                std::string const& file = filesList[f];

                Multidim::Array<float, 3> projectedCoordinates = StereoVision::IO::readStevimg<float, 3>(file + "_projected_coords.stevimg");
                Multidim::Array<float, 3> billFile = read_envi_bil_to_float(file);

                for (int i = 0; i < billFile.shape()[0]; i++) { //lines
                    for (int j = 0; j < billFile.shape()[1]; j++) { //samples

                        float pi = mapScale*projectedCoordinates.valueUnchecked(j,i,0) - i0;
                        float pj = mapScale*projectedCoordinates.valueUnchecked(j,i,1) - j0;

                        int pIfloor = std::floor(pi);
                        int pIceil = std::ceil(pi);

                        int pJfloor = std::floor(pj);
                        int pJceil = std::ceil(pj);

                        float wI = pIceil - pi;
                        float wJ = pJceil - pj;

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

                        for (int c = 0; c < billFile.shape()[2]; c++) {

                            float val = billFile.valueUnchecked(i,j,c);

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

            //write the projected split, if anything was written to it

            if (!anyWritten) {
                out << "\t" << "Skipped" << Qt::endl;
                continue;
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

            std::stringstream ss;
            ss << "tile_" << v << "_" << h << ".stevimg";

            bool ok = StereoVision::IO::writeStevimg<float>(output_folder + ss.str(), samples);
            if (ok) {
                out << "\t" << "Written" << Qt::endl;
            } else {
                out << "\t" << "Failed to write" << Qt::endl;
            }

        }
    }

    Multidim::Array<bool,2> const& coverMask = projector.cover();
    Multidim::Array<uint8_t,2> coverImg(coverMask.shape());

    for (int i = 0; i < coverMask.shape()[0]; i++) {
        for (int j = 0; j < coverMask.shape()[1]; j++) {

            coverImg.atUnchecked(i,j) = (coverMask.valueUnchecked(i,j)) ? 255 : 0;

        }
    }

    StereoVision::IO::writeImage<uint8_t>(output_folder + "cover_mask.bmp", coverImg);

    return 0;

}

int main(int argc, char** argv) {

    QTextStream out(stdout);

    int listPos = -1;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] != '-') {
            listPos = i;
            break;
        }
    }

    if (listPos < 1) {
        out << "Missing input bil folder argument" << Qt::endl;
        return 1;
    }

    std::string bil_folderpath(argv[listPos]);
    std::vector<std::string> fileList = get_bil_sequence_files(bil_folderpath);

    if (fileList.size() <= 0) {
        out << "Empty bil files list" << Qt::endl;
        return 1;
    }

    if (envi_bil_img_match_type<uint8_t>(fileList[0])) {
        return projectBilSequence<uint8_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<int8_t>(fileList[0])) {
        return projectBilSequence<int8_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<uint16_t>(fileList[0])) {
        return projectBilSequence<uint16_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<int16_t>(fileList[0])) {
        return projectBilSequence<int16_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<uint32_t>(fileList[0])) {
        return projectBilSequence<uint32_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<int32_t>(fileList[0])) {
        return projectBilSequence<int32_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<uint64_t>(fileList[0])) {
        return projectBilSequence<uint64_t>(fileList, argc, argv);
    }

    if (envi_bil_img_match_type<int64_t>(fileList[0])) {
        return projectBilSequence<int64_t>(fileList, argc, argv);
    }

    out << "Unsupported image type!" << Qt::endl;
    return 1;


}
