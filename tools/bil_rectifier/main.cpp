#include "libs/io/read_envi_bil.h"
#include "libs/io/georasterreader.h"

#include "libs/geo/coordinate_conversions.h"

#include <StereoVision/io/image_io.h>

#include <steviapp/geo/geoRaster.h>
#include <steviapp/geo/terrainProjector.h>

#include <QTextStream>
#include <QVector>
#include <QRectF>

QTextStream& usage(QTextStream & out) {
    out <<  "bil_rectifier bil_sequence_folder input_dtm [output_folder = ./rectified]" << Qt::endl;
    return out;
}

template<typename T>
int projectBilSequence(std::vector<std::string> const& filesList, int argc, char** argv) {

    QTextStream out(stdout);

    if (argc < 2) {
        out << "Missing input bil folder argument" << Qt::endl;
        usage(out);
        return 1;
    }

    std::string bil_folderpath(argv[1]);

    if (argc < 3) {
        out << "Missing input digital terrain model argument" << Qt::endl;
        usage(out);
        return 1;
    }

    std::string dtm_filepath(argv[2]);

    std::string output_folder;

    if (argc == 4) {
        output_folder = argv[3];
    } else {
        output_folder = bil_folderpath;
    }

    if (output_folder.back() != '/') {
        output_folder += '/';
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

    QVector<QRectF> bilFilesROI(filesList.size()); //keep track of the regions of the terrain (in pixels coordinates) which have already been covered

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
                header.count("framerate") <= 0) {
            out << "Failed to extract field of view and samples for file \"" << QString::fromStdString(file) << "\"" << Qt::endl;
            continue;
        }

        double fieldOfViewDeg = stod(header["field of view"]);
        double fieldOfViewRad = fieldOfViewDeg/180 * M_PI;
        int nSamples = stoi(header["samples"]);
        int nLines = stoi(header["lines"]);
        double frameRate = stod(header["framerate"]);
        double frameTime = 1/frameRate;

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
        auto trajOpt = convertLcfSequenceToTrajectory(rawTrajectory);

        if (!trajOpt.has_value()) {
            out << "Failed to extract trajectory for file \"" << QString::fromStdString(file) << "\"" << Qt::endl;
            continue;
        }

        Trajectory<double>& trajectory = trajOpt.value();
        double t0 = trajectory.times[0];

        int currentPoseIndex = 0;

        for (int i = 0; i < nLines; i++) {

            out << "\r\t" << "Treating line " << (i+1) << "/" << nLines;
            out.flush();

            double targetTime = t0 + i*frameTime;

            while (trajectory.times[currentPoseIndex] < targetTime) {
                if (currentPoseIndex + 1 == trajectory.times.size()) {
                    break;
                }
                currentPoseIndex++;
            }

            int previousPoseIndex = std::max(0,currentPoseIndex-1);

            double nextTime = trajectory.times[currentPoseIndex];
            double previousTime = trajectory.times[previousPoseIndex];

            double delta_t = targetTime - previousTime;

            if (targetTime == previousTime) {
                delta_t = 1;
            }

            StereoVision::Geometry::AffineTransform<double> const& poseNext = trajectory.poses[currentPoseIndex];
            StereoVision::Geometry::AffineTransform<double> const& posePrevious = trajectory.poses[previousPoseIndex];

            double coefficient = (targetTime - previousTime)/delta_t;

            Eigen::Vector3d rNext = StereoVision::Geometry::inverseRodriguezFormula(poseNext.R);
            Eigen::Vector3d rPrev = StereoVision::Geometry::inverseRodriguezFormula(posePrevious.R);

            Eigen::Vector3d rInterp = coefficient*rNext + (1-coefficient)*rPrev;

            Eigen::Matrix3d interpR = StereoVision::Geometry::rodriguezFormula(rInterp);
            Eigen::Vector3d interpT = coefficient*poseNext.t + (1-coefficient)*posePrevious.t;

            StereoVision::Geometry::AffineTransform<double> pose(interpR, interpT);

            std::array<double, 3> ecefOrigin{pose.t.x(), pose.t.y(), pose.t.z()};

            std::vector<std::array<float, 3>> viewDirectionsECEF(nSamples);

            for (int i = 0; i < nSamples; i++) {
                Eigen::Vector3d vec(viewDirectionsSensor[i][0], viewDirectionsSensor[i][1], viewDirectionsSensor[i][2]);
                Eigen::Vector3d transformed = pose.R*vec;

                viewDirectionsECEF[i] = std::array<float, 3>{float(transformed.x()), float(transformed.y()), float(transformed.z())};
            }

            auto projResOpt = projector.projectVectorsOptimized(ecefOrigin, viewDirectionsECEF);

            if (!projResOpt.has_value()) {
                continue;
            }

            StereoVisionApp::Geo::TerrainProjector<double>::ProjectionResults& results = projResOpt.value();

            for (int j = 0; j < results.projectedPoints.size(); j++) {
                std::array<float,2> proj = results.projectedPoints[j];

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

                projectedCoordinates.atUnchecked(j,i,0) = proj[0];
                projectedCoordinates.atUnchecked(j,i,1) = proj[1];

            }

        }
        out << Qt::endl;

        bilFilesROI[fileId] = QRectF(QPointF(min_x, min_y), QPointF(max_x, max_y));

        StereoVision::IO::writeStevimg<float>(file + "_projected_coords.stevimg", projectedCoordinates);
    }

    out << "Processing done!" << Qt::endl;

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

    if (argc < 2) {
        out << "Missing input bil folder argument" << Qt::endl;
        usage(out);
        return 1;
    }

    std::string bil_folderpath(argv[1]);
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
