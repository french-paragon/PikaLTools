#include <iostream>
#include <vector>
#include <string>
#include <optional>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <random>

#include <StereoVision/geometry/genericraysalignement.h>

#include "io/read_envi_bil.h"
#include "geo/coordinate_conversions.h"

struct BilInfos {

    BilInfos(
            int pinternalId = -1,
            std::string pfileName = "",
            std::string pshortName = "",
            int poffsetSbg = 0) :
        internalId(pinternalId),
        fileName(pfileName),
        shortName(pshortName),
        offsetSbg(poffsetSbg)
    {

    }

    int internalId;
    std::string fileName;
    std::string shortName;
    int offsetSbg;
};

std::optional<std::vector<BilInfos>> loadBilsInfos(std::string const& filepath) {

    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        return std::nullopt;
    }

    std::vector<BilInfos> ret;

    std::string line;
    //get the header and do nothing with it.
    std::getline(inFile, line);

    while (!inFile.eof()) {
        std::getline(inFile, line);

        int coma1Pos = line.find(',');

        if (coma1Pos == std::string::npos) {
            continue;
        }

        int coma2Pos = line.find(',', coma1Pos+1);

        if (coma2Pos == std::string::npos) {
            continue;
        }

        int coma3Pos = line.find(',', coma2Pos+1);

        if (coma3Pos == std::string::npos) {
            continue;
        }

        std::string id_str = line.substr(0,coma1Pos);
        std::string path_str = line.substr(coma1Pos+1, coma2Pos-coma1Pos-1);
        std::string shortName_str = line.substr(coma2Pos+1, coma3Pos-coma2Pos-1);
        std::string offset_str = line.substr(coma3Pos+1, line.size()-coma3Pos-1);

        int id;
        int offset;

        try {

            id = std::stoi(id_str);
            offset = std::stoi(offset_str);

        } catch (std::invalid_argument const& e)  {
            continue;
        }

        ret.emplace_back(id, std::move(path_str), std::move(shortName_str), offset);
    }

    inFile.close();

    return ret;

}

struct ChunkInfos {

    ChunkInfos(
            int pchunkId = -1,
            int pbilId = -1) :
        chunkId(pchunkId),
        bilId(pbilId)
    {

    }

    int chunkId;
    int bilId;
};

std::optional<std::vector<ChunkInfos>> loadChunksInfos(std::string const& filepath) {

    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        return std::nullopt;
    }

    std::vector<ChunkInfos> ret;

    std::string line;
    //get the header and do nothing with it.
    std::getline(inFile, line);

    while (!inFile.eof()) {
        std::getline(inFile, line);

        std::istringstream f(line);
        std::string s;

        std::getline(f, s, ',');

        std::getline(f, s, ',');
        std::string id_str = s;

        std::getline(f, s, ',');
        std::string bilid_str = s;

        int chunk_id;
        int bil_id;

        try {

            chunk_id = std::stoi(id_str);

            bil_id = std::stoi(bilid_str);

        }  catch (std::invalid_argument const& e) {
           continue;
        }

        ret.emplace_back(chunk_id, bil_id);
    }

    return ret;

}

struct RawPointInfos {

    RawPointInfos(
            int pinternalId = -1,
            int pchunk1_id = -1,
            int pchunk2_id = -1,
            int px1 = 0,
            int py1 = 0,
            int px2 = 0,
            int py2 = 0) :
        internalId(pinternalId),
        chunk1_id(pchunk1_id),
        chunk2_id(pchunk2_id),
        x1(px1),
        y1(py1),
        x2(px2),
        y2(py2)

    {

    }

    int internalId;
    int chunk1_id;
    int chunk2_id;
    int x1; //coordinate horizontal bil1
    int y1; //coordinate vertical bil1
    int x2; //coordinate horizontal bil1
    int y2; //coordinate vertical bil1
};

std::optional<std::vector<RawPointInfos>> loadRawMatchInfos(std::string const& filepath) {

    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        return std::nullopt;
    }

    std::vector<RawPointInfos> ret;

    std::string line;
    //get the header and do nothing with it.
    std::getline(inFile, line);

    while (!inFile.eof()) {
        std::getline(inFile, line);

        std::istringstream f(line);
        std::string s;

        std::getline(f, s, ',');
        std::string id_str = s;

        std::getline(f, s, ',');
        std::string chunk1id_str = s;

        std::getline(f, s, ',');
        std::string chunk2id_str = s;

        //skip coordinates in chunks, got to coordinates in bil.
        std::getline(f, s, ',');
        std::getline(f, s, ',');
        std::getline(f, s, ',');
        std::getline(f, s, ',');

        std::getline(f, s, ',');
        std::string x1_str = s;

        std::getline(f, s, ',');
        std::string y1_str = s;

        std::getline(f, s, ',');
        std::getline(f, s, ',');

        std::getline(f, s, ',');
        std::string x2_str = s;

        std::getline(f, s, ',');
        std::string y2_str = s;

        int id;

        int chunk1_id;
        int chunk2_id;

        int x1;
        int y1;

        int x2;
        int y2;

        try {

            id = std::stoi(id_str);

            chunk1_id = std::stoi(chunk1id_str);
            chunk2_id = std::stoi(chunk2id_str);

            x1 = std::stoi(x1_str);
            y1 = std::stoi(y1_str);

            x2 = std::stoi(x2_str);
            y2 = std::stoi(y2_str);

        }  catch (std::invalid_argument const& e) {
           continue;
        }

        if (x1 == 0 or y1 == 0 or x2 == 0 or y2 == 0) {
            continue; //ignore points when it could not be reprojected.
        }

        ret.emplace_back(id, chunk1_id, chunk2_id, x1, y1, x2, y2);
    }

    return ret;
}

using RayPairInfos =  StereoVision::Geometry::RayPairInfos<double>;

std::optional<std::vector<RayPairInfos>> getRayPairsInfos(std::vector<RawPointInfos> const& rawPairsInfos,
                                                          std::vector<ChunkInfos> const& chunksInfos,
                                                          std::vector<BilInfos> const& bilsInfos,
                                                          std::set<int> const& inliers) {

    struct BilData {
        std::map<std::string, std::string> bilHeaderData;
        std::vector<double> linesTimes;
        Trajectory<double> trajectory;
        std::vector<Eigen::Vector3d> viewDirectionsSensor;
    };

    std::vector<BilData> bilsData;
    bilsData.reserve(bilsInfos.size());

    for (BilInfos const& bilInfos : bilsInfos) {

        std::optional<std::map<std::string, std::string>> headerOpt = readBilHeaderData(bilInfos.fileName);

        if (!headerOpt.has_value()) {
            return std::nullopt;
        }

        std::vector<double> linesTimes = get_envi_bil_lines_times(bilInfos.fileName);

        if (linesTimes.empty()) {
            return std::nullopt;
        }

        std::vector<EnviBilLcfLine> lcfData = read_envi_bil_lcf_data(bilInfos.fileName);

        if (lcfData.empty()) {
            return std::nullopt;
        }

        std::optional<Trajectory<double>> trajOpt = convertLcfSequenceToTrajectory(lcfData);

        if (!trajOpt.has_value()) {
            return std::nullopt;
        }

        std::map<std::string, std::string>& header = headerOpt.value();

        if (header.count("field of view") <= 0 or
                header.count("samples") <= 0) {
            return std::nullopt;
        }

        double fieldOfViewDeg = stod(header["field of view"]);
        double fieldOfViewRad = fieldOfViewDeg/180 * M_PI;
        int nSamples = stoi(header["samples"]);

        double midPoint = static_cast<double>(nSamples-1)/2;
        double fLenPix = static_cast<double>(nSamples-1)/2 * std::tan(M_PI_2 - fieldOfViewRad/2);

        std::vector<Eigen::Vector3d> viewDirectionsSensor(nSamples);

        for (int i = 0; i < nSamples; i++) {
            viewDirectionsSensor[i] = Eigen::Vector3d(0, midPoint - i, fLenPix);
            viewDirectionsSensor[i].normalize();
        }

        bilsData.push_back({headerOpt.value(), linesTimes, trajOpt.value(), viewDirectionsSensor});
    }

    std::vector<RayPairInfos> ret;
    ret.reserve(rawPairsInfos.size());

    for (RawPointInfos const& matchInfos : rawPairsInfos) {

        if (inliers.count(matchInfos.internalId) <= 0 and !inliers.empty()) {
            continue; //do not include outliers
        }

        int bil1_id = chunksInfos[matchInfos.chunk1_id].bilId;
        int bil2_id = chunksInfos[matchInfos.chunk2_id].bilId;

        if (bil1_id == bil2_id) {
            continue;
        }

        BilData const& bil1Data = bilsData[bil1_id];
        BilData const& bil2Data = bilsData[bil2_id];

        int line1 = matchInfos.y1;
        int line2 = matchInfos.y2;

        double time1 = bil1Data.linesTimes[line1];
        double time2 = bil2Data.linesTimes[line2];

        auto pose1Interp = bil1Data.trajectory.getValueAtTime(time1);
        StereoVision::Geometry::RigidBodyTransform<double> pose1 = pose1Interp.weigthLower*pose1Interp.valLower + pose1Interp.weigthUpper*pose1Interp.valUpper;
        //body1Toecef

        auto pose2Interp = bil2Data.trajectory.getValueAtTime(time2);
        StereoVision::Geometry::RigidBodyTransform<double> pose2 = pose2Interp.weigthLower*pose2Interp.valLower + pose2Interp.weigthUpper*pose2Interp.valUpper;
        //body2Toecef

        //body1ToBody2
        StereoVision::Geometry::RigidBodyTransform<double> pose1ToPose2 = pose2.inverse()*pose1;
        StereoVision::Geometry::AffineTransform<double> pose1ToPose2Aff = pose1ToPose2.toAffineTransform();

        Eigen::Vector3d v1 = bil1Data.viewDirectionsSensor[matchInfos.x1];
        Eigen::Vector3d v2 = bil2Data.viewDirectionsSensor[matchInfos.x2];

        if (!pose1ToPose2Aff.R.array().isFinite().all() or !pose1ToPose2Aff.t.array().isFinite().all()) {
            continue;
        }

        ret.push_back(RayPairInfos{v1, v2, pose1ToPose2Aff.R, pose1ToPose2Aff.t});

    }

    return ret;

}

std::set<int> loadInliersSet(std::string const& filepath) {

    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        return std::set<int>();
    }

    std::set<int> ret;

    std::string line;
    //get the header and do nothing with it.

    while (!inFile.eof()) {
        std::getline(inFile, line);

        int id;

        try {
            id = std::stoi(line);
        } catch (std::invalid_argument const& e) {
            continue;
        }

        ret.insert(id);
    }

    inFile.close();
    return ret;
}

struct ExperimentResults {
    int nSamples;
    int nFails;
    double expectedAngularError;
    double meanIterationNumber;
    Eigen::Vector3d avgAxisAngle;
};

template<bool relaxed = true>
ExperimentResults runExperiment(std::vector<RayPairInfos> const& pairsInfos,
                                int nRepeat = 100,
                                double bootstrapprop = 0.5,
                                int maxBootstrapObs = 500,
                                bool robust = false,
                                int seed = 4269) {

    constexpr int maxiter = 500;
    constexpr double tol = 1e-2;
    constexpr double huberRange = 0.25;

    int nBootstrapSamples = std::round(bootstrapprop*pairsInfos.size());

    if (nBootstrapSamples > maxBootstrapObs) {
        nBootstrapSamples = maxBootstrapObs;
    }

    if (nBootstrapSamples > pairsInfos.size()) {
        nBootstrapSamples = pairsInfos.size();
    }

    int repeats = (nBootstrapSamples == pairsInfos.size()) ? 1 : nRepeat;

    std::vector<Eigen::Vector3d> results;
    results.reserve(repeats);

    ExperimentResults ret;
    ret.nSamples = 0;
    ret.nFails = 0;
    ret.expectedAngularError = std::numeric_limits<double>::infinity();
    ret.meanIterationNumber = 0;

    Eigen::Vector3d avg = Eigen::Vector3d::Zero();

    std::vector<int> idxs(pairsInfos.size());

    for (int i = 0; i < idxs.size(); i++) {
        idxs[i] = i;
    }

    std::default_random_engine rng;
    rng.seed(seed);

    auto rngRange = [&rng] (int i) {
        return rng()%i;
    };

    for (int i = 0; i < repeats; i++) {

        std::random_shuffle(idxs.begin(), idxs.end(), rngRange);

        std::vector<RayPairInfos> bootstrapSamples(nBootstrapSamples);

        for (int i = 0; i < nBootstrapSamples; i++) {
            bootstrapSamples[i] = pairsInfos[idxs[i]];
        }

        StereoVision::Optimization::GaussNewtownKernel<double>* kernel = nullptr;

        if (robust) {
            kernel = new StereoVision::Optimization::GaussNewtownHuberKernel(huberRange);
        }

        StereoVision::Geometry::AxisRaysSetsAligner aligner(bootstrapSamples, kernel);

        StereoVision::ConvergenceType convergence = aligner.run(maxiter, tol);

        if (convergence == StereoVision::ConvergenceType::Failed or
                convergence == StereoVision::ConvergenceType::Unknown) {
            ret.nFails++;
            continue;
        }

        Eigen::Vector3d axisAngle = aligner.solution();

        if (!axisAngle.array().allFinite()) {
            ret.nFails++;
            continue;
        }

        Eigen::Matrix3d R = StereoVision::Geometry::rodriguezFormula(axisAngle);
        axisAngle = StereoVision::Geometry::inverseRodriguezFormula(R);

        results.push_back(axisAngle);
        ret.nSamples++;

        ret.meanIterationNumber += aligner.nIterations();

        avg += axisAngle;

    }

    ret.meanIterationNumber /= ret.nSamples;

    avg /= ret.nSamples;
    ret.avgAxisAngle = avg;

    double meanAngleError = 0;

    for (Eigen::Vector3d axis : results) {
        meanAngleError += (avg - axis).norm();
    }

    meanAngleError /= ret.nSamples;

    ret.expectedAngularError = meanAngleError/M_PI * 180;

    ret.avgAxisAngle *= 180/M_PI;

    return ret;

}

int main(int argc, char** argv) {

    if (argc != 5) {
        std::cerr << "Invalid arguments provided" << "\n";
        std::cerr << "Expected ./exec path_to_bils.csv chunks_infos.csv path_to_tiepoints_data.csv path_to_outliers.txt" << std::endl;
        return 1;
    }

    std::string bils_csv_path = argv[1];
    std::string chunks_csv_path = argv[2];
    std::string tie_points_data_csv_path = argv[3];
    std::string tie_points_inliers_path = argv[4];

    std::optional<std::vector<BilInfos>> bilsInfosOpt = loadBilsInfos(bils_csv_path);

    if (!bilsInfosOpt.has_value()) {
        std::cerr << "Failed to read bils infos file: " << bils_csv_path << std::endl;
        return 1;
    }

    std::vector<BilInfos>& bilsInfos = bilsInfosOpt.value();

    std::optional<std::vector<ChunkInfos>> chunksInfosOpt = loadChunksInfos(chunks_csv_path);

    if (!chunksInfosOpt.has_value()) {
        std::cerr << "Failed to read chunks infos file: " << chunks_csv_path << std::endl;
        return 1;
    }

    std::vector<ChunkInfos>& chunksInfos = chunksInfosOpt.value();

    std::optional<std::vector<RawPointInfos>> tiePointsInfosOpt = loadRawMatchInfos(tie_points_data_csv_path);

    if (!tiePointsInfosOpt.has_value()) {
        std::cerr << "Failed to read tie points infos file: " << chunks_csv_path << std::endl;
        return 1;
    }

    std::vector<RawPointInfos>& tiePointsInfos = tiePointsInfosOpt.value();

    std::set<int> inliersSet = loadInliersSet(tie_points_inliers_path);

    std::optional<std::vector<RayPairInfos>> rayPairsInfosOpt = getRayPairsInfos(tiePointsInfos, chunksInfos, bilsInfos, inliersSet);

    if (!rayPairsInfosOpt.has_value()) {
        std::cerr << "Unable to process points data" << std::endl;
        return 1;
    }

    std::vector<RayPairInfos>& rayPairsInfos = rayPairsInfosOpt.value();

    std::cout << "Loaded " << rayPairsInfos.size() << " pairs infos, starting experiment..." << std::endl;

    constexpr int nRepeats = 100;
    constexpr double bootstrapProp = 0.5;
    constexpr int maxBootstrapObs = 500;

    ExperimentResults results = runExperiment<false>(rayPairsInfos, nRepeats, bootstrapProp, maxBootstrapObs);

    if (results.nSamples <= 0) {
        std::cerr << "Failed to run experiment!" << std::endl;
        return 1;
    }

    std::cout << "Experiment ran with " << results.nSamples << " samples, ";
    std::cout << "expected error is " << results.expectedAngularError << ", ";
    std::cout << "mean number of iterations is " << results.meanIterationNumber << ", ";
    std::cout << "mean rotation axis is " << results.avgAxisAngle.x() << " " << results.avgAxisAngle.y() << " " << results.avgAxisAngle.z() << "." << std::endl;

    ExperimentResults resultsRobust = runExperiment<false>(rayPairsInfos, nRepeats, bootstrapProp, maxBootstrapObs, true);

    if (resultsRobust.nSamples <= 0) {
        std::cerr << "Failed to run robust experiment!" << std::endl;
        return 1;
    }

    std::cout << "Robust experiment ran with " << resultsRobust.nSamples << " samples, ";
    std::cout << "expected error is " << resultsRobust.expectedAngularError << ", ";
    std::cout << "mean number of iterations is " << resultsRobust.meanIterationNumber << ", ";
    std::cout << "mean rotation axis is " << resultsRobust.avgAxisAngle.x() << " " << resultsRobust.avgAxisAngle.y() << " " << resultsRobust.avgAxisAngle.z() << "." << std::endl;

    return 0;
}
