#include "cerespushbroomsolver.h"

#include "datablocks/bilacquisitiondata.h"

#include "steviapp/datablocks/landmark.h"
#include "steviapp/sparsesolver/costfunctors/imustepcost.h"
#include "steviapp/sparsesolver/costfunctors/weightedcostfunction.h"

#include "cost_functors/parametrizedxyz2pushbroom.h"
#include "cost_functors/orientationspeedconsistency.h"

#include <ceres/normal_prior.h>

#include <random>

namespace PikaLTools {


class CeresPushBroomSolverIterationCallback : public ceres::IterationCallback {
public:

    CeresPushBroomSolverIterationCallback(CeresPushBroomSolver* solver) :
        _solver(solver)
    {

    }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {

        _solver->jumpStep();

        if (summary.step_is_valid) {
            return ceres::SOLVER_CONTINUE;
        }

        return ceres::SOLVER_ABORT;

    }

protected:

    CeresPushBroomSolver* _solver;
};

CeresPushBroomSolver::CeresPushBroomSolver(StereoVisionApp::Project* p,
                                           const StereoVision::Geometry::AffineTransform<double> &initialLeverArm,
                                           BilSequenceAcquisitionData* push_broom_sequence,
                                           GPSMeasurementInfos const& gpsMeasurements,
                                           InitialOrientationInfos const& initialOrientations,
                                           AccMeasurementInfos const& insMeasurements,
                                           GyroMeasurementInfos const& imuMeasurements,
                                           const AdditionalPointsInfos &additionalPoints,
                                           const AdditionalViewsInfos &additionalViews,
                                           bool computeUncertainty,
                                           bool sparse,
                                           QObject* parent) :
    StereoVisionApp::SparseSolverBase(p,parent),
    _sparse(sparse),
    _compute_marginals(computeUncertainty),
    _sequence(push_broom_sequence),
    _gpsMeasurements(gpsMeasurements),
    _initialOrientations(initialOrientations),
    _accMeasurements(insMeasurements),
    _gyroMeasurements(imuMeasurements),
    _additionalPoints(additionalPoints),
    _additionalViews(additionalViews),
    _resultsDataTable(nullptr),
    _random_seed(0)
{

    Eigen::Vector3d rAxis = StereoVision::Geometry::inverseRodriguezFormula(initialLeverArm.R);

    _initialLeverArm.rAxis[0] = rAxis[0];
    _initialLeverArm.rAxis[1] = rAxis[1];
    _initialLeverArm.rAxis[2] = rAxis[2];

    _initialLeverArm.t[0] = initialLeverArm.t[0];
    _initialLeverArm.t[1] = initialLeverArm.t[1];
    _initialLeverArm.t[2] = initialLeverArm.t[2];

    if (push_broom_sequence != nullptr) {
        if (push_broom_sequence->isInfosOnly()) {
            _sensorWidth = push_broom_sequence->sequenceInfos().lineWidth;
        } else {
            auto bilsInfos = push_broom_sequence->getBilInfos();

            if (bilsInfos.size() > 0) {
                auto header = bilsInfos[0].headerData();
                QString widthStr = header.value("samples", "-1");

                bool ok;
                _sensorWidth = widthStr.toDouble(&ok);

                if (!ok) {
                    _sensorWidth = -1;
                }
            }
        }
    }

}

CeresPushBroomSolver::CeresPushBroomSolver(StereoVisionApp::Project* p,
                                           const StereoVision::Geometry::AffineTransform<double> &initialLeverArm,
                                           SequenceInfos const& pseudo_push_broom_sequence,
                                           GPSMeasurementInfos const& gpsMeasurements,
                                           InitialOrientationInfos const& initialOrientations,
                                           AccMeasurementInfos const& insMeasurements,
                                           GyroMeasurementInfos const& imuMeasurements,
                                           AdditionalPointsInfos const& additionalPoints,
                                           AdditionalViewsInfos const& additionalViews,
                                           bool computeUncertainty,
                                           bool sparse,
                                           QObject* parent) :
    CeresPushBroomSolver(p,
                         initialLeverArm,
                         nullptr,
                         gpsMeasurements,
                         initialOrientations,
                         insMeasurements,
                         imuMeasurements,
                         additionalPoints,
                         additionalViews,
                         computeUncertainty,
                         sparse,
                         parent)
{
    _pseudo_sequence = pseudo_push_broom_sequence;
}

int CeresPushBroomSolver::uncertaintySteps() const {
    return (_compute_marginals) ? 1 : 0;
}

bool CeresPushBroomSolver::hasUncertaintyStep() const  {
    return _compute_marginals;
}

bool CeresPushBroomSolver::init() {

    std::cout << "Entering init function" << std::endl;

    if (_currentProject == nullptr) {
        return false;
    }

    if (_sensorWidth < 0) {
        return false;
    }

    QMap<QString, QString> sequence_header;

    BilSequenceAcquisitionData* seq = _sequence;

    if (seq != nullptr) {

        if (seq->getBilFiles().size() == 0) { //sequence is just a tie point container
            _pseudo_sequence = seq->sequenceInfos();
            seq = nullptr;
        }
    }

    if (seq != nullptr) {

        sequence_header = seq->getBilInfos()[0].headerData();

    }

    bool s = true;

    int nPoses;

    if (seq != nullptr) {
        nPoses = seq->nLinesInSequence();
    } else {
        nPoses = _pseudo_sequence.nLines;
    }

    if (nPoses <= 0) {
        return false;
    }

    int nLandmarks = 0;
    int nAdditionalPts = 0;
    QSet<qint64> lmIdxs;

    if (_sequence != nullptr) {
        QVector<qint64> idxs = _sequence->getAttachedLandmarksIds();
        lmIdxs = QSet<qint64>(idxs.begin(), idxs.end());
    }

    QVector<pointInfos> additionalPoints = _additionalPoints.compilePoints();
    QVector<viewInfos> additionalViews = _additionalViews.compilePoints();

    QMap<qint64, pointInfos> additionalPtsPositions;

    //register additional landmarks
    for (viewInfos const& viewInfo : additionalViews) {

        if (viewInfo.lmId >= 0) {
            lmIdxs.insert(viewInfo.lmId);

        } //register additional landmarks

    }

    //register additional points.
    for (pointInfos const& ptInfo : additionalPoints) {
        additionalPtsPositions.insert(ptInfo.additionalPtId, ptInfo);
    }

    nLandmarks = lmIdxs.size();
    nAdditionalPts = additionalPtsPositions.size();

    QVector<viewInfos> lmObs;
    lmObs.reserve(_sequence->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className()).size() + additionalViews.size());

    if (_sequence != nullptr) {
        QVector<qint64> imlmids = _sequence->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

        for (qint64 id : imlmids) {
            BilSequenceLandmark* lm = _sequence->getBilSequenceLandmark(id);

            if (lm == nullptr) {
                continue;
            }

            lmObs.push_back({lm->attachedLandmarkid(), -1, lm->x().value(), lm->y().value()});
        }
    }
    for (viewInfos const& viewInfo : additionalViews) {
            lmObs.push_back(viewInfo);
    }

    std::sort(lmObs.begin(), lmObs.end(), [] (viewInfos const& v1, viewInfos const& v2) {
        return v1.y < v2.y;
    });

    //prepare  time sequences:
    IndexedVec3TimeSeq GPSpos = _gpsMeasurements.compileSequence();
    IndexedVec3TimeSeq InitialOrientations = _initialOrientations.compileSequence();
    IndexedVec3TimeSeq AccMeasurements = _accMeasurements.compileSequence();
    IndexedVec3TimeSeq GyroMeasurements = _gyroMeasurements.compileSequence();

    std::default_random_engine randomEngine;
    randomEngine.seed(_random_seed+1237862); //get a unique random seed for this function

    std::normal_distribution<double> gpsErrorDist(0,_gpsAccuracy);
    std::normal_distribution<double> gyroErrorDist(0,_gyroAccuracy);
    std::normal_distribution<double> accErrorDist(0,_accAccuracy);

    //init gravity, lever arm and other fix parameters
    _gravity.vec[0] = 0;
    _gravity.vec[1] = 0;
    _gravity.vec[2] = 9.81;

    _problem.AddParameterBlock(_gravity.vec.data(), _gravity.vec.size());

    _leverArm = _initialLeverArm;

    _problem.AddParameterBlock(_leverArm.rAxis.data(), _leverArm.rAxis.size());
    _problem.AddParameterBlock(_leverArm.t.data(), _leverArm.t.size());

    if (seq != nullptr) {

        double fov = sequence_header.value("").toDouble();
        double fov_rad = fov*M_PI/180.;
        _sensorParameters.fLen[0] = sensorWidth()*std::tan(M_PI_2-fov_rad/2.)/2.;

    } else {
        _sensorParameters.fLen[0] = _pseudo_sequence.fLen;
    }

    _sensorParameters.principalPoint[0] = _sensorWidth/2;
    std::fill(_sensorParameters.lateralDistortion.begin(), _sensorParameters.lateralDistortion.end(), 0);
    std::fill(_sensorParameters.frontalDistortion.begin(), _sensorParameters.frontalDistortion.end(), 0);

    _problem.AddParameterBlock(_sensorParameters.fLen.data(), _sensorParameters.fLen.size());
    _problem.AddParameterBlock(_sensorParameters.principalPoint.data(), _sensorParameters.principalPoint.size());
    _problem.AddParameterBlock(_sensorParameters.lateralDistortion.data(), _sensorParameters.lateralDistortion.size());
    _problem.AddParameterBlock(_sensorParameters.frontalDistortion.data(), _sensorParameters.frontalDistortion.size());

    //init landmarks

    _landmarksParameters.resize(nLandmarks + nAdditionalPts);

    int i = 0;

    for (qint64 lmIdx : lmIdxs) {
        StereoVisionApp::Landmark* lm = _currentProject->getDataBlock<StereoVisionApp::Landmark>(lmIdx);

        LandmarkPos& pos = _landmarksParameters[i];

        _landmarkParametersIndex.insert(lmIdx, i);

        pos.position[0] = lm->optPos().value(0);
        pos.position[1] = lm->optPos().value(1);
        pos.position[2] = lm->optPos().value(2);

        _problem.AddParameterBlock(pos.position.data(), pos.position.size());

        if (lm->xCoord().isSet() and lm->yCoord().isSet() and lm->zCoord().isSet()) {

            ceres::Vector m;
            m.resize(3);

            m[0] = lm->xCoord().value();
            m[1] = lm->yCoord().value();
            m[2] = lm->zCoord().value();

            pos.position[0] = m[0];
            pos.position[1] = m[1];
            pos.position[2] = m[2];

            ceres::Matrix stiffness = Eigen::Matrix3d::Identity();

            if (lm->xCoord().isUncertain()) {
                stiffness(0,0) = 1./lm->xCoord().stddev();
            } else {
                stiffness(0,0) = 1e6;
            }

            if (lm->yCoord().isUncertain()) {
                stiffness(1,1) = 1./lm->yCoord().stddev();
            } else {
                stiffness(1,1) = 1e6;
            }

            if (lm->zCoord().isUncertain()) {
                stiffness(2,2) = 1./lm->zCoord().stddev();
            } else {
                stiffness(2,2) = 1e6;
            }

           if (lm->xCoord().isUncertain() or lm->yCoord().isUncertain() or lm->zCoord().isUncertain()) {
                ceres::NormalPrior* normalPrior = new ceres::NormalPrior(stiffness, m);

                _problem.AddResidualBlock(normalPrior, nullptr, pos.position.data());
            } else {
                _problem.SetParameterBlockConstant(pos.position.data());
            }
        }

        i++;
    }

    QList<qint64> additionalPtsIdxs = additionalPtsPositions.keys();

    for (qint64 lmIdx : additionalPtsIdxs) {

        pointInfos ptInfo = additionalPtsPositions[lmIdx];

        LandmarkPos& pos = _landmarksParameters[i];

        _additionalPointsParametersIndex.insert(lmIdx, i);

        pos.position[0] = ptInfo.x;
        pos.position[1] = ptInfo.y;
        pos.position[2] = ptInfo.z;

        _problem.AddParameterBlock(pos.position.data(), pos.position.size());

        i++;
    }

    //init poses

    _frameParameters.resize(nPoses);

    std::vector<double> lines_times;

    if (seq != nullptr) {
        lines_times = seq->ecefTimes();
    } else {
        lines_times.resize(nPoses);

        for (int i = 0; i < nPoses; i++) {
            lines_times[i] = _pseudo_sequence.initial_time + i*_pseudo_sequence.time_per_line;
        }
    }

    int lmViewIdx = 0;

    for (int i = 0; i < nPoses; i++) {

        FramePoseParameters & pose = _frameParameters[i];

        double time = lines_times[i];

        auto pos = GPSpos.getValueAtTime(time);
        auto orientation = InitialOrientations.getValueAtTime(time);

        pose.time = time;

        pose.rAxis[0] = orientation.weigthLower*orientation.valLower[0] + orientation.weigthUpper*orientation.valUpper[0];
        pose.rAxis[1] = orientation.weigthLower*orientation.valLower[1] + orientation.weigthUpper*orientation.valUpper[1];
        pose.rAxis[2] = orientation.weigthLower*orientation.valLower[2] + orientation.weigthUpper*orientation.valUpper[2];

        pose.t[0] = pos.weigthLower*pos.valLower[0] + pos.weigthUpper*pos.valUpper[0];
        pose.t[1] = pos.weigthLower*pos.valLower[1] + pos.weigthUpper*pos.valUpper[1];
        pose.t[2] = pos.weigthLower*pos.valLower[2] + pos.weigthUpper*pos.valUpper[2];

        pose.tGt = pose.t;
        pose.rAxisGt = pose.rAxis;

        pose.t[0] += gpsErrorDist(randomEngine);
        pose.t[1] += gpsErrorDist(randomEngine);
        pose.t[2] += gpsErrorDist(randomEngine);

        pose.rAxis[0] += gyroErrorDist(randomEngine);
        pose.rAxis[1] += gyroErrorDist(randomEngine);
        pose.rAxis[2] += gyroErrorDist(randomEngine);

        pose.v[0] = 0;
        pose.v[1] = 0;
        pose.v[2] = 0;

        pose.wAxis[0] = 0;
        pose.wAxis[1] = 0;
        pose.wAxis[2] = 0;

        pose.tInitial = pose.t;
        pose.rAxisInitial = pose.rAxis;

        _problem.AddParameterBlock(pose.rAxis.data(), pose.rAxis.size());
        _problem.AddParameterBlock(pose.wAxis.data(), pose.wAxis.size());
        _problem.AddParameterBlock(pose.t.data(), pose.t.size());
        _problem.AddParameterBlock(pose.v.data(), pose.v.size());

        //add GPS position prior
        ceres::Vector m;
        m.resize(3);

        m[0] = pose.t[0];
        m[1] = pose.t[1];
        m[2] = pose.t[2];

        ceres::Vector mr;
        mr.resize(3);

        mr[0] = pose.rAxis[0];
        mr[1] = pose.rAxis[1];
        mr[2] = pose.rAxis[2];

        ceres::Matrix stiffness = Eigen::Matrix3d::Identity();

        stiffness(0,0) = 1./_gpsAccuracy;
        stiffness(1,1) = 1./_gpsAccuracy;
        stiffness(2,2) = 1./_gpsAccuracy;

        ceres::NormalPrior* positionNormalPrior = new ceres::NormalPrior(stiffness, m);
        ceres::NormalPrior* attitudeNormalPrior = new ceres::NormalPrior(stiffness, mr);

        _problem.AddResidualBlock(positionNormalPrior, nullptr, pose.t.data());
        _problem.AddResidualBlock(attitudeNormalPrior, nullptr, pose.rAxis.data());

        //add eventual point measurements

        while (lmViewIdx < lmObs.size() and lmObs[lmViewIdx].y < i+0.5) {

            int lm_idx = -1;

            if (lmObs[lmViewIdx].lmId >= 0) {
                lm_idx = _landmarkParametersIndex[lmObs[lmViewIdx].lmId];
            } else if (lmObs[lmViewIdx].additionalPtId >= 0) {
                lm_idx = _additionalPointsParametersIndex[lmObs[lmViewIdx].additionalPtId];
            } else {
                lmViewIdx++;
                continue;
            }

            double y = lmObs[lmViewIdx].y;
            int pLineId = std::floor(y);
            int nLineId = std::ceil(y);

            if (pLineId == nLineId) {
                nLineId += 1;
            }

            if (nLineId >= nPoses) {
                nLineId--;
                pLineId--;
            }

            if (pLineId < 0) {
                pLineId++;
                nLineId++;
            }

            double pixelsTimeInterval = lines_times[nLineId] - lines_times[pLineId]; //compute the time interval for current pose

            Eigen::Vector2d uv(lmObs[lmViewIdx].x, lmObs[lmViewIdx].y-i);
            ParametrizedXYZ2PushBroom* projectionCost = new ParametrizedXYZ2PushBroom(uv, _sensorWidth, pixelsTimeInterval);

            ceres::AutoDiffCostFunction<ParametrizedXYZ2PushBroom, 2,3,3,3,3,3,3,3,1,1,6,6>* projectionCostFunction =
                    new ceres::AutoDiffCostFunction<ParametrizedXYZ2PushBroom, 2,3,3,3,3,3,3,3,1,1,6,6>(projectionCost);

            Eigen::Matrix<double,2,2> weigthMat = Eigen::Matrix<double,2,2>::Identity();

            weigthMat(0,0) = 1./_tiepointsAccuracy;
            weigthMat(1,1) = 1./_tiepointsAccuracy;

            StereoVisionApp::WeightedCostFunction<2,3,3,3,3,3,3,3,1,1,6,6>* weigthedProjectionCost =
                    new StereoVisionApp::WeightedCostFunction<2,3,3,3,3,3,3,3,1,1,6,6>(projectionCostFunction, weigthMat);

            LandmarkPos& lmPos = _landmarksParameters[lm_idx];

#ifndef NDEBUG
            std::array<double,2> residual;
            bool ok = projectionCost->operator()(lmPos.position.data(),
                                     _leverArm.rAxis.data(),
                                     _leverArm.t.data(),
                                     pose.rAxis.data(),
                                     pose.wAxis.data(),
                                     pose.t.data(),
                                     pose.v.data(),
                                     _sensorParameters.fLen.data(),
                                     _sensorParameters.principalPoint.data(),
                                     _sensorParameters.lateralDistortion.data(),
                                     _sensorParameters.frontalDistortion.data(),
                                                 residual.data());

            if (!ok) {
                std::cout << "Misaligned initial poses" << std::endl;
            }
#endif

            _problem.AddResidualBlock(weigthedProjectionCost, nullptr,
                                      lmPos.position.data(),
                                      _leverArm.rAxis.data(),
                                      _leverArm.t.data(),
                                      pose.rAxis.data(),
                                      pose.wAxis.data(),
                                      pose.t.data(),
                                      pose.v.data(),
                                      _sensorParameters.fLen.data(),
                                      _sensorParameters.principalPoint.data(),
                                      _sensorParameters.lateralDistortion.data(),
                                      _sensorParameters.frontalDistortion.data());

            lmViewIdx++;
        }

        if (i > 0) {

            FramePoseParameters & previous_pose = _frameParameters[i-1];

            double p_time = lines_times[i-1];

            previous_pose.v[0] = pose.t[0] - previous_pose.t[0];
            previous_pose.v[1] = pose.t[1] - previous_pose.t[1];
            previous_pose.v[2] = pose.t[2] - previous_pose.t[2];

            Eigen::Vector3d rAxisPrev(previous_pose.rAxis[0], previous_pose.rAxis[1], previous_pose.rAxis[2]);
            Eigen::Vector3d rAxis(pose.rAxis[0], pose.rAxis[1], pose.rAxis[2]);

            Eigen::Matrix3d Rprev = StereoVision::Geometry::rodriguezFormula(rAxisPrev);
            Eigen::Matrix3d R = StereoVision::Geometry::rodriguezFormula(rAxis);

            Eigen::Matrix3d deltaR = R*Rprev.transpose();

            Eigen::Vector3d deltarAxis = StereoVision::Geometry::inverseRodriguezFormula(deltaR);

            previous_pose.wAxis[0] = deltarAxis[0];
            previous_pose.wAxis[1] = deltarAxis[1];
            previous_pose.wAxis[2] = deltarAxis[2];

            StereoVisionApp::ImuStepCost* imuStepCost = StereoVisionApp::ImuStepCost::getIntegratedIMUDiff(GyroMeasurements,
                                                                                                           AccMeasurements,
                                                                                                           p_time,
                                                                                                           time);

            ceres::AutoDiffCostFunction<StereoVisionApp::ImuStepCost, 9,3,3,3,3,3,3,3>* imuStepCostFunction =
                    new ceres::AutoDiffCostFunction<StereoVisionApp::ImuStepCost, 9,3,3,3,3,3,3,3>(imuStepCost);

            double dt = time - p_time;
            Eigen::Matrix<double,9,9> weigthMat = Eigen::Matrix<double,9,9>::Identity();

            double speedUncertainty = _accAccuracy*dt;
            double posUncertainty = _accAccuracy*dt*dt/2;
            double poseUncertainty = _gyroAccuracy*dt;

            weigthMat(0,0) = 1/poseUncertainty;
            weigthMat(1,1) = 1/poseUncertainty;
            weigthMat(2,2) = 1/poseUncertainty;

            weigthMat(3,3) = 1/posUncertainty;
            weigthMat(4,4) = 1/posUncertainty;
            weigthMat(5,5) = 1/posUncertainty;

            weigthMat(6,6) = 1/speedUncertainty;
            weigthMat(7,7) = 1/speedUncertainty;
            weigthMat(8,8) = 1/speedUncertainty;

            StereoVisionApp::WeightedCostFunction<9,3,3,3,3,3,3,3>* weigthedImuStepCost =
                    new StereoVisionApp::WeightedCostFunction<9,3,3,3,3,3,3,3>(imuStepCostFunction, weigthMat);

            _problem.AddResidualBlock(weigthedImuStepCost, nullptr,
                                      previous_pose.rAxis.data(),
                                      previous_pose.t.data(),
                                      previous_pose.v.data(),
                                      pose.rAxis.data(),
                                      pose.t.data(),
                                      pose.v.data(),
                                      _gravity.vec.data());

            OrientationSpeedConsistency* orientationSpeedConsistency = new OrientationSpeedConsistency(dt);

            ceres::AutoDiffCostFunction<OrientationSpeedConsistency, 3,3,3,3>* orientationSpeedConsistencyCostFunction =
                    new ceres::AutoDiffCostFunction<OrientationSpeedConsistency, 3,3,3,3>(orientationSpeedConsistency);

            Eigen::Matrix<double,3,3> orientationStiffness = Eigen::Matrix<double,3,3>::Identity();

            orientationStiffness(0,0) = poseUncertainty;
            orientationStiffness(1,1) = poseUncertainty;
            orientationStiffness(2,2) = poseUncertainty;

            StereoVisionApp::WeightedCostFunction<3,3,3,3>* weigthedOrientationStepCost =
                    new StereoVisionApp::WeightedCostFunction<3,3,3,3>(orientationSpeedConsistencyCostFunction, orientationStiffness);

            _problem.AddResidualBlock(weigthedOrientationStepCost, nullptr,
                                      previous_pose.rAxis.data(),
                                      previous_pose.wAxis.data(),
                                      pose.rAxis.data());

        }

    }

    _not_first_step = false;

    return true;

}

bool CeresPushBroomSolver::opt_step() {

    std::cout << "Entering opt step function" << std::endl;

    if (_not_first_step) {
        return true; //optimization already converged
    }

    ceres::Solver::Options options;
    options.linear_solver_type = (_sparse) ? ceres::SPARSE_SCHUR : ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    CeresPushBroomSolverIterationCallback* iterationCallBack = new CeresPushBroomSolverIterationCallback(this);

    options.max_num_iterations = optimizationSteps();

    options.callbacks = {iterationCallBack};

    ceres::Solver::Summary summary;

    try {
        ceres::Solve(options, &_problem, &summary);
    } catch (std::exception const& e) {
        std::cout << e.what() << std::endl;
        return false;
    }

    std::cout << summary.BriefReport() << std::endl;

    _not_first_step = true;

    return true;
}
bool CeresPushBroomSolver::std_step() {

    std::cout << "Entering std step function" << std::endl;
    return true;
}
bool CeresPushBroomSolver::writeResults() {

    std::cout << "Entering write results function" << std::endl;

    if (_currentProject == nullptr) {
        return false;
    }

    if (_resultsDataTable == nullptr) {
        qint64 id = _currentProject->createDataBlock(StereoVisionApp::DataTable::staticMetaObject.className());

        if (id <= 0) {
            return false;
        }

        _resultsDataTable = _currentProject->getDataBlock<StereoVisionApp::DataTable>(id);
        _resultsDataTable->setObjectName("Pushbroom trajectory optimization results");
    }

    QVector<QVariant> finalTimes;

    QVector<QVariant> initialTrajPosX;
    QVector<QVariant> initialTrajPosY;
    QVector<QVariant> initialTrajPosZ;

    QVector<QVariant> initialTrajOrientX;
    QVector<QVariant> initialTrajOrientY;
    QVector<QVariant> initialTrajOrientZ;

    QVector<QVariant> finalTrajPosX;
    QVector<QVariant> finalTrajPosY;
    QVector<QVariant> finalTrajPosZ;

    QVector<QVariant> finalTrajOrientX;
    QVector<QVariant> finalTrajOrientY;
    QVector<QVariant> finalTrajOrientZ;

    QVector<QVariant> gtTrajPosX;
    QVector<QVariant> gtTrajPosY;
    QVector<QVariant> gtTrajPosZ;

    QVector<QVariant> gtTrajOrientX;
    QVector<QVariant> gtTrajOrientY;
    QVector<QVariant> gtTrajOrientZ;

    finalTimes.reserve(_frameParameters.size());

    initialTrajPosX.reserve(_frameParameters.size());
    initialTrajPosY.reserve(_frameParameters.size());
    initialTrajPosZ.reserve(_frameParameters.size());

    initialTrajOrientX.reserve(_frameParameters.size());
    initialTrajOrientY.reserve(_frameParameters.size());
    initialTrajOrientZ.reserve(_frameParameters.size());

    finalTrajPosX.reserve(_frameParameters.size());
    finalTrajPosY.reserve(_frameParameters.size());
    finalTrajPosZ.reserve(_frameParameters.size());

    finalTrajOrientX.reserve(_frameParameters.size());
    finalTrajOrientY.reserve(_frameParameters.size());
    finalTrajOrientZ.reserve(_frameParameters.size());

    gtTrajPosX.reserve(_frameParameters.size());
    gtTrajPosY.reserve(_frameParameters.size());
    gtTrajPosZ.reserve(_frameParameters.size());

    gtTrajOrientX.reserve(_frameParameters.size());
    gtTrajOrientY.reserve(_frameParameters.size());
    gtTrajOrientZ.reserve(_frameParameters.size());

    for (FramePoseParameters const& pose : _frameParameters) {

        finalTimes.push_back(pose.time);

        initialTrajPosX.push_back(pose.tInitial[0]);
        initialTrajPosY.push_back(pose.tInitial[1]);
        initialTrajPosZ.push_back(pose.tInitial[2]);

        initialTrajOrientX.push_back(pose.rAxisInitial[0]);
        initialTrajOrientY.push_back(pose.rAxisInitial[1]);
        initialTrajOrientZ.push_back(pose.rAxisInitial[2]);

        finalTrajPosX.push_back(pose.t[0]);
        finalTrajPosY.push_back(pose.t[1]);
        finalTrajPosZ.push_back(pose.t[2]);

        finalTrajOrientX.push_back(pose.rAxis[0]);
        finalTrajOrientY.push_back(pose.rAxis[1]);
        finalTrajOrientZ.push_back(pose.rAxis[2]);

        if (pose.tGt.has_value()) {
            gtTrajPosX.push_back(pose.tGt.value()[0]);
            gtTrajPosY.push_back(pose.tGt.value()[1]);
            gtTrajPosZ.push_back(pose.tGt.value()[2]);
        } else {
            gtTrajPosX.push_back(QVariant());
            gtTrajPosY.push_back(QVariant());
            gtTrajPosZ.push_back(QVariant());
        }

        if (pose.rAxisGt.has_value()) {
            gtTrajOrientX.push_back(pose.rAxisGt.value()[0]);
            gtTrajOrientY.push_back(pose.rAxisGt.value()[1]);
            gtTrajOrientZ.push_back(pose.rAxisGt.value()[2]);
        } else {
            gtTrajOrientX.push_back(QVariant());
            gtTrajOrientY.push_back(QVariant());
            gtTrajOrientZ.push_back(QVariant());
        }
    }

    QMap<QString, QVector<QVariant>> data;

    data.insert("Time", finalTimes);

    data.insert("Initial trajectory X coordinate", initialTrajPosX);
    data.insert("Initial trajectory Y coordinate", initialTrajPosY);
    data.insert("Initial trajectory Z coordinate", initialTrajPosZ);

    data.insert("Initial trajectory X orientation", initialTrajOrientX);
    data.insert("Initial trajectory Y orientation", initialTrajOrientY);
    data.insert("Initial trajectory Z orientation", initialTrajOrientZ);

    data.insert("Trajectory X coordinate", finalTrajPosX);
    data.insert("Trajectory Y coordinate", finalTrajPosY);
    data.insert("Trajectory Z coordinate", finalTrajPosZ);

    data.insert("Trajectory X orientation", finalTrajOrientX);
    data.insert("Trajectory Y orientation", finalTrajOrientY);
    data.insert("Trajectory Z orientation", finalTrajOrientZ);

    data.insert("GT trajectory X coordinate", gtTrajPosX);
    data.insert("GT trajectory Y coordinate", gtTrajPosY);
    data.insert("GT trajectory Z coordinate", gtTrajPosZ);

    data.insert("GT trajectory X orientation", gtTrajOrientX);
    data.insert("GT trajectory Y orientation", gtTrajOrientY);
    data.insert("GT trajectory Z orientation", gtTrajOrientZ);

    _resultsDataTable->setData(data);

    return true;

}
bool CeresPushBroomSolver::writeUncertainty() {

    std::cout << "Entering write uncertainty function" << std::endl;
    return true;
}
void CeresPushBroomSolver::cleanup() {

    std::cout << "Entering cleanup function" << std::endl;

    _landmarksParameters.clear();
    _landmarkParametersIndex.clear();

    _frameParameters.clear();
    _frameParametersIndex.clear();

    _problem = ceres::Problem(); //reset problem

}

bool CeresPushBroomSolver::splitOptSteps() const {
    return true;
}

void CeresPushBroomSolver::setResultsDataTable(StereoVisionApp::DataTable *newResultsDataTable)
{
    _resultsDataTable = newResultsDataTable;
}

double CeresPushBroomSolver::tiepointsAccuracy() const
{
    return _tiepointsAccuracy;
}

void CeresPushBroomSolver::setTiepointsAccuracy(double newTiepointsAccuracy)
{
    _tiepointsAccuracy = newTiepointsAccuracy;
}

double CeresPushBroomSolver::accAccuracy() const
{
    return _accAccuracy;
}

void CeresPushBroomSolver::setAccAccuracy(double newAccAccuracy)
{
    _accAccuracy = newAccAccuracy;
}

double CeresPushBroomSolver::gyroAccuracy() const
{
    return _gyroAccuracy;
}

void CeresPushBroomSolver::setGyroAccuracy(double newGyroAccuracy)
{
    _gyroAccuracy = newGyroAccuracy;
}

double CeresPushBroomSolver::sensorWidth() const
{
    return _sensorWidth;
}

void CeresPushBroomSolver::setSensorWidth(double newSensorWidth)
{
    _sensorWidth = newSensorWidth;
}

double CeresPushBroomSolver::gpsAccuracy() const
{
    return _gpsAccuracy;
}

void CeresPushBroomSolver::setGpsAccuracy(double newGpsAccuracy)
{
    _gpsAccuracy = newGpsAccuracy;
}

QVector<CeresPushBroomSolver::viewInfos> CeresPushBroomSolver::AdditionalViewsInfos::compilePoints() {

    if (dataTable == nullptr) {
        return QVector<CeresPushBroomSolver::viewInfos>();
    }

    QVector<QVariant> lmIdxs = dataTable->getColumnData(colLmId);
    QVector<QVariant> ptIdxs = dataTable->getColumnData(colPtId);
    QVector<QVariant> posx = dataTable->getColumnData(colPosX);
    QVector<QVariant> posy = dataTable->getColumnData(colPosY);

    int nPoints = lmIdxs.size();

    QVector<CeresPushBroomSolver::viewInfos> ret(nPoints);

    for (int i = 0; i < nPoints; i++) {

        bool ok;
        int lmid = lmIdxs[i].toInt(&ok);

        ret[i].lmId = (ok) ? lmid : -1;

        int ptid = ptIdxs[i].toInt(&ok);

        ret[i].additionalPtId = (ok) ? ptid : -1;

        ret[i].x = posx[i].toFloat();
        ret[i].y = posy[i].toFloat();
    }

    return ret;
}

QVector<CeresPushBroomSolver::pointInfos> CeresPushBroomSolver::AdditionalPointsInfos::compilePoints() {

    if (dataTable == nullptr) {
        return QVector<CeresPushBroomSolver::pointInfos>();
    }

    QVector<QVariant> ptIdxs = dataTable->getColumnData(colPtId);
    QVector<QVariant> posx = dataTable->getColumnData(colPosX);
    QVector<QVariant> posy = dataTable->getColumnData(colPosY);
    QVector<QVariant> posz = dataTable->getColumnData(colPosZ);

    int nPoints = ptIdxs.size();

    QVector<CeresPushBroomSolver::pointInfos> ret;
    ret.reserve(nPoints);

    for (int i = 0; i < nPoints; i++) {

        bool ok;

        int ptid = ptIdxs[i].toInt(&ok);

        if (!ok) {
            continue;
        }

        ret[i].additionalPtId = ptid;

        ret[i].x = posx[i].toFloat();
        ret[i].y = posy[i].toFloat();
        ret[i].z = posz[i].toFloat();
    }

    return ret;

}

std::optional<QString> CeresPushBroomSolver::GPSMeasurementInfos::configureFromColumnsSelection(DataColumnsSelectionWidget::ColumnSelectionInfos const& colSelectionInfos) {

    if (colSelectionInfos.dataTable == nullptr) {
        return QObject::tr("Missing GPS data");
    }

    bool validGPSdata = true;
    QVector<QString> gpsDataColumns = colSelectionInfos.dataTable->columns();

    if (colSelectionInfos.variableType != DataColumnsSelectionWidget::Position or
            !gpsDataColumns.contains(colSelectionInfos.indexingCol) or
            colSelectionInfos.columnsNames.size() != 3) {
        validGPSdata = false;
    }

    if (validGPSdata) {
        for (QString const& colName : colSelectionInfos.columnsNames) {
            if (!gpsDataColumns.contains(colName)) {
                validGPSdata = false;
            }
        }
    }

    if (!validGPSdata) {
        return QObject::tr("Invalid GPS data");
    }

    dataTable = colSelectionInfos.dataTable;
    colTime = colSelectionInfos.indexingCol;
    colPosX = colSelectionInfos.columnsNames[0];
    colPosY = colSelectionInfos.columnsNames[1];
    colPosZ = colSelectionInfos.columnsNames[2];

    return std::nullopt;
}

CeresPushBroomSolver::IndexedVec3TimeSeq CeresPushBroomSolver::GPSMeasurementInfos::compileSequence() {

    if (dataTable == nullptr) {
        return CeresPushBroomSolver::IndexedVec3TimeSeq();
    }

    QVector<QVariant> time = dataTable->getColumnData(colTime);
    QVector<QVariant> posx = dataTable->getColumnData(colPosX);
    QVector<QVariant> posy = dataTable->getColumnData(colPosY);
    QVector<QVariant> posz = dataTable->getColumnData(colPosZ);

    using ptT = CeresPushBroomSolver::IndexedVec3TimeSeq::TimedElement;

    std::vector<ptT> sequence(time.size());

    for (int i = 0; i < time.size(); i++) {
        sequence[i].time = time[i].toFloat();
        sequence[i].val = {posx[i].toFloat(), posy[i].toFloat(), posz[i].toFloat()};
    }

    return CeresPushBroomSolver::IndexedVec3TimeSeq(std::move(sequence));
}

std::optional<QString> CeresPushBroomSolver::InitialOrientationInfos::configureFromColumnsSelection(DataColumnsSelectionWidget::ColumnSelectionInfos const& colSelectionInfos) {

    if (colSelectionInfos.dataTable == nullptr) {
        return QObject::tr("Missing Initial orientation data");
    }

    bool validInitialOrientationdata = true;
    QVector<QString> initialOrientationDataColumns = colSelectionInfos.dataTable->columns();

    if (colSelectionInfos.variableType != DataColumnsSelectionWidget::Orientation or
            !initialOrientationDataColumns.contains(colSelectionInfos.indexingCol)) {
        validInitialOrientationdata = false;
    }

    if ((colSelectionInfos.angleRepType != DataColumnsSelectionWidget::Quaternion and colSelectionInfos.columnsNames.size() != 3)) {
        validInitialOrientationdata = false;
    }

    if ((colSelectionInfos.angleRepType == DataColumnsSelectionWidget::Quaternion and colSelectionInfos.columnsNames.size() != 4)) {
        validInitialOrientationdata = false;
    }

    if (validInitialOrientationdata) {
        for (QString const& colName : colSelectionInfos.columnsNames) {
            if (!initialOrientationDataColumns.contains(colName)) {
                validInitialOrientationdata = false;
            }
        }
    }

    if (!validInitialOrientationdata) {
        return QObject::tr("Invalid Initial orientation data");
    }

    dataTable = colSelectionInfos.dataTable;
    colTime = colSelectionInfos.indexingCol;

    if (colSelectionInfos.angleRepType == DataColumnsSelectionWidget::Quaternion) {
        rotationRepresentation = CeresPushBroomSolver::Quaternion;
        colRotW = colSelectionInfos.columnsNames[0];
        colRotX = colSelectionInfos.columnsNames[1];
        colRotY = colSelectionInfos.columnsNames[2];
        colRotZ = colSelectionInfos.columnsNames[3];
    }

    if (colSelectionInfos.angleRepType == DataColumnsSelectionWidget::AxisAngle) {
        rotationRepresentation = CeresPushBroomSolver::AngleAxis;
        colRotX = colSelectionInfos.columnsNames[0];
        colRotY = colSelectionInfos.columnsNames[1];
        colRotZ = colSelectionInfos.columnsNames[2];
    }

    if (colSelectionInfos.angleRepType == DataColumnsSelectionWidget::EulerXYZ) {
        rotationRepresentation = CeresPushBroomSolver::EulerXYZ;
        colRotX = colSelectionInfos.columnsNames[0];
        colRotY = colSelectionInfos.columnsNames[1];
        colRotZ = colSelectionInfos.columnsNames[2];
    }

    return std::nullopt;
}

CeresPushBroomSolver::IndexedVec3TimeSeq CeresPushBroomSolver::InitialOrientationInfos::compileSequence() {

    if (dataTable == nullptr) {
        return CeresPushBroomSolver::IndexedVec3TimeSeq();
    }

    QVector<QVariant> time = dataTable->getColumnData(colTime);
    QVector<QVariant> rotw = dataTable->getColumnData(colRotW);
    QVector<QVariant> rotx = dataTable->getColumnData(colRotX);
    QVector<QVariant> roty = dataTable->getColumnData(colRotY);
    QVector<QVariant> rotz = dataTable->getColumnData(colRotZ);

    using ptT = CeresPushBroomSolver::IndexedVec3TimeSeq::TimedElement;

    std::vector<ptT> sequence(time.size());

    for (int i = 0; i < time.size(); i++) {
        sequence[i].time = time[i].toFloat();

        switch (rotationRepresentation) {
        case AngleAxis:

            sequence[i].val = {rotx[i].toFloat(), roty[i].toFloat(), rotz[i].toFloat()};
            break;

        case Quaternion:
        {
            Eigen::AngleAxis<double> axis;
            axis = Eigen::Quaternion<double>(rotw[i].toFloat(), rotx[i].toFloat(), roty[i].toFloat(), rotz[i].toFloat());

            Eigen::Vector3d raxis = axis.axis()*axis.angle();
            sequence[i].val = {static_cast<float>(raxis.x()), static_cast<float>(raxis.y()), static_cast<float>(raxis.z())};
        }
            break;

        case EulerXYZ:
        {
            Eigen::AngleAxis<double> axis;
            axis = Eigen::AngleAxis<double>(rotz[i].toFloat(), Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxis<double>(roty[i].toFloat(), Eigen::Vector3d::UnitY())*
            Eigen::AngleAxis<double>(rotx[i].toFloat(), Eigen::Vector3d::UnitX());

            Eigen::Vector3d raxis = axis.axis()*axis.angle();
            sequence[i].val = {static_cast<float>(raxis.x()), static_cast<float>(raxis.y()), static_cast<float>(raxis.z())};
        }
            break;
        }
    }

    return CeresPushBroomSolver::IndexedVec3TimeSeq(std::move(sequence));
}

CeresPushBroomSolver::IndexedVec3TimeSeq CeresPushBroomSolver::AccMeasurementInfos::compileSequence() {

    if (dataTable == nullptr) {
        return CeresPushBroomSolver::IndexedVec3TimeSeq();
    }

    QVector<QVariant> time = dataTable->getColumnData(colTime);
    QVector<QVariant> accx = dataTable->getColumnData(colAccX);
    QVector<QVariant> accy = dataTable->getColumnData(colAccY);
    QVector<QVariant> accz = dataTable->getColumnData(colAccZ);

    using ptT = CeresPushBroomSolver::IndexedVec3TimeSeq::TimedElement;

    std::vector<ptT> sequence(time.size());

    for (int i = 0; i < time.size(); i++) {
        sequence[i].time = time[i].toFloat();
        sequence[i].val = {accx[i].toFloat(), accy[i].toFloat(), accz[i].toFloat()};
    }

    return CeresPushBroomSolver::IndexedVec3TimeSeq(std::move(sequence));
}

CeresPushBroomSolver::IndexedVec3TimeSeq CeresPushBroomSolver::GyroMeasurementInfos::compileSequence() {

    if (dataTable == nullptr) {
        return CeresPushBroomSolver::IndexedVec3TimeSeq();
    }

    QVector<QVariant> time = dataTable->getColumnData(colTime);
    QVector<QVariant> asw = dataTable->getColumnData(colAngSpeedW);
    QVector<QVariant> asx = dataTable->getColumnData(colAngSpeedX);
    QVector<QVariant> asy = dataTable->getColumnData(colAngSpeedY);
    QVector<QVariant> asz = dataTable->getColumnData(colAngSpeedZ);

    using ptT = CeresPushBroomSolver::IndexedVec3TimeSeq::TimedElement;

    std::vector<ptT> sequence(time.size());

    for (int i = 0; i < time.size(); i++) {
        sequence[i].time = time[i].toFloat();

        switch (rotationRepresentation) {
        case AngleAxis:

            sequence[i].val = {asx[i].toFloat(), asy[i].toFloat(), asz[i].toFloat()};
            break;

        case Quaternion:
        {
            Eigen::AngleAxis<double> axis;
            axis = Eigen::Quaternion<double>(asw[i].toFloat(), asx[i].toFloat(), asy[i].toFloat(), asz[i].toFloat());

            Eigen::Vector3d raxis = axis.axis()*axis.angle();
            sequence[i].val = {static_cast<float>(raxis.x()), static_cast<float>(raxis.y()), static_cast<float>(raxis.z())};
        }
            break;

        case EulerXYZ:
        {
            Eigen::AngleAxis<double> axis;
            axis = Eigen::AngleAxis<double>(asz[i].toFloat(), Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxis<double>(asy[i].toFloat(), Eigen::Vector3d::UnitY())*
            Eigen::AngleAxis<double>(asx[i].toFloat(), Eigen::Vector3d::UnitX());

            Eigen::Vector3d raxis = axis.axis()*axis.angle();
            sequence[i].val = {static_cast<float>(raxis.x()), static_cast<float>(raxis.y()), static_cast<float>(raxis.z())};
        }
            break;
        }
    }

    return CeresPushBroomSolver::IndexedVec3TimeSeq(std::move(sequence));
}

} // namespace PikaLTools
