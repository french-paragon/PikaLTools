#include "bilsequencesbamodule.h"

#include "datablocks/bilacquisitiondata.h"

#include <steviapp/sparsesolver/costfunctors/modularuvprojection.h>
#include <steviapp/sparsesolver/costfunctors/posedecoratorfunctors.h>

#include "cost_functors/pinholepushbroomuvprojector.h"

#include <ceres/normal_prior.h>

namespace PikaLTools {

const char* BilSequenceSBAModule::ModuleName = "PikaLTools::SBAModule::BilSequence";

using PushBroomUVProj = PinholePushbroomUVProjector;
using PushBroomUVCostInterpolated =
StereoVisionApp::InterpolatedPose< //need to interpolate
StereoVisionApp::LeverArm< //need to add lever arm
StereoVisionApp::InvertPose< //UV2ParametrizedXYZCost takes mapping2sensor but we parametrize the trajectory as body2mapping
StereoVisionApp::UV2ParametrizedXYZCost<PushBroomUVProj,1,1,6,6>
>,
StereoVisionApp::Body2World | StereoVisionApp::Body2Sensor
>
>;
using PushBroomUVCostTransformed =
StereoVisionApp::PoseTransform< //or use a rigid transform to the nearest node
StereoVisionApp::LeverArm< //need to add lever arm
StereoVisionApp::InvertPose< //UV2ParametrizedXYZCost takes mapping2sensor but we parametrize the trajectory as body2mapping
StereoVisionApp::UV2ParametrizedXYZCost<PushBroomUVProj,1,1,6,6>
>,
StereoVisionApp::Body2World | StereoVisionApp::Body2Sensor
>,
StereoVisionApp::PoseTransformDirection::SourceToInitial
>;

BilSequenceSBAModule::BilSequenceSBAModule()
{

}

bool BilSequenceSBAModule::addGraphReductorVariables(StereoVisionApp::Project *currentProject,
                                                     StereoVisionApp::GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr or graphReductor == nullptr) {
        return false;
    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    for (qint64 seqId : bilSeqsIdxs) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        graphReductor->insertItem(seqId, 0);
    }

    return true;
}

bool BilSequenceSBAModule::addGraphReductorObservations(StereoVisionApp::Project *currentProject,
                                                        StereoVisionApp::GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr or graphReductor == nullptr) {
        return false;
    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    for (qint64 seqId : bilSeqsIdxs) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        QVector<qint64> imlmids = seq->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

        for (qint64 imlmid : imlmids) {

            BilSequenceLandmark* blm = seq->getBilSequenceLandmark(imlmid);

            qint64 lmid = blm->attachedLandmarkid();

            graphReductor->insertObservation(seqId, lmid, 2);
        }
    }

    return true;
}

bool BilSequenceSBAModule::setupParameters(StereoVisionApp::ModularSBASolver* solver) {

    _sensorsParameters.clear();
    _sensorParametersIndex.clear();
    _sensorIndexMap.clear();

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    _sensorsParameters.reserve(bilSeqsIdxs.size());

    for (qint64 seqId : bilSeqsIdxs) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        qint64 trajId = seq->assignedTrajectory();

        StereoVisionApp::ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, false);

        //no trajectory node
        if (trajNode == nullptr) {
            continue;
        }

        QMap<QString, QString> sequence_header = seq->getBilInfos()[0].headerData();

        double sensorWidth = sequence_header.value("samples").toDouble();

        double fov = sequence_header.value("field of view").toDouble();
        double fov_rad = fov*M_PI/180.;

        int seqSensorIndex = seq->sensorIndex();

        if (seqSensorIndex < 0 or !_sensorIndexMap.contains(seqSensorIndex)) {

            BilCameraParameters parameters;

            parameters.sensorId = seqSensorIndex;

            parameters.tLeverArm = {0,0,0};
            parameters.rLeverArm = {0,0,0};

            parameters.fLen[0] = sensorWidth*std::tan(M_PI_2-fov_rad/2.)/2.;
            parameters.principalPoint[0] = sensorWidth/2;
            parameters.frontalDistortion = {0,0,0,0,0,0};
            parameters.lateralDistortion = {0,0,0,0,0,0};

            _sensorsParameters.push_back(parameters);
            int pos = _sensorsParameters.size()-1;
            _sensorParametersIndex.insert(seqId, pos);

            _sensorIndexMap.insert(seqSensorIndex, pos);

        } else {
            //set the sensor to the parameters already set for the previous line with the same sensor id.
            _sensorParametersIndex.insert(seqId, _sensorIndexMap[seqSensorIndex]);
        }

    }

    return true;
}

bool BilSequenceSBAModule::init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    for (BilCameraParameters& sensorParameters : _sensorsParameters) {

        problem.AddParameterBlock(sensorParameters.tLeverArm.data(), sensorParameters.tLeverArm.size());
        problem.AddParameterBlock(sensorParameters.rLeverArm.data(), sensorParameters.rLeverArm.size());

        problem.AddParameterBlock(sensorParameters.fLen.data(), sensorParameters.fLen.size());
        problem.AddParameterBlock(sensorParameters.principalPoint.data(), sensorParameters.principalPoint.size());
        problem.AddParameterBlock(sensorParameters.frontalDistortion.data(), sensorParameters.frontalDistortion.size());
        problem.AddParameterBlock(sensorParameters.lateralDistortion.data(), sensorParameters.lateralDistortion.size());

        solver->addLogger(QString("Intrisic for BilSensor%1 - Lever arm").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<3>(sensorParameters.tLeverArm.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Boresight").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<3>(sensorParameters.rLeverArm.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Focal lenght").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<1>(sensorParameters.fLen.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Principal point").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<1>(sensorParameters.principalPoint.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Frontal distortion").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<6>(sensorParameters.frontalDistortion.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Lateral distortion").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<6>(sensorParameters.lateralDistortion.data()));

        //priors

        //lever arm
        Eigen::Matrix3d leverArmStiffness = Eigen::Matrix3d::Identity();

        leverArmStiffness(0,0) = 0.1;
        leverArmStiffness(1,1) = 0.1;
        leverArmStiffness(2,2) = 0.1;

        ceres::NormalPrior* leverArmPrior = new ceres::NormalPrior(leverArmStiffness, Eigen::Vector3d::Zero());

        problem.AddResidualBlock(leverArmPrior, nullptr, sensorParameters.tLeverArm.data());

        Eigen::Matrix<double,6,6> distortionStiffness = Eigen::Matrix<double,6,6>::Identity();

        distortionStiffness(0,0) = 1;
        distortionStiffness(1,1) = 1;
        distortionStiffness(2,2) = 0.001;
        distortionStiffness(3,3) = 0.01;
        distortionStiffness(4,4) = 0.2;
        distortionStiffness(5,5) = 0.5; //larger cooefficients are expected to be smaller

        ceres::NormalPrior* distortionPrior = new ceres::NormalPrior(distortionStiffness, Eigen::Matrix<double,6,1>::Zero());

        problem.AddResidualBlock(distortionPrior, nullptr, sensorParameters.frontalDistortion.data());
        problem.AddResidualBlock(distortionPrior, nullptr, sensorParameters.lateralDistortion.data());

    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    for (qint64 seqId : bilSeqsIdxs) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        qint64 trajId = seq->assignedTrajectory();

        StereoVisionApp::ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, false);

        //no trajectory node
        if (trajNode == nullptr) {
            continue;
        }

        QMap<QString, QString> sequence_header = seq->getBilInfos()[0].headerData();

        double sensorWidth = sequence_header.value("samples").toDouble();

        int paramIdxs = _sensorParametersIndex[seqId];
        BilCameraParameters& sensorParameters = _sensorsParameters[paramIdxs];

        QVector<qint64> imlmids = seq->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

        for (qint64 imlmid : imlmids) {

            BilSequenceLandmark* blm = seq->getBilSequenceLandmark(imlmid);

            qint64 lmid = blm->attachedLandmarkid();

            StereoVisionApp::Landmark* lm = currentProject->getDataBlock<StereoVisionApp::Landmark>(lmid);

            if (lm == nullptr) {

                QString message = QObject::tr("[Warning] Bil landmark %3 attached lanmark id (%4) in bil sequence %1 (\"%2\") is not a landmark in project, skipping!")
                        .arg(seqId)
                        .arg(seq->objectName())
                        .arg(imlmid)
                        .arg(lmid);

                solver->logMessage(message);
                continue;
            }

            if (!lm->optPos().isSet()) {
                continue; //skip uninitialized landmarks.
            }

            StereoVisionApp::ModularSBASolver::PositionNode* lmNode = solver->getPositionNode(lmid);

            if (lmNode == nullptr) {

                QString message = QObject::tr("[Warning] Could not get node for landmark %3 (%4)(\"%5\") in bil sequence %1 (\"%2\"), skipping!")
                        .arg(seqId)
                        .arg(seq->objectName())
                        .arg(imlmid)
                        .arg(lmid)
                        .arg(lm->objectName());

                solver->logMessage(message);
                continue;
            }

            Eigen::Vector2d uv(blm->x().value(), 0);

            double time = seq->getTimeFromPixCoord(blm->y().value());

            int trajNodeId = trajNode->getNodeForTime(time);

            if (trajNodeId < 0 or trajNodeId+1 >= trajNode->nodes.size()) {

                QString message = QObject::tr("[Warning] Could not get timing for bil sequence %1 (\"%2\"), bil landmark %3 (%4)(\"%5\"), skipping!")
                        .arg(seqId)
                        .arg(seq->objectName())
                        .arg(imlmid)
                        .arg(lmid)
                        .arg(lm->objectName());

                solver->logMessage(message);
                continue;
            }

            StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& previousPose = trajNode->nodes[trajNodeId];
            StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& nextPose = trajNode->nodes[trajNodeId+1];

            Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
            //TODO find a way to build the info matrix for pushbroom.

            if (trajNode->initialTrajectory.nPoints() > 0) {

                StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& closest =
                        (time - previousPose.time < nextPose.time - time) ? previousPose : nextPose;

                auto nodeInitialPose = trajNode->initialTrajectory.getValueAtTime(closest.time);
                auto measureInitialPose = trajNode->initialTrajectory.getValueAtTime(time);

                StereoVision::Geometry::RigidBodyTransform<double> node2worldInitial =
                        StereoVision::Geometry::interpolateRigidBodyTransformOnManifold
                        (nodeInitialPose.weigthLower, nodeInitialPose.valLower,
                         nodeInitialPose.weigthUpper, nodeInitialPose.valUpper);

                StereoVision::Geometry::RigidBodyTransform<double> measure2worldInitial =
                        StereoVision::Geometry::interpolateRigidBodyTransformOnManifold
                        (measureInitialPose.weigthLower, measureInitialPose.valLower,
                         measureInitialPose.weigthUpper, measureInitialPose.valUpper);

                StereoVision::Geometry::RigidBodyTransform<double> measure2node =
                        node2worldInitial.inverse()*measure2worldInitial;

                PushBroomUVCostTransformed* cost =
                        new PushBroomUVCostTransformed(measure2node, new PushBroomUVProj(sensorWidth), uv, info);

                PushBroomUVCostTransformed* error =
                        new PushBroomUVCostTransformed(measure2node, new PushBroomUVProj(sensorWidth), uv, Eigen::Matrix2d::Identity());


                using ceresFunc = ceres::AutoDiffCostFunction<PushBroomUVCostTransformed,2,3,3,3,3,3,1,1,6,6>;
                ceresFunc* costFunc = new ceresFunc(cost);

                ceresFunc* errorFunc = new ceresFunc(error);

                QString lmName = "Fantom landmark";
                if (lm != nullptr) {
                    lmName = lm->objectName();
                }
                QString loggerName = QString("Projection Bil %1 Landmark %2 (transformed)").arg(seq->objectName()).arg(lmName);
                StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<9, 2>::ParamsType params =
                    {closest.rAxis.data(), closest.t.data(),
                     sensorParameters.rLeverArm.data(), sensorParameters.tLeverArm.data(),
                     lmNode->pos.data(),
                     sensorParameters.fLen.data(),
                     sensorParameters.principalPoint.data(),
                     sensorParameters.frontalDistortion.data(),
                     sensorParameters.lateralDistortion.data()
                    };

                solver->addLogger(loggerName, new StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<9, 2>(errorFunc, params, true));

                problem.AddResidualBlock(costFunc, nullptr, params.data(), params.size());


            } else {

                double w1 = (nextPose.time - time)/(nextPose.time - previousPose.time);
                double w2 = (time - previousPose.time)/(nextPose.time - previousPose.time);

                PushBroomUVCostInterpolated* cost =
                        new PushBroomUVCostInterpolated(w1, w2, new PushBroomUVProj(sensorWidth), uv, info);

                PushBroomUVCostInterpolated* error =
                        new PushBroomUVCostInterpolated(w1, w2, new PushBroomUVProj(sensorWidth), uv, Eigen::Matrix2d::Identity());

                using ceresFunc = ceres::AutoDiffCostFunction<PushBroomUVCostInterpolated,2,3,3,3,3,3,3,3,1,1,6,6>;
                ceresFunc* costFunc = new ceresFunc(cost);

                ceresFunc* errorFunc = new ceresFunc(error);

                QString lmName = "Fantom landmark";
                if (lm != nullptr) {
                    lmName = lm->objectName();
                }
                QString loggerName = QString("Projection Bil %1 Landmark %2 (interpolated)").arg(seq->objectName()).arg(lmName);
                StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<11, 2>::ParamsType params =
                    {previousPose.rAxis.data(), previousPose.t.data(),
                     nextPose.rAxis.data(), nextPose.t.data(),
                     sensorParameters.rLeverArm.data(), sensorParameters.tLeverArm.data(),
                     lmNode->pos.data(),
                     sensorParameters.fLen.data(),
                     sensorParameters.principalPoint.data(),
                     sensorParameters.frontalDistortion.data(),
                     sensorParameters.lateralDistortion.data()
                    };

                solver->addLogger(loggerName, new StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<11, 2>(errorFunc, params, true));


                problem.AddResidualBlock(costFunc, nullptr, params.data(), params.size());
            }
        }

    }

    return true;

}
bool BilSequenceSBAModule::writeResults(StereoVisionApp::ModularSBASolver* solver) {


    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QList<qint64> bilSeqList = _sensorParametersIndex.keys();

    for (qint64 seqId : bilSeqList) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        StereoVisionApp::floatParameterGroup<3> optLeverArm;
        StereoVisionApp::floatParameterGroup<3> optBoresight;

        BilCameraParameters& camParams = _sensorsParameters[_sensorParametersIndex[seqId]];

        for (int i = 0; i < 3; i++) {
            optLeverArm.value(i) = camParams.tLeverArm[i];
            optBoresight.value(i) = camParams.rLeverArm[i];
        }
        optLeverArm.setIsSet();
        optBoresight.setIsSet();

        seq->setOptPos(optLeverArm);
        seq->setOptRot(optBoresight);

        seq->setOptimizedFLen(camParams.fLen[0]);
        seq->setOptimizedOpticalCenterX(camParams.principalPoint[0]);

        seq->setOptimizedA0(camParams.lateralDistortion[0]);
        seq->setOptimizedA1(camParams.lateralDistortion[1]);
        seq->setOptimizedA2(camParams.lateralDistortion[2]);
        seq->setOptimizedA3(camParams.lateralDistortion[3]);
        seq->setOptimizedA4(camParams.lateralDistortion[4]);
        seq->setOptimizedA5(camParams.lateralDistortion[5]);

        seq->setOptimizedB0(camParams.frontalDistortion[0]);
        seq->setOptimizedB1(camParams.frontalDistortion[1]);
        seq->setOptimizedB2(camParams.frontalDistortion[2]);
        seq->setOptimizedB3(camParams.frontalDistortion[3]);
        seq->setOptimizedB4(camParams.frontalDistortion[4]);
        seq->setOptimizedB5(camParams.frontalDistortion[5]);

    }

    return true;

}
bool BilSequenceSBAModule::writeUncertainty(StereoVisionApp::ModularSBASolver* solver) {

    return true;

}
void BilSequenceSBAModule::cleanup(StereoVisionApp::ModularSBASolver* solver) {

    _sensorsParameters.clear();
    _sensorParametersIndex.clear();
    _sensorIndexMap.clear();
}

} // namespace PikaLTools
