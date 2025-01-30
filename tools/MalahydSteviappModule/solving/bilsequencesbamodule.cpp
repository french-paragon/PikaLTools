#include "bilsequencesbamodule.h"

#include "datablocks/bilacquisitiondata.h"

#include <steviapp/datablocks/cameras/pushbroompinholecamera.h>

#include <steviapp/sparsesolver/sbamodules/pinholecameraprojectormodule.h>

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

        //check if a previous module assigned a projection module already to the frame.
        StereoVisionApp::ModularSBASolver::ProjectorModule* projectionModule = solver->getProjectorForFrame(seqId);

        if (projectionModule == nullptr) {

            StereoVisionApp::PushBroomPinholeCamera* cam = seq->getAssignedCamera();

            if (cam == nullptr) {
                continue;
            }

            projectionModule = solver->getProjectorForCamera(cam->internalId());

            if (projectionModule == nullptr) {
                StereoVisionApp::PinholePushBroomCamProjectorModule* pcpm = new StereoVisionApp::PinholePushBroomCamProjectorModule(cam);
                projectionModule = pcpm;

                solver->addProjector(projectionModule);
                solver->assignProjectorToCamera(projectionModule, cam->internalId());
            }
            solver->assignProjectorToFrame(projectionModule, seqId);
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

            parameters.seqId = seq->internalId();
            parameters.sensorId = seqSensorIndex;

            parameters.tLeverArm = {0,0,0};
            parameters.rLeverArm = {0,0,0};

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

        solver->addLogger(QString("Intrisic for BilSensor%1 - Lever arm").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<3>(sensorParameters.tLeverArm.data()));

        solver->addLogger(QString("Intrisic for BilSensor%1 - Boresight").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::ParamsValsLogger<3>(sensorParameters.rLeverArm.data()));

        //priors

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(sensorParameters.seqId);

        //lever arm

        Eigen::Matrix3d leverArmStiffness = Eigen::Matrix3d::Identity();
        Eigen::Vector3d leverArmPrior = Eigen::Vector3d::Zero();

        leverArmStiffness(0,0) = 10;
        leverArmStiffness(1,1) = 10;
        leverArmStiffness(2,2) = 10;

        if (seq != nullptr) {

            auto leverArmX = seq->xCoord();
            auto leverArmY = seq->yCoord();
            auto leverArmZ = seq->zCoord();

            if (leverArmX.isSet() and leverArmX.isUncertain() and
                    leverArmY.isSet() and leverArmY.isUncertain() and
                    leverArmZ.isSet() and leverArmZ.isUncertain()) {

                leverArmPrior.x() = leverArmX.value();
                leverArmPrior.y() = leverArmY.value();
                leverArmPrior.z() = leverArmZ.value();

                leverArmStiffness(0,0) = 1/leverArmX.stddev();
                leverArmStiffness(1,1) = 1/leverArmY.stddev();
                leverArmStiffness(2,2) = 1/leverArmZ.stddev();

                solver->logMessage(QString("Set hyperspectral sensor %1 lever arm prior stiffness %2, %3, %4")
                                   .arg(sensorParameters.sensorId).arg(leverArmStiffness(0,0)).arg(leverArmStiffness(1,1)).arg(leverArmStiffness(2,2)));
            }

        }

        ceres::NormalPrior* leverArmPriorCost = new ceres::NormalPrior(leverArmStiffness, leverArmPrior);

        problem.AddResidualBlock(leverArmPriorCost, nullptr, sensorParameters.tLeverArm.data());

        solver->addLogger(QString("Lever arm prior for BilSensor%1").arg(sensorParameters.sensorId),
                          new StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<1, 3>(leverArmPriorCost, {sensorParameters.tLeverArm.data()}, false));

    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

    for (qint64 seqId : bilSeqsIdxs) {

        BilSequenceAcquisitionData* seq = currentProject->getDataBlock<BilSequenceAcquisitionData>(seqId);

        if (seq == nullptr) {
            continue;
        }

        StereoVisionApp::ModularSBASolver::ProjectorModule* projectionModule = solver->getProjectorForFrame(seqId);

        if (projectionModule == nullptr) {
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



                QString lmName = "Fantom landmark";
                if (lm != nullptr) {
                    lmName = lm->objectName();
                }

                projectionModule->addProjectionCostFunction(lmNode->pos.data(),
                                                            closest.rAxis.data(),
                                                            closest.t.data(),
                                                            uv,
                                                            info,
                                                            problem,
                                                            measure2node,
                                                            sensorParameters.rLeverArm.data(),
                                                            sensorParameters.tLeverArm.data());


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
