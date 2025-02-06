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

QString BilSequenceSBAModule::moduleName() const {
    return QObject::tr("Bil Sequence SBA Module");
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

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> bilSeqsIdxs = currentProject->getIdsByClass(BilSequenceAcquisitionData::staticMetaObject.className());

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

        qint64 mountingId = seq->assignedMounting();

        StereoVisionApp::ModularSBASolver::PoseNode* mountingNode = solver->getNodeForMounting(mountingId, true);

        //no mounting node
        if (mountingNode == nullptr) {
            continue;
        }

        QMap<QString, QString> sequence_header = seq->getBilInfos()[0].headerData();

        double sensorWidth = sequence_header.value("samples").toDouble();

        double fov = sequence_header.value("field of view").toDouble();
        double fov_rad = fov*M_PI/180.;

    }

    return true;
}

bool BilSequenceSBAModule::init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
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

        qint64 mountingId = seq->assignedMounting();

        StereoVisionApp::ModularSBASolver::PoseNode* mountingNode = solver->getNodeForMounting(mountingId, true);

        //no mounting node
        if (mountingNode == nullptr) {
            continue;
        }

        QMap<QString, QString> sequence_header = seq->getBilInfos()[0].headerData();

        double sensorWidth = sequence_header.value("samples").toDouble();

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
                                                            mountingNode->rAxis.data(),
                                                            mountingNode->t.data());


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

    return true;

}
bool BilSequenceSBAModule::writeUncertainty(StereoVisionApp::ModularSBASolver* solver) {

    return true;

}
void BilSequenceSBAModule::cleanup(StereoVisionApp::ModularSBASolver* solver) {
}

} // namespace PikaLTools
