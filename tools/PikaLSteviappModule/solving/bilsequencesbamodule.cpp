#include "bilsequencesbamodule.h"

#include "datablocks/bilacquisitiondata.h"

#include "cost_functors/parametrizedxyz2pushbroom.h"

namespace PikaLTools {

BilSequenceSBAModule::BilSequenceSBAModule()
{

}

bool BilSequenceSBAModule::init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) {

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

            parameters.tLeverArm = {0,0,0};
            parameters.rLeverArm = {0,0,0};

            parameters.fLen[0] = sensorWidth*std::tan(M_PI_2-fov_rad/2.)/2.;
            parameters.principalPoint[0] = sensorWidth/2;
            parameters.frontalDistortion = {0,0,0,0,0,0};
            parameters.lateralDistortion = {0,0,0,0,0,0};

            _sensorsParameters.push_back(parameters);
            int pos = _sensorsParameters.size()-1;
            _sensorParametersIndex.insert(seqId, pos);

            BilCameraParameters& sensorParameters = _sensorsParameters[pos];

            problem.AddParameterBlock(sensorParameters.tLeverArm.data(), sensorParameters.tLeverArm.size());
            problem.AddParameterBlock(sensorParameters.rLeverArm.data(), sensorParameters.rLeverArm.size());

            problem.AddParameterBlock(sensorParameters.fLen.data(), sensorParameters.fLen.size());
            problem.AddParameterBlock(sensorParameters.principalPoint.data(), sensorParameters.principalPoint.size());
            problem.AddParameterBlock(sensorParameters.frontalDistortion.data(), sensorParameters.frontalDistortion.size());
            problem.AddParameterBlock(sensorParameters.lateralDistortion.data(), sensorParameters.lateralDistortion.size());

            if (seqSensorIndex >= 0) {
                _sensorIndexMap[seqSensorIndex] = seqId;
            }

        } else {
            //set the sensor to the parameters already set for the previous line with the same sensor id.
            _sensorParametersIndex.insert(seqId, _sensorParametersIndex[_sensorIndexMap[seqSensorIndex]]);
        }

        int paramIdxs = _sensorParametersIndex[seqId];
        BilCameraParameters& sensorParameters = _sensorsParameters[paramIdxs];

        QVector<qint64> imlmids = seq->listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

        for (qint64 imlmid : imlmids) {

            BilSequenceLandmark* blm = seq->getBilSequenceLandmark(imlmid);

            qint64 lmid = blm->attachedLandmarkid();

            StereoVisionApp::ModularSBASolver::LandmarkNode* lmNode = solver->getNodeForLandmark(lmid, true);

            if (lmNode == nullptr) {
                continue;
            }

            StereoVisionApp::Landmark* lm = currentProject->getDataBlock<StereoVisionApp::Landmark>(lmNode->lmId);

            Eigen::Vector2d uv(blm->x().value(), 0);

            double time = seq->getTimeFromPixCoord(blm->y().value());

            int trajNodeId = trajNode->getNodeForTime(time);

            if (trajNodeId < 0 or trajNodeId+1 >= trajNode->nodes.size()) {
                continue;
            }

            StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& previousPose = trajNode->nodes[trajNodeId];
            StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& nextPose = trajNode->nodes[trajNodeId+1];

            double w1 = (nextPose.time - time)/(nextPose.time - previousPose.time);
            double w2 = (time - previousPose.time)/(nextPose.time - previousPose.time);

            ParametrizedInterpolatedXYZ2PushBroom* cost =
                    new ParametrizedInterpolatedXYZ2PushBroom(uv, sensorWidth, w1, w2);

            using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedInterpolatedXYZ2PushBroom,2,3,3,3,3,3,3,3,1,1,6,6>;
            ceresFunc* costFunc = new ceresFunc(cost);

            problem.AddResidualBlock(costFunc, nullptr,
                                     lmNode->pos.data(),
                                     sensorParameters.rLeverArm.data(), sensorParameters.tLeverArm.data(),
                                     previousPose.rAxis.data(), previousPose.t.data(),
                                     nextPose.rAxis.data(), nextPose.t.data(),
                                     sensorParameters.fLen.data(),
                                     sensorParameters.principalPoint.data(),
                                     sensorParameters.frontalDistortion.data(),
                                     sensorParameters.lateralDistortion.data());

            QString lmName = "Fantom landmark";
            if (lm != nullptr) {
                lmName = lm->objectName();
            }
            QString loggerName = QString("Projection Bil %1 Landmark %2").arg(seq->objectName()).arg(lmName);
            StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<11, 2>::ParamsType params =
                {lmNode->pos.data(),
                 sensorParameters.rLeverArm.data(), sensorParameters.tLeverArm.data(),
                 previousPose.rAxis.data(), previousPose.t.data(),
                 nextPose.rAxis.data(), nextPose.t.data(),
                 sensorParameters.fLen.data(),
                 sensorParameters.principalPoint.data(),
                 sensorParameters.frontalDistortion.data(),
                 sensorParameters.lateralDistortion.data()
                };

            solver->addLogger(loggerName, new StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<11, 2>(costFunc, params));

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
