#include "dtmtiepointsmodule.h"

#include "datablocks/inputdtm.h"

#include <steviapp/datablocks/landmark.h>

#include <proj.h>
#include <ceres/normal_prior.h>

namespace PikaLTools {

DtmTiePointsModule::DtmTiePointsModule()
{

}

bool DtmTiePointsModule::init(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> dtmsIdxs = currentProject->getIdsByClass(InputDtm::staticMetaObject.className());

    for (qint64 dtmId : qAsConst(dtmsIdxs)) {
        InputDtm* inputDtm = currentProject->getDataBlock<InputDtm>(dtmId);

        if (inputDtm == nullptr) {
            continue;
        }

        QVector<qint64> dtmLmsIdxs = inputDtm->listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());

        QVector<DtmLandmark*> dtmLandmarks;
        dtmLandmarks.reserve(dtmLmsIdxs.size());

        for (qint64 dtmLmId : qAsConst(dtmLmsIdxs)) {
            DtmLandmark* dtmLm = inputDtm->getDtmLandmark(dtmLmId);

            if (dtmLm == nullptr) {
                continue;
            }

            if (!dtmLm->x().isSet()) {
                continue;
            }

            if (!dtmLm->y().isSet()) {
                continue;
            }

            dtmLandmarks.push_back(dtmLm);
        }

        if (dtmLandmarks.isEmpty()) {
            continue;
        }

        auto optDtmData = inputDtm->dtmData();

        if (!optDtmData.has_value()) {
            continue;
        }

        StereoVisionApp::Geo::GeoRasterData<float,2>& dtmData = optDtmData.value();

        for (DtmLandmark* dtmLm : dtmLandmarks) {

            qint64 lmid = dtmLm->attachedLandmarkid();

            StereoVisionApp::ModularSBASolver::LandmarkNode* lmNode = solver->getNodeForLandmark(lmid, true);

            if (lmNode == nullptr) {
                continue;
            }

            Eigen::Vector3d imgCoord;
            imgCoord.x() = dtmLm->x().value();
            imgCoord.y() = dtmLm->y().value();
            imgCoord.z() = 1;

            if (imgCoord.x() < 0 or imgCoord.x() >= dtmData.raster.shape()[1]+1) {
                continue;
            }

            if (imgCoord.y() < 0 or imgCoord.y() >= dtmData.raster.shape()[0]+1) {
                continue;
            }

            float wX = imgCoord.x() - std::floor(imgCoord.x());
            float wY = imgCoord.y() - std::floor(imgCoord.y());

            int xm = std::floor(imgCoord.x());
            int xp = std::ceil(imgCoord.x());

            int ym = std::floor(imgCoord.y());
            int yp = std::ceil(imgCoord.y());

            double z = (1-wX)*(1-wY)*dtmData.raster.valueUnchecked(ym,xm) +
                    (wX)*(1-wY)*dtmData.raster.valueUnchecked(ym,xp) +
                    (1-wX)*(wY)*dtmData.raster.valueUnchecked(yp,xm) +
                    (wX)*(wY)*dtmData.raster.valueUnchecked(yp,xp);

            Eigen::Vector2d geoCoord = dtmData.geoTransform*imgCoord;

            Eigen::Vector3d vecPrior (geoCoord.x(), geoCoord.y(), z);

            const char* wgs84_ecef = "EPSG:4978"; //The WGS84 earth centered, earth fixed frame used for projection on the terrain

            bool needReproject = dtmData.crsInfos != wgs84_ecef and !dtmData.crsInfos.empty();

            if (needReproject) {

                PJ_CONTEXT* ctx = proj_context_create();

                PJ*  reprojector = proj_create_crs_to_crs(ctx, dtmData.crsInfos.c_str(), wgs84_ecef, nullptr);

                if (reprojector == nullptr) { //in case of error
                    proj_destroy(reprojector);
                    proj_context_destroy(ctx);
                    continue;
                }

                PJ_COORD coord;

                coord.v[0] = geoCoord.x();
                coord.v[1] = geoCoord.y();
                coord.v[2] = z;
                coord.v[3] = 0;

                PJ_COORD reproject = proj_trans(reprojector, PJ_FWD, coord);

                vecPrior.x() = reproject.v[0];
                vecPrior.y() = reproject.v[1];
                vecPrior.z() = reproject.v[2];

                proj_destroy(reprojector);
                proj_context_destroy(ctx);
            }

            if (currentProject->hasLocalCoordinateFrame()) {
                vecPrior = currentProject->ecef2local().cast<double>()*vecPrior;
            }

            ceres::Vector m;
            m.resize(3);

            m[0] = vecPrior.x();
            m[1] = vecPrior.y();
            m[2] = vecPrior.z();

            Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

            stiffness(0,0) = 1;
            stiffness(1,1) = 1;
            stiffness(2,2) = 1;

            ceres::NormalPrior* normalPrior = new ceres::NormalPrior(stiffness, m);

            lmNode->pos[0] = m[0];
            lmNode->pos[1] = m[1];
            lmNode->pos[2] = m[2];

            StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<1, 3>::ParamsType params =
                {lmNode->pos.data()};

            QString lmName = "Missing Landmark";

            StereoVisionApp::Landmark* lm = currentProject->getDataBlock<StereoVisionApp::Landmark>(lmNode->lmId);

            if (lm != nullptr) {
                lmName = lm->objectName();
            }

            QString loggerName = QString("DsmPrior Landmark %1").arg(lmName);

            solver->addLogger(loggerName, new StereoVisionApp::ModularSBASolver::AutoErrorBlockLogger<1, 3>(normalPrior, params));

            problem.AddResidualBlock(normalPrior, nullptr, lmNode->pos.data());

        }
    }

    return true;

}

bool DtmTiePointsModule::writeResults(StereoVisionApp::ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return true;
}
bool DtmTiePointsModule::writeUncertainty(StereoVisionApp::ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return true;
}
void DtmTiePointsModule::cleanup(StereoVisionApp::ModularSBASolver* solver) {
    Q_UNUSED(solver);
}

} // namespace PikaLTools
