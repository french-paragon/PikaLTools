#include "coordinate_conversions.h"

#include <proj.h>

std::optional<Trajectory<double>> convertLcfSequenceToTrajectory(std::vector<EnviBilLcfLine> const& lines) {

    if (lines.empty()) {
        return std::nullopt;
    }

    using TimedElement = Trajectory<double>::TimedElement;
    std::vector<TimedElement> elements;
    elements.reserve(lines.size());

    PJ_CONTEXT* ctx = proj_context_create();

    const char* wgs84_latlon = "EPSG:4326";
    const char* wgs84_ecef = "EPSG:4978";
    PJ* converter = proj_create_crs_to_crs(ctx, wgs84_latlon, wgs84_ecef, nullptr);

    if (converter == 0) { //in case of error
        return std::nullopt;
    }

    for (EnviBilLcfLine line : lines) {

        double vx = line.lat;
        double vy = line.lon;
        double vz = line.height;

        Eigen::Vector3f pos;

        proj_trans_generic(converter, PJ_FWD, &vx, 0, 1, &vy, 0, 1, &vz, 0, 1, nullptr, 0, 1);

        Eigen::Vector3d t(vx, vy, vz);

        StereoVision::Geometry::AffineTransform<double> ecef2ned = getLocalFrameAtPosNED<double>(line.lat, line.lon, WGS84_Ellipsoid, line.height);

        line.pitch = line.pitch;
        line.roll = -line.roll;

        Eigen::Matrix3d yawRMat;
        yawRMat << cos(line.yaw), -sin(line.yaw), 0,
                       sin(line.yaw), cos(line.yaw), 0,
                       0, 0, 1;

        Eigen::Matrix3d pitchRMat;
        pitchRMat << cos(line.pitch), 0, sin(line.pitch),
                         0, 1, 0,
                         -sin(line.pitch), 0, cos(line.pitch);

        Eigen::Matrix3d rollRMat;
        rollRMat << 1, 0, 0,
                        0, cos(line.roll), -sin(line.roll),
                        0, sin(line.roll), cos(line.roll);

        Eigen::Matrix3d RIMU2NED = yawRMat*pitchRMat*rollRMat;

        Eigen::Matrix3d cam2imu = Eigen::Matrix3d::Identity();
        cam2imu << 0, 1, 0,
                       -1, 0, 0,
                       0, 0, 1;

        StereoVision::Geometry::AffineTransform<double> composed(ecef2ned.R.transpose()*RIMU2NED*cam2imu,t);

        TimedElement element;
        element.time = line.timeStamp;
        element.val = StereoVision::Geometry::RigidBodyTransform<double>(composed);

        elements.push_back(element);
    }

    return Trajectory<double>(std::move(elements));
}
