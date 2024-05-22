#ifndef COORDINATE_CONVERSIONS_H
#define COORDINATE_CONVERSIONS_H

#include <cmath>
#include <vector>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <steviapp/vision/indexed_timed_sequence.h>

#include "../io/read_envi_bil.h"

#include "proj.h"

struct EllipsoidDefinition {
    double equatorialRadius;
    double polarRadius;
};

constexpr EllipsoidDefinition WGS84_Ellipsoid = {6378137.0, 6356752.3142};

template<typename T>
struct CartesianCoord {

    CartesianCoord(T px, T py, T pz) :
        x(px),
        y(py),
        z(pz)
    {

    }

    T x;
    T y;
    T z;
};

template<typename CT>
inline CartesianCoord<CT> convertLatLonToECEF(CT lat, CT lon, CT alt) {

    PJ_CONTEXT* ctx = proj_context_create();

    if (ctx == nullptr) {
        return CartesianCoord{static_cast<CT>(std::nan("")), static_cast<CT>(std::nan("")), static_cast<CT>(std::nan(""))};
    }

    const char* wgs84_ecef = "EPSG:4978";
    const char* wgs84_geo = "EPSG:4979";

    PJ* reprojector = proj_create_crs_to_crs(ctx, wgs84_geo, wgs84_ecef, nullptr);

    if (reprojector == nullptr) {
        proj_context_destroy(ctx);
        return CartesianCoord{static_cast<CT>(std::nan("")), static_cast<CT>(std::nan("")), static_cast<CT>(std::nan(""))};
    }

    PJ_COORD src, dst;

    src = proj_coord(lat, lon, alt, 0);

    dst = proj_trans(reprojector, PJ_FWD, src);

    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    return CartesianCoord{static_cast<CT>(dst.xyz.x), static_cast<CT>(dst.xyz.y), static_cast<CT>(dst.xyz.z)};

}

template<typename CT>
inline CartesianCoord<CT> convertECEF2LatLon(CT x, CT y, CT z) {

    PJ_CONTEXT* ctx = proj_context_create();

    if (ctx == nullptr) {
        return CartesianCoord{static_cast<CT>(std::nan("")), static_cast<CT>(std::nan("")), static_cast<CT>(std::nan(""))};
    }

    const char* wgs84_ecef = "EPSG:4978";
    const char* wgs84_geo = "EPSG:4979";

    PJ* reprojector = proj_create_crs_to_crs(ctx, wgs84_ecef, wgs84_geo, nullptr);

    if (reprojector == nullptr) {
        proj_context_destroy(ctx);
        return CartesianCoord{static_cast<CT>(std::nan("")), static_cast<CT>(std::nan("")), static_cast<CT>(std::nan(""))};
    }

    PJ_COORD src, dst;

    src = proj_coord(x, y, z, 0);

    dst = proj_trans(reprojector, PJ_FWD, src);

    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    return CartesianCoord{static_cast<CT>(dst.xyz.x), static_cast<CT>(dst.xyz.y), static_cast<CT>(dst.xyz.z)};

}

/*!
 * \brief getLocalFrameAtPos get the affine transform from ECEF to a local frame at the position on the ellipsoid
 * \param lat the current latitude
 * \param lon the current longitude
 * \param ellipsoid the ellipsoid to use
 * \return a 3x4 matrix, representing the affine transform from ECEF to the local frame at altitude alt.
 */
template<typename CT>
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPos(CT lat, CT lon, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1);

    Eigen::Matrix<CT,3,1> vec_zero(zero.x, zero.y, zero.z);
    Eigen::Matrix<CT,3,1> vec_z_one(z_one.x, z_one.y, z_one.z);

    Eigen::Matrix<CT,3,1> x_local(-std::sin(lonRad), std::cos(lonRad), 0);
    Eigen::Matrix<CT,3,1> z_local = vec_z_one - vec_zero;
    Eigen::Matrix<CT,3,1> y_local = z_local.cross(x_local);

    Eigen::Matrix<CT,3,3> Rlocal2ecef;
    Rlocal2ecef.template block<3,1>(0,0) = x_local;
    Rlocal2ecef.template block<3,1>(0,1) = y_local;
    Rlocal2ecef.template block<3,1>(0,2) = z_local;

    Eigen::Matrix<CT,3,3> Recef2local = Rlocal2ecef.transpose();

    StereoVision::Geometry::AffineTransform<CT> ret(Recef2local, -Recef2local*vec_zero);

    return ret;
}

/*!
 * \brief getLocalFrameAtPosXNorth does the same thing than getLocalFrameAtPos, using the North West Up convention
 * \param lat the current latitude
 * \param lon the current longitude
 * \param ellipsoid the ellipsoid to use
 * \return a 3x4 matrix, representing the affine transform from ECEF to the local frame.
 */
template<typename CT>
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPosNWU(CT lat, CT lon, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1);

    Eigen::Matrix<CT,3,1> vec_zero(zero.x, zero.y, zero.z);
    Eigen::Matrix<CT,3,1> vec_z_one(z_one.x, z_one.y, z_one.z);

    Eigen::Matrix<CT,3,1> y_local(std::sin(lonRad), -std::cos(lonRad), 0);
    Eigen::Matrix<CT,3,1> z_local = vec_z_one - vec_zero;
    Eigen::Matrix<CT,3,1> x_local = y_local.cross(z_local);

    Eigen::Matrix<CT,3,3> Rlocal2ecef;
    Rlocal2ecef.template block<3,1>(0,0) = x_local;
    Rlocal2ecef.template block<3,1>(0,1) = y_local;
    Rlocal2ecef.template block<3,1>(0,2) = z_local;

    Eigen::Matrix<CT,3,3> Recef2local = Rlocal2ecef.transpose();

    StereoVision::Geometry::AffineTransform<CT> ret(Recef2local, -Recef2local*vec_zero);

    return ret;
}

/*!
 * \brief getLocalFrameAtPosXNorth does the same thing than getLocalFrameAtPos, using the North East Down convention
 * \param lat the current latitude
 * \param lon the current longitude
 * \param ellipsoid the ellipsoid to use
 * \return a 3x4 matrix, representing the affine transform from ECEF to the local frame.
 */
template<typename CT>
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPosNED(CT lat, CT lon, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1);

    Eigen::Matrix<CT,3,1> vec_zero(zero.x, zero.y, zero.z);
    Eigen::Matrix<CT,3,1> vec_z_one(z_one.x, z_one.y, z_one.z);

    Eigen::Matrix<CT,3,1> y_local(-std::sin(lonRad), std::cos(lonRad), 0);
    Eigen::Matrix<CT,3,1> z_local = vec_zero - vec_z_one;
    Eigen::Matrix<CT,3,1> x_local = y_local.cross(z_local);

    Eigen::Matrix<CT,3,3> Rlocal2ecef;
    Rlocal2ecef.template block<3,1>(0,0) = x_local;
    Rlocal2ecef.template block<3,1>(0,1) = y_local;
    Rlocal2ecef.template block<3,1>(0,2) = z_local;

    Eigen::Matrix<CT,3,3> Recef2local = Rlocal2ecef.transpose();

    StereoVision::Geometry::AffineTransform<CT> ret(Recef2local, -Recef2local*vec_zero);

    return ret;
}

template<typename CT>
using Trajectory = StereoVisionApp::IndexedTimeSequence<StereoVision::Geometry::RigidBodyTransform<CT>, CT>;

/*!
 * \brief convertLcfSequenceToTrajectory convert an input lcf sequence to a series of ecef poses
 * \param lines the input lines from the lcf file
 * \return a trajectory with the times and poses as body 2 ecef transforms, or std::nullopt in case of error
 */
std::optional<Trajectory<double>> convertLcfSequenceToTrajectory(std::vector<EnviBilLcfLine> const& lines);

#endif // COORDINATE_CONVERSIONS_H
