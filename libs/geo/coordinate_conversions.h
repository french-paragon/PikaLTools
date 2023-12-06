#ifndef COORDINATE_CONVERSIONS_H
#define COORDINATE_CONVERSIONS_H

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <LibStevi/geometry/core.h>

struct EllipsoidDefinition {
    double equatorialRadius;
    double polarRadius;
};

constexpr EllipsoidDefinition WGS84_Ellipsoid = {6378137.0, 6356752.3142};

template<typename T>
struct CartesianCoord {
    T x;
    T y;
    T z;
};

template<typename CT>
inline CartesianCoord<CT> convertLatLonToECEF(CT lat, CT lon, CT alt, EllipsoidDefinition ellipsoid) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CT x_lat;
    CT z_lat;
    CT radius_lat;

    if (std::tan(std::fabs(latRad)) < std::tan(M_PI_2 - std::fabs(latRad))) {

        CT equatorial = 1/ellipsoid.equatorialRadius;
        CT polar = std::tan(std::fabs(latRad))/ellipsoid.polarRadius;

        x_lat = std::sqrt(1/((equatorial*equatorial + polar*polar)));

        CT tmp = 1 - x_lat*x_lat*equatorial*equatorial;

        radius_lat = std::sqrt(ellipsoid.polarRadius*ellipsoid.polarRadius*tmp + 1/((equatorial*equatorial + polar*polar)));

        if (tmp <= 0) {
            z_lat = 0;
        } else {

            z_lat = ellipsoid.polarRadius*std::sqrt(tmp);

            if (lat < 0) {
                z_lat = -z_lat;
            }
        }
    } else {

        CT equatorial = std::tan(M_PI_2 - std::fabs(latRad))/ellipsoid.equatorialRadius;
        CT polar = 1/ellipsoid.polarRadius;

        z_lat = std::sqrt(1/((equatorial*equatorial + polar*polar)));

        if (lat < 0) {
            z_lat = -z_lat;
        }

        CT tmp = 1 - z_lat*z_lat*polar*polar;

        radius_lat = std::sqrt(ellipsoid.equatorialRadius*ellipsoid.equatorialRadius*tmp + 1/((equatorial*equatorial + polar*polar)));

        if (tmp <= 0) {
            x_lat = 0;
        } else {

            x_lat = ellipsoid.equatorialRadius*std::sqrt(tmp);
        }

    }

    CT scaling = (radius_lat+alt)/radius_lat;
    x_lat *= scaling;
    z_lat *= scaling;

    CartesianCoord<CT> ret;
    ret.x = std::cos(lonRad)*x_lat;
    ret.y = std::sin(lonRad)*x_lat;
    ret.z = z_lat;

    return ret;

}

/*!
 * \brief getLocalFrameAtPos get the affine transform from ECEF to a local frame at the position on the ellipsoid
 * \param lat the current latitude
 * \param lon the current longitude
 * \param ellipsoid the ellipsoid to use
 * \return a 3x4 matrix, representing the affine transform from ECEF to the local frame at altitude alt.
 */
template<typename CT>
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPos(CT lat, CT lon, EllipsoidDefinition ellipsoid, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt, ellipsoid);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1, ellipsoid);

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
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPosNWU(CT lat, CT lon, EllipsoidDefinition ellipsoid, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt, ellipsoid);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1, ellipsoid);

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
inline StereoVision::Geometry::AffineTransform<CT> getLocalFrameAtPosNED(CT lat, CT lon, EllipsoidDefinition ellipsoid, CT alt = 0) {

    CT latRad = lat*M_PI/180;
    CT lonRad = lon*M_PI/180;

    CartesianCoord<CT> zero = convertLatLonToECEF<CT>(lat, lon, alt, ellipsoid);
    CartesianCoord<CT> z_one = convertLatLonToECEF<CT>(lat, lon, alt+1, ellipsoid);

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

#endif // COORDINATE_CONVERSIONS_H
