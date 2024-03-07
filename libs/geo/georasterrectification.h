#ifndef GEORASTERRECTIFICATION_H
#define GEORASTERRECTIFICATION_H

#include "../io/georasterreader.h"
#include <StereoVision/geometry/core.h>

#include <proj.h>

namespace PikaLTools {

template <typename T, int nDim>
struct RectifiedGeoRasterData {

    static_assert (nDim == 2 or nDim == 3, "a georaster can only have 2 or three dimensions");

    //! \brief the
    Multidim::Array<T, nDim> raster;

    //! \brief _ecef2imageLocal encode the transformation between the ECEF frame and the local frame of the image (euclidian with x&y in px and z in meter).
    StereoVision::Geometry::AffineTransform<double> _ecef2imageLocal;

};

/*!
 * \brief rectifyDtm rectify a dtm such that the image represent the height on a local plane rather than above the ellipsoid
 * \param rawDtm the dtm, given as height above an ellipsoid
 * \return a RectifiedGeoRasterData struct containing the rectified dtm and the transform from the ecef frame to the local frame
 */
template <typename T>
RectifiedGeoRasterData<T, 2> rectifyDtm(StereoVisionApp::Geo::GeoRasterData<T, 2> const& rawDtm) {

    OGRSpatialReference ogrSpatialRef(rawDtm.crsInfos.c_str());
    bool invertXY = ogrSpatialRef.EPSGTreatsAsLatLong(); //ogr will always treat coordinates as lon then lat, but proj will stick to the epsg order definition. This mean we might need to invert the order.

    int nPoints = rawDtm.raster.shape()[0]*rawDtm.raster.shape()[1];

    typename Multidim::Array<T,3>::ShapeBlock shape = {rawDtm.raster.shape()[0], rawDtm.raster.shape()[1], 3};
    typename Multidim::Array<T,3>::ShapeBlock strides = {rawDtm.raster.shape()[1]*3, 3, 1}; //ensure each triplets of floats is a point.

    typename Multidim::Array<double,3> coordinates_vecs(shape, strides);

    for (int i = 0; i < shape[0]; i++) {
        for (int j = 0; j < shape[1]; j++) {

            Eigen::Vector3d homogeneousImgCoord(j,i,1);
            Eigen::Vector2d geoCoord = rawDtm.geoTransform*homogeneousImgCoord;

            if (invertXY) {
                coordinates_vecs.atUnchecked(i,j,0) = geoCoord.y();
                coordinates_vecs.atUnchecked(i,j,1) = geoCoord.x();
            } else {
                coordinates_vecs.atUnchecked(i,j,0) = geoCoord.x();
                coordinates_vecs.atUnchecked(i,j,1) = geoCoord.y();
            }

            coordinates_vecs.atUnchecked(i,j,2) = 0;

        }
    }


    PJ_CONTEXT* ctx = proj_context_create();


    const char* wgs84_ecef = "EPSG:4978";


    PJ* reprojector = proj_create_crs_to_crs(ctx, rawDtm.crsInfos.c_str(), wgs84_ecef, nullptr);

    if (reprojector == 0) { //in case of error
        return false;
    }


    proj_trans_generic(reprojector, PJ_FWD,
                       &coordinates_vecs.atUnchecked(0,0,0), nPoints*sizeof(double), 3,
                       &coordinates_vecs.atUnchecked(0,0,1), nPoints*sizeof(double), 3,
                       &coordinates_vecs.atUnchecked(0,0,2), nPoints*sizeof(double), 3,
                       nullptr,0,0); //reproject to ecef coordinates


    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    //estimate local frame
    Eigen::Vector3f origin;
    origin << coordinates_vecs.valueUnchecked(0,0,0), coordinates_vecs.valueUnchecked(0,0,1), coordinates_vecs.valueUnchecked(0,0,2);

    Eigen::Vector3f local_frame_x_pixVec;
    local_frame_x_pixVec << coordinates_vecs.valueUnchecked(0,shape[1]-1,0), coordinates_vecs.valueUnchecked(0,shape[1]-1,1), coordinates_vecs.valueUnchecked(0,shape[1]-1,2);
    local_frame_x_pixVec -= origin;
    local_frame_x_pixVec /= (shape[1]-1);
    Eigen::Vector3f local_frame_x_axis = local_frame_x_pixVec.normalized();


    Eigen::Vector3f local_frame_y_pixVec;
    local_frame_y_pixVec << coordinates_vecs.valueUnchecked(shape[0]-1,0,0), coordinates_vecs.valueUnchecked(shape[0]-1,0,1), coordinates_vecs.valueUnchecked(shape[0]-1,0,2);
    local_frame_y_pixVec -= origin;
    local_frame_y_pixVec /= (shape[0]-1);
    Eigen::Vector3f local_frame_y_axis = local_frame_y_pixVec.normalized();

    Eigen::Vector3f local_frame_z_axis = local_frame_y_axis.cross(local_frame_x_axis);

    Eigen::Matrix3f A;
    A.col(0) = local_frame_x_pixVec;
    A.col(1) = local_frame_y_pixVec;
    A.col(2) = local_frame_z_axis;

    StereoVision::Geometry::AffineTransform<double> local2ecef(A, origin); //transform from the image local frame to ecef.

    Eigen::Matrix3f inv = local2ecef.R.inverse();
    StereoVision::Geometry::AffineTransform<double> ecef2local(inv, -inv*origin); //this can map points from ecef to the local plane approximating the image


    Multidim::Array<double,3> local_coordinates_vecs(shape, strides);

    //offset of positions due to height above ellipsoid
    float minX = 0;
    float maxX = 0;
    float minY = 0;
    float maxY = 0;

    for (int i = 0; i < shape[0]; i++) {
        for (int j = 0; j < shape[1]; j++) {

            Eigen::Vector3d ecefCoord(coordinates_vecs.atUnchecked(i,j,0), coordinates_vecs.atUnchecked(i,j,1), coordinates_vecs.atUnchecked(i,j,2));

            Eigen::Vector3d localCoords = ecef2local*ecefCoord;

            double localx = localCoords.x();
            double localy = localCoords.y();
            double localz = localCoords.z();

            local_coordinates_vecs.atUnchecked(i,j,0) = localx;
            local_coordinates_vecs.atUnchecked(i,j,1) = localy;
            local_coordinates_vecs.atUnchecked(i,j,2) = localz;

            if (localx < minX) {
                minX = localx;
            }

            if (localy < minY) {
                minY = localy;
            }

            if (localx > maxX) {
                maxX = localx;
            }

            if (localy > maxY) {
                maxY = localy;
            }


        }
    }

    StereoVision::Geometry::AffineTransform<double> localCorrection(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-minX, -minY, 0));

    //pixel size in cartesian world
    Eigen::Vector3f offset1Pixel;
    offset1Pixel << local_coordinates_vecs.valueUnchecked(1,0,0), local_coordinates_vecs.valueUnchecked(1,0,1), local_coordinates_vecs.valueUnchecked(1,0,2);

    float pixDist = offset1Pixel.norm();

    StereoVision::Geometry::AffineTransform<double> scale2Img(Eigen::DiagonalMatrix<double,3>(1/pixDist, 1/pixDist, 1), Eigen::Vector3f::Zero());

    int nPixelsX = std::ceil((maxX - minX)/pixDist);
    int nPixelsY = std::ceil((maxY - minY)/pixDist);

    Multidim::Array<T, 2> rectifiedHeightModel(nPixelsY, nPixelsX);
    std::fill_n(&rectifiedHeightModel.atUnchecked(0,0), rectifiedHeightModel.flatLenght(), 0);

    Multidim::Array<T, 2> nObservations(nPixelsY, nPixelsX);
    std::fill_n(&nObservations.atUnchecked(0,0), nObservations.flatLenght(), 0);

    for (int i = 0; i < local_coordinates_vecs.shape()[0]; i++) {
        for (int j = 0; j < local_coordinates_vecs.shape()[1]; j++) {

            Eigen::Vector3d localCoord(local_coordinates_vecs.atUnchecked(i,j,0), local_coordinates_vecs.atUnchecked(i,j,1), local_coordinates_vecs.atUnchecked(i,j,2));

            Eigen::Vector3d shiftedCoord = localCoord + localCorrection.t;
            Eigen::Vector2d imgCoord = shiftedCoord.block<2,1>(0,0)/pixDist;

            double z = shiftedCoord.z();
            int rx = std::round(shiftedCoord.x());
            int ry = std::round(shiftedCoord.y());

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {

                    int idx = rx+dx;
                    int idy = ry+dy;

                    //set the weight to 0 if out (branchless programming)
                    T w = (idx >= 0 and idx < rectifiedHeightModel.shape()[1]) ? 1 : 0;
                    w *= (idy >= 0 and idy < rectifiedHeightModel.shape()[0]) ? 1 : 0;

                    w *= std::max(0, -std::abs(idx-rx) + 1);
                    w *= std::max(0, -std::abs(idy-ry) + 1);

                    //set the index to 0 if out
                    idx = (idx >= 0 and idx < rectifiedHeightModel.shape()[1]) ? idx : 0;
                    idy = (idy >= 0 and idy < rectifiedHeightModel.shape()[0]) ? idy : 0;

                    rectifiedHeightModel.atUnchecked(idy, idx) += w*z;
                    nObservations.atUnchecked(idy, idx) += w;

                }
            }

        }
    }

    for (int i = 0; i < rectifiedHeightModel.shape()[0]; i++) {
        for (int j = 0; j < rectifiedHeightModel.shape()[1]; j++) {

            T nObs = nObservations.valueUnchecked(i, j);

            rectifiedHeightModel.atUnchecked(i, j) /= (nObs > 1e-9) ? nObs : std::nan("");
        }
    }

    return RectifiedGeoRasterData<T,2> { std::move(rectifiedHeightModel), scale2Img*localCorrection*ecef2local } ;

}

}

#endif // GEORASTERRECTIFICATION_H
