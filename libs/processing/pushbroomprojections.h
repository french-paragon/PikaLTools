#ifndef PIKALTOOLS_PUSHBROOMPROJECTIONS_H
#define PIKALTOOLS_PUSHBROOMPROJECTIONS_H

#include <Eigen/Core>
#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/geometry/core.h>

#include <optional>

namespace PikaLTools {


struct PixReprojection{
    float pixCoord; //the coordinate of the reprojected point in pixels
    float verticalError; //the vertical error when reprojecting.
};


PixReprojection findPixCoordinateInLine(Eigen::Vector3f const& pointTarget,
                                        StereoVision::Geometry::AffineTransform<float> ref2cam,
                                        float camFlen,
                                        float camPP);

Eigen::Vector2f findRayIntersection(Multidim::Array<float, 2> const& heightMap,
                                    Eigen::Vector3f const& rayOrigin,
                                    Eigen::Vector3f const& rayDirection,
                                    bool extendHeightMap = true,
                                    std::optional<float> maxHeight = std::nullopt);

std::vector<std::array<int, 2>> terrainPixelsSeenByScannerLine(Multidim::Array<float, 2> const& terrain,
                                                               StereoVision::Geometry::AffineTransform<float> const& cam2terrain,
                                                               float camFLen,
                                                               float camPP,
                                                               int nSensorPixels,
                                                               std::optional<float> maxHeight = std::nullopt);

Multidim::Array<bool, 2> terrainPixelsSeenForTrajectory(Multidim::Array<float, 2> const& terrain,
                                                        std::vector<StereoVision::Geometry::AffineTransform<float>> const& trajectory,
                                                        float camFLen,
                                                        float camPP,
                                                        int nSensorPixels,
                                                        std::optional<float> maxHeight = std::nullopt);

} // namespace PikaLTools

#endif // PIKALTOOLS_PUSHBROOMPROJECTIONS_H
