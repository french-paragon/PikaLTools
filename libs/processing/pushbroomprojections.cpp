#include "pushbroomprojections.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <StereoVision/imageProcessing/pixelsLines.h>

#include <set>

namespace PikaLTools {

/*!
 * \brief findIntersectionWithLine solve the equation a*v1 + (1-a)*v2 = v0 + t*vd
 * \param v0 the start of the ray
 * \param vd the direction of the ray
 * \param v1 the first barycentric vector on the segment
 * \param v2 the second barycentric vector on the segment.
 * \return  v0 + t*vd
 */
inline Eigen::Vector2f findIntersectionWithLine(Eigen::Vector2f const& v0,
                                                Eigen::Vector2f const& vd,
                                                Eigen::Vector2f const& v1,
                                                Eigen::Vector2f const& v2) {

    Eigen::Matrix2f M;
    M.col(0) = v1 - v2;
    M.col(1) = -vd;

    Eigen::Vector2f sol =  M.inverse()*(v0 - v2);

    return v0 + sol[1]*vd;

}

PixReprojection findPixCoordinateInLine(Eigen::Vector3f const& pointTarget,
                                        StereoVision::Geometry::AffineTransform<float> ref2cam,
                                        float camFlen,
                                        float camPP) {

    Eigen::Vector3f pointCamRef = ref2cam*pointTarget;

    Eigen::Vector2f normalized = pointCamRef.block<2,1>(0,0)/pointCamRef[2];

    PixReprojection ret;
    ret.pixCoord = normalized.x()*camFlen + camPP;
    ret.verticalError = normalized.y();

    return ret;

}

Eigen::Vector2f findRayIntersection(Multidim::Array<float, 2> const& heightMap,
                                    Eigen::Vector3f const& rayOrigin,
                                    Eigen::Vector3f const& rayDirection,
                                    bool extendHeightMap,
                                    std::optional<float> pMaxHeight) {

    float maxHeight;

    if (pMaxHeight.has_value()) {
        maxHeight = pMaxHeight.value();
    } else {
        maxHeight = heightMap.valueUnchecked(0,0);

        for (int i = 0; i < heightMap.shape()[0]; i++) {
            for (int j = 0; j < heightMap.shape()[1]; j++) {
                float h = heightMap.valueUnchecked(i,j);

                if (h > maxHeight) {
                    maxHeight = h;
                }
            }
        }
    }

    using MatHType = Eigen::Matrix<float, 4, 4>;
    using VecvType = Eigen::Matrix<float, 4, 1>;

    //check between which 4 pixels the ray reach the maximum height

    float projT = (maxHeight - rayOrigin[2])/rayDirection[2];

    if (projT < 0) {
        projT = 0;
    }

    Eigen::Vector3f startPos = rayOrigin + projT*rayDirection;

    std::array<int, 4> pixels_x_coords;
    std::array<int, 4> pixels_y_coords;

    Eigen::Vector2f currentSearchPoint = startPos.block<2,1>(0,0);

    Eigen::Vector3f startPosNoInt = startPos;

    if (std::floor(startPosNoInt.x()) == std::ceil(startPosNoInt.x())) {
        startPosNoInt.x() += (startPos.x() > 0) ? 0.5 : -0.5;
    }

    if (std::floor(startPosNoInt.y()) == std::ceil(startPosNoInt.y())) {
        startPosNoInt.y() += (startPos.y() > 0) ? 0.5 : -0.5;
    }

    for (int i = 0; i < 4; i++) {

        int x = (i < 2) ? std::floor(startPosNoInt.x()) : std::ceil(startPosNoInt.x());
        int y = (i % 2 == 0) ? std::floor(startPosNoInt.y()) : std::ceil(startPosNoInt.y());

        pixels_x_coords[i] = x;
        pixels_y_coords[i] = y;

        if (!extendHeightMap) {
            if (x < 0 or x >= heightMap.shape()[0] or y < 0 or y >= heightMap.shape()[1]) {
                return Eigen::Vector2f(-1,-1);
            }
        }
    }

    Eigen::Vector2i minCorner(pixels_x_coords[0], pixels_y_coords[0]);

    auto computeNextSetOf4Pixels = [&rayOrigin,
            &rayDirection,
            &pixels_x_coords,
            &pixels_y_coords,
            &minCorner,
            &heightMap,
            &currentSearchPoint,
            extendHeightMap] () -> bool {

        Eigen::Vector2f v0 = rayOrigin.block<2,1>(0,0);
        Eigen::Vector2f vd = rayDirection.block<2,1>(0,0);

        //find the increment

        Eigen::Vector2f currentCentroid(static_cast<float>(pixels_x_coords[0] + pixels_x_coords[1] + pixels_x_coords[2] + pixels_x_coords[3])/4,
                static_cast<float>(pixels_y_coords[0] + pixels_y_coords[1] + pixels_y_coords[2] + pixels_y_coords[3])/4);

        Eigen::Vector2i rayDirSign;
        rayDirSign[0] = vd[0] > 0 ? 1 : -1;
        rayDirSign[1] = vd[1] > 0 ? 1 : -1;

        std::array<std::array<int,2>, 3> possibleIncrements = {std::array<int,2>{rayDirSign[0], 0},
                                                               std::array<int,2>{0, rayDirSign[1]},
                                                               std::array<int,2>{rayDirSign[0], rayDirSign[1]}};

        float currentT = std::numeric_limits<float>::infinity();
        Eigen::Vector2i incr;

        for (std::array<int,2> const& incrCandidate : possibleIncrements) {

            Eigen::Vector2f incrementedCentroid = currentCentroid + Eigen::Vector2f(incrCandidate[0], incrCandidate[1]);

            float approxT = vd.dot(incrementedCentroid - v0)/vd.dot(vd);

            Eigen::Vector2f approxDelta = v0 + approxT*vd - incrementedCentroid;

            if (std::fabs(approxDelta[0]) > 0.5 or std::fabs(approxDelta[1]) > 0.5) {
                continue; //line do not pass in the pixel
            }

            if (approxT < currentT) {
                incr[0] = incrCandidate[0];
                incr[1] = incrCandidate[1];
                currentT = approxT;
            }
        }

        if (!std::isfinite(currentT)) {
            return false;
        }

        for (int i = 0; i < 4; i++) {
            pixels_x_coords[i] += incr[0];
            pixels_y_coords[i] += incr[1];
        }

        minCorner += incr;

        currentSearchPoint = v0 + currentT*vd;

        if (!extendHeightMap) {
            for (int i = 0; i < 4; i++) {
                if (pixels_x_coords[i] < 0 or pixels_x_coords[i] >= heightMap.shape()[0]) {
                    return true; //did overshoot
                }
                if (pixels_y_coords[i] < 0 or pixels_y_coords[i] >= heightMap.shape()[1]) {
                    return true; //did overshoot
                }
            }
        }

        return false;

    };

    //for the current 4 pixels we are in between

    while(true) {
        //check if the ray intersect
        float dirProjNorm = rayDirection.block<2,1>(0,0).norm();
        float tEst = (currentSearchPoint - rayOrigin.block<2,1>(0,0)).norm()/dirProjNorm;

        if (!(std::isinf(tEst) or std::isnan(tEst))) {

            //basic test, just check the max and min height
            float hHigh;
            float hLow;

            int xIdx = std::clamp(pixels_x_coords[0], 0, heightMap.shape()[0]-1);
            int yIdx = std::clamp(pixels_y_coords[0], 0, heightMap.shape()[1]-1);

            hHigh = heightMap.valueUnchecked(xIdx, yIdx);
            hLow = hHigh;

            for (int i = 1; i < 4; i++) {

                xIdx = std::clamp(pixels_x_coords[i], 0, heightMap.shape()[0]-1);
                yIdx = std::clamp(pixels_y_coords[i], 0, heightMap.shape()[1]-1);

                float val = heightMap.valueUnchecked(xIdx, yIdx);

                if (val > hHigh) {
                    hHigh = val;
                }

                if (val < hLow) {
                    hLow = val;
                }
            }

            float nProjT = tEst + std::sqrt(2)/dirProjNorm;

            float hMax = rayOrigin.z() + tEst*rayDirection.z();
            float hMin = rayOrigin.z() + nProjT*rayDirection.z();

            if (hMin > hHigh) {

                bool out = computeNextSetOf4Pixels();

                if (out) {
                    //if the loop reach the side of the image without having found an intersection, return invalid coordinate;
                    return Eigen::Vector2f(std::nanf(""),std::nanf(""));
                }

                continue;
            }

            if (hLow > hMax) {
                return Eigen::Vector2f(std::nanf(""),std::nanf(""));
            }

        }

        //full test
        MatHType H;

        //We are trying to find the solution to the overdetermined system Ha = v0 + t*vd such that the solution a is as close as possible to the centroid of the four corners.

        //The way we do that is by first finding the t that yields the optimal a, then solve for a.

        for (int i = 0; i < 4; i++) {
            H(0,i) = pixels_x_coords[i];
            H(1,i) = pixels_y_coords[i];

            int xIdx = std::clamp(pixels_x_coords[i], 0, heightMap.shape()[0]-1);
            int yIdx = std::clamp(pixels_y_coords[i], 0, heightMap.shape()[1]-1);

            H(2,i) = heightMap.valueUnchecked(xIdx, yIdx);
            H(3,i) = 1;
        }

        VecvType v0;
        VecvType vd;

        v0.block<3,1>(0,0) = rayOrigin;
        v0[3] = 1;

        vd.block<3,1>(0,0) = rayDirection;
        vd[3] = 0;

        auto Hinv = H.completeOrthogonalDecomposition();

        VecvType Hm1v0 = Hinv.solve(v0);
        VecvType Hm1vd = Hinv.solve(vd);

        VecvType Hm1v0m05 = Hm1v0;

        for (int i = 0; i < 4; i++) {
            Hm1v0m05[i] -= 0.5; //ensure we are as close as possible to the middle of the simplex when fitting the homogeneous coordinates.
        }

        float opt_t = -Hm1vd.dot(Hm1v0m05)/Hm1vd.dot(Hm1vd);

        VecvType a = Hm1v0 + opt_t*Hm1vd; //barycentric coordinates

        bool ok = true;
        for (int i = 0; i < 4; i++) {
            if (std::isinf(a[i]) or std::isnan(a[i]) or a[i] < 0 or a[i] > 1) {
                ok = false;
                break;
            }
        }

        if (!ok) {

            bool out = computeNextSetOf4Pixels();

            if (out) {
                //if the loop reach the side of the image without having found an intersection, return invalid coordinate;
                return Eigen::Vector2f(std::nanf(""),std::nanf(""));
            }

            continue;
        }

        //if yes return the intersection coordinate
        Eigen::Vector2f combined;
        combined[0] = 0;
        combined[1] = 0;

        for (int i = 0; i < 4; i++) {
            combined[0] += a[i]*pixels_x_coords[i];
            combined[1] += a[i]*pixels_y_coords[i];
        }

        return combined;
    }

    return Eigen::Vector2f(std::nanf(""),std::nanf(""));
}

std::vector<std::array<int, 2>> terrainPixelsSeenByScannerLine(Multidim::Array<float, 2> const& terrain,
                                                               StereoVision::Geometry::AffineTransform<float> const& cam2terrain,
                                                               float camFLen,
                                                               float camPP,
                                                               int nSensorPixels,
                                                               std::optional<float> pMaxHeight) {

    Eigen::Vector3f leftSide((nSensorPixels - 1 - camPP)/camFLen, 0,1);
    Eigen::Vector3f rightSide(-camPP/camFLen, 0,1);

    Eigen::Vector3f camPos = cam2terrain.t;

    Eigen::Vector3f leftDir = cam2terrain*leftSide - camPos;
    Eigen::Vector3f rightDir = cam2terrain*rightSide - camPos;

    float maxHeight;

    if (pMaxHeight.has_value()) {

        maxHeight = pMaxHeight.value();

    } else {

        float maxHeight = terrain.valueUnchecked(0,0);

        for (int i = 0; i < terrain.shape()[0]; i++) {
            for (int j = 0; j < terrain.shape()[1]; j++) {
                float h = terrain.valueUnchecked(i,j);

                if (h > maxHeight) {
                    maxHeight = h;
                }
            }
        }
    }

    bool extendHeightMap = true;

    Eigen::Vector2f leftCoord = findRayIntersection(terrain, camPos, leftDir, extendHeightMap, maxHeight);
    Eigen::Vector2f rightCoord = findRayIntersection(terrain, camPos, rightDir, extendHeightMap, maxHeight);

    bool discret = true;

    Eigen::Array<float, 2, Eigen::Dynamic> linePoints = StereoVision::ImageProcessing::listPixPointsOnLine(leftCoord, rightCoord, discret);

    std::vector<std::array<int, 2>> discretPoints(linePoints.cols());
    std::set<std::array<int, 2>> pointsSet;

    for (int i = 0; i < linePoints.cols(); i++) {
        float iFrac = linePoints(0,i);
        float jFrac = linePoints(1,i);

        std::array<int, 2> coord{static_cast<int>(std::round(iFrac)), static_cast<int>(std::round(jFrac))};

        discretPoints[i] = coord;
        pointsSet.insert(coord);
    }

    if (discretPoints.size() != pointsSet.size()) {
        std::cout << "Point extraction error" << std::endl;
    }

    return discretPoints;
}

} // namespace PikaLTools
