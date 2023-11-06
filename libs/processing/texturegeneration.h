#ifndef PIKALTOOLS_TEXTUREGENERATION_H
#define PIKALTOOLS_TEXTUREGENERATION_H

#include <MultidimArrays/MultidimArrays.h>
#include <MultidimArrays/MultidimIndexManipulators.h>

#include <random>

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace PikaLTools {

template<typename T>
Multidim::Array<T, 2> generateTerrainTexture(int w,
                                             int h,
                                             T max,
                                             T min = 0,
                                             int featureSquareSide = 10,
                                             float spatialCorrelation = 0.8,
                                             int rSeed = -1) {

    if (rSeed >= 0) {
        srand(rSeed);
    } else {
        srand((unsigned int) time(nullptr));
    }

    using NodeVecT = Eigen::Matrix<T,Eigen::Dynamic,1>;
    using NodeMatT = Eigen::Matrix<T,Eigen::Dynamic,1>;

    int regionSide = std::min(h/featureSquareSide, w/featureSquareSide);

    float regionInnerMargins = 0.9;

    int nRegionsW = std::ceil(static_cast<float>(w)/regionSide);
    int nRegionsH = std::ceil(static_cast<float>(h)/regionSide);

    Multidim::Array<T, 2> nodesHeights(nRegionsH, nRegionsW);
    Multidim::Array<float, 3> nodesDistortion(nRegionsH, nRegionsW, 2);

    Multidim::IndexConverter<2> nodesIdxConv(nodesHeights.shape());

    int nNodes = nodesIdxConv.numberOfPossibleIndices();

    NodeVecT nodesNoiseVec = NodeVecT::Random(nNodes,1);
    NodeVecT nodesMeanVec = NodeVecT::Constant(nNodes,1, max - min);

    T std = (max - min)/4;

    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> nodesHeightCov;
    nodesHeightCov.resize(nNodes, nNodes);

    for (int i = 0; i < nNodes; i++) {

        auto posI = nodesIdxConv.getIndexFromPseudoFlatId(i);

        for (int j = 0; j < nNodes; j++) {

            if (i == j) {
                nodesHeightCov(i,j) = std*std;
                continue;
            }

            auto posJ = nodesIdxConv.getIndexFromPseudoFlatId(j);

            int delta_i = posI[0] - posJ[0];
            int delta_j = posI[1] - posJ[1];
            float dist = sqrt(delta_i*delta_i + delta_j*delta_j);

            float corr = spatialCorrelation/dist;
            nodesHeightCov(i,j) = std*std*corr;

        }
    }

    Eigen::LLT decomposition = nodesHeightCov.llt();

    NodeVecT nodesHeightVec = decomposition.matrixU()*nodesNoiseVec + nodesMeanVec;

    int seed = rSeed + 42;

    if (rSeed < 0) {
        std::random_device rd;
        seed = rd();
    }
    std::default_random_engine re(seed);

    std::uniform_real_distribution nodesDeltaDist(-regionInnerMargins/2, regionInnerMargins/2);

    for (int i = 0; i < nRegionsH; i++) {
        for (int j = 0; j < nRegionsW; j++) {

            int flatId = nodesIdxConv.getPseudoFlatIdFromIndex({i,j});

            nodesHeights.atUnchecked(i,j) = std::clamp(nodesHeightVec[flatId], min, max);

            nodesDistortion.atUnchecked(i,j,0) = nodesDeltaDist(re) + i;
            nodesDistortion.atUnchecked(i,j,1) = nodesDeltaDist(re) + j;

        }
    }

    auto interpolationKernel = [] (float coord) { //sligthly smoother than linear itnerpolation
        if (coord < 0.5) {
            return -2*coord*coord + 1;
        }

        float mirror = 1 - coord;
        return 2*mirror*mirror;
    };

    Multidim::Array<T,2> terrain(h,w);

    for (int i = 0; i < h; i++) {

        int lowerNodeI = i/regionSide;
        int higherNodeI = lowerNodeI+1;

        if (higherNodeI == nRegionsH) {
            higherNodeI = lowerNodeI;
        }

        float lowerNodeIDist = static_cast<float>(i)/regionSide - lowerNodeI;

        float coeffLi = interpolationKernel(lowerNodeIDist);
        float coeffHi = 1 - coeffLi;

        for (int j = 0; j < w; j++) {

            int lowerNodeJ = j/regionSide;
            int higherNodeJ = lowerNodeJ+1;

            if (higherNodeJ == nRegionsW) {
                higherNodeJ = lowerNodeJ;
            }

            float lowerNodeJDist = static_cast<float>(j)/regionSide - lowerNodeJ;

            float coeffLj = interpolationKernel(lowerNodeJDist);
            float coeffHj = 1 - coeffLj;


            float interpolatedCoordI = coeffLj*(coeffLi*nodesDistortion.valueUnchecked(lowerNodeI, lowerNodeJ, 0) +
                                                coeffHi*nodesDistortion.valueUnchecked(higherNodeI, lowerNodeJ, 0)) +
                                        coeffHj*(coeffLi*nodesDistortion.valueUnchecked(lowerNodeI, higherNodeJ, 0) +
                                                 coeffHi*nodesDistortion.valueUnchecked(higherNodeI, higherNodeJ, 0));

            float interpolatedCoordJ = coeffLj*(coeffLi*nodesDistortion.valueUnchecked(lowerNodeI, lowerNodeJ, 1) +
                                                coeffHi*nodesDistortion.valueUnchecked(higherNodeI, lowerNodeJ, 1)) +
                                        coeffHj*(coeffLi*nodesDistortion.valueUnchecked(lowerNodeI, higherNodeJ, 1) +
                                                 coeffHi*nodesDistortion.valueUnchecked(higherNodeI, higherNodeJ, 1));

            int interpLowI = std::floor(interpolatedCoordI);

            float interpLowIdist = interpolatedCoordI - interpLowI;

            if (interpLowI < 0) {
                interpLowI = 0;
            }

            if (interpLowI >= nRegionsH) {
                interpLowI = nRegionsH-1;
            }

            int interpHighI = interpLowI+1;

            if (interpHighI == nRegionsH) {
                interpHighI = nRegionsH-1;
            }

            float coeffIntLi = interpolationKernel(interpLowIdist);
            float coeffIntHi = 1 - coeffIntLi;

            int interpLowJ = std::floor(interpolatedCoordJ);

            float interpLowJdist = interpolatedCoordJ - interpLowJ;

            if (interpLowJ < 0) {
                interpLowJ = 0;
            }

            if (interpLowJ >= nRegionsW) {
                interpLowJ = nRegionsW-1;
            }

            int interpHighJ = interpLowJ+1;

            if (interpHighJ == nRegionsW) {
                interpHighJ = nRegionsW-1;
            }

            float coeffIntLj = interpolationKernel(interpLowJdist);
            float coeffIntHj = 1 - coeffIntLj;


            float interpolatedHeight = coeffIntLj*(coeffIntLi*nodesHeights.valueUnchecked(interpLowI, interpLowJ) +
                                                coeffIntHi*nodesHeights.valueUnchecked(interpHighI, interpLowJ)) +
                                        coeffIntHj*(coeffIntLi*nodesHeights.valueUnchecked(interpLowI, interpHighJ) +
                                                 coeffIntHi*nodesHeights.valueUnchecked(interpHighI, interpHighJ));

            terrain.atUnchecked(i,j) = interpolatedHeight;
        }
    }

    return terrain;

}

} // namespace PikaLTools

#endif // PIKALTOOLS_TEXTUREGENERATION_H
