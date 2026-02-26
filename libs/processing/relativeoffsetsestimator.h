#ifndef RELATIVEOFFSETSESTIMATOR_H
#define RELATIVEOFFSETSESTIMATOR_H

//this header file contain functions for estimating relative shifts between push-broom data lines to pre-rectify the images (e.g. for tie points search).

#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/correlation/matching_costs.h>
#include <StereoVision/interpolation/interpolation.h>
#include <StereoVision/statistics/covarianceKernels.h>
#include <StereoVision/utils/iterative_numerical_algorithm_output.h>
#include <StereoVision/utils/combinatorial.h>
#include <StereoVision/optimization/bfgs.h>

#include <iostream>

namespace PikaLTools {

namespace PushBroomRelativeOffsets {


template<typename T>
struct AccumulatedShiftsInfos {
    std::vector<T> accumulatedShifts;
    int minDelta;
    int maxDelta;
};

template<bool meanZero = false, typename T>
AccumulatedShiftsInfos<T> computeAccumulatedFromRelativeShifts(std::vector<T> const& shifts) {

    float shiftMin = 0;
    float shiftMax = 0;
    float accumulatedShift = 0;

    for (int i = 0; i < shifts.size(); i++) {
        accumulatedShift -= shifts[i]; //disparity of +n pix mean the object is further on the right, so the line has to be pushed on the left to compensate.
        shiftMin = std::min(shiftMin, accumulatedShift);
        shiftMax = std::max(shiftMax, accumulatedShift);
    }

    AccumulatedShiftsInfos<T> ret;

    ret.minDelta = std::floor(shiftMin);
    ret.maxDelta = std::ceil(shiftMax);

    ret.accumulatedShifts.resize(shifts.size()+1);
    ret.accumulatedShifts[0] = -ret.minDelta; //start at -minDelta, so that when reaching the minDelta region, the pixels starts at index 0

    for (int i = 1; i <= shifts.size(); i++) {
        ret.accumulatedShifts[i] = ret.accumulatedShifts[i-1] - shifts[i-1];
    }

    if constexpr (meanZero) {
        T mean = 0;
        for (int i = 0; i <= ret.accumulatedShifts.size(); i++) {
            mean += ret.accumulatedShifts[i];
        }
        mean /= ret.accumulatedShifts.size();
        for (int i = 0; i <= ret.accumulatedShifts.size(); i++) {
            ret.accumulatedShifts[i] -= mean;
        }
    }

    return ret;
}

enum class PushBroomCovarianceShiftEstimateFlags {
    WithSubPixel = 1,
    Normalized = 2
};

template<typename T, int flags = int(PushBroomCovarianceShiftEstimateFlags::WithSubPixel)|int(PushBroomCovarianceShiftEstimateFlags::Normalized)>
std::vector<float> estimatePushBroomHorizontalShiftCorr(Multidim::Array<T,3> const& image,
                                                        int searchRangeMin = -3,
                                                        int searchRangeMax = 3,
                                                        int linesAxis = 0,
                                                        int samplesAxis = 1,
                                                        int bandsAxis = 2) {

    auto shape = image.shape();

    int nLines = shape[linesAxis];
    int nSamples = shape[samplesAxis];
    int nBands = shape[bandsAxis];

    constexpr bool withSubPixel = flags & int(PushBroomCovarianceShiftEstimateFlags::WithSubPixel);

    constexpr bool normalized = withSubPixel or //withSubpixel force to normalize
                                (flags & int(PushBroomCovarianceShiftEstimateFlags::Normalized));

    constexpr StereoVision::Correlation::matchingFunctions matchFunc =
        (normalized) ? StereoVision::Correlation::matchingFunctions::NCC :
         StereoVision::Correlation::matchingFunctions::CC;

    using MatchingFuncTraits = StereoVision::Correlation::MatchingFunctionTraits<matchFunc>;

    constexpr bool matchFuncIsScore = MatchingFuncTraits::extractionStrategy == StereoVision::Correlation::dispExtractionStartegy::Score;
    constexpr bool manageViews = false;

    int sampleS = (searchRangeMin < 0) ? -searchRangeMin : 0;
    int sampleE = (searchRangeMax > 0) ? nSamples-searchRangeMax-1 : nSamples-1;

    int nSelectableSamples = sampleE - sampleS + 1;

    if (nSelectableSamples <= 0) {
        return std::vector<float>();
    }

    int nDisp = searchRangeMax - searchRangeMin + 1;

    if (nDisp <= 0) {
        return std::vector<float>();
    }

    int nFeatures = nSelectableSamples*nBands;

    std::vector<float> deltas(nLines-1);

    if (nDisp == 1) {
        std::fill(deltas.begin(), deltas.end(), 0);
        return deltas;
    }

    #pragma omp parallel for
    for (int i = 0; i < nLines-1; i++) {

        Eigen::VectorXf f;
        f.resize(nFeatures);

        int j = 0;
        for (int s = sampleS; s <= sampleE; s++) {
            for (int b = 0; b < nBands; b++) {
                std::array<int,3> idx;
                idx[linesAxis] = i;
                idx[samplesAxis] = s;
                idx[bandsAxis] = b;
                f[j] = image.valueUnchecked(idx);
                j++;
            }
        }

        if (normalized) {
            f.normalize();
        }

        Multidim::Array<T, 1> features(f.data(), {nFeatures}, {1}, manageViews);

        std::vector<double> correlations(nDisp);

        for (int d = searchRangeMin; d <= searchRangeMax; d++) {
            int selectableSamples = nSamples - std::abs(d);

            Eigen::VectorXf t;
            t.resize(nFeatures);

            int j = 0;
            for (int s = sampleS; s <= sampleE; s++) {
                for (int b = 0; b < nBands; b++) {
                    std::array<int,3> idx;
                    idx[linesAxis] = i + 1;
                    idx[samplesAxis] = s + d;
                    idx[bandsAxis] = b;
                    t[j] = image.valueUnchecked(idx);
                    j++;
                }
            }

            if (normalized) {
                t.normalize();
            }

            Multidim::Array<T,1> target(t.data(), {nFeatures}, {1}, manageViews);

            int dId = d - searchRangeMin;

            correlations[dId] = MatchingFuncTraits::featureComparison(features, target);
        }

        float d = 0;
        double current = correlations[0];

        for (int dId = 1; dId < correlations.size(); dId++) {
            if (matchFuncIsScore) {
                if (correlations[dId] > current) {
                    current = correlations[dId];
                    d = dId;
                }
            } else {
                if (correlations[dId] < current) {
                    current = correlations[dId];
                    d = dId;
                }
            }
        }

        d += searchRangeMin;

        if constexpr (withSubPixel) {
            int dI = int(d);

            Eigen::Matrix<float, Eigen::Dynamic, 2> A;
            A.resize(nFeatures, 2);

            Eigen::VectorXf b;
            b.resize(nFeatures);

            for (int f = 0; f < nFeatures; f++) {
                b[f] = features.valueUnchecked(f);
            }

            int j = 0;
            for (int s = sampleS; s <= sampleE; s++) {
                for (int b = 0; b < nBands; b++) {
                    std::array<int,3> idx;
                    idx[linesAxis] = i + 1;
                    idx[samplesAxis] = s + dI;
                    idx[bandsAxis] = b;
                    A(j,0) = image.valueUnchecked(idx);
                    j++;
                }
            }

            if (dI > searchRangeMin) {

                int j = 0;
                for (int s = sampleS; s <= sampleE; s++) {
                    for (int b = 0; b < nBands; b++) {
                        std::array<int,3> idx;
                        idx[linesAxis] = i + 1;
                        idx[samplesAxis] = s + dI - 1;
                        idx[bandsAxis] = b;
                        A(j,1) = image.valueUnchecked(idx);
                        j++;
                    }
                }

                Eigen::Vector2f x = MatchingFuncTraits::barycentricBestApproximation(A,b);

                Eigen::VectorXf newF = A*x;

                if (normalized) {
                    newF.normalize();
                }

                Multidim::Array<float,1> interpolated(newF.data(), {nFeatures}, {1}, manageViews);

                float cand = MatchingFuncTraits::featureComparison(features, interpolated);

                if (matchFuncIsScore) {
                    if (cand > current) {
                        current = cand;
                        d = dI - x[1];
                    }
                } else {
                    if (cand < current) {
                        current = cand;
                        d = dI - x[1];
                    }
                }
            }

            if (dI < searchRangeMax) {

                int j = 0;
                for (int s = sampleS; s <= sampleE; s++) {
                    for (int b = 0; b < nBands; b++) {
                        std::array<int,3> idx;
                        idx[linesAxis] = i + 1;
                        idx[samplesAxis] = s + dI + 1;
                        idx[bandsAxis] = b;
                        A(j,1) = image.valueUnchecked(idx);
                        j++;
                    }
                }

                Eigen::Vector2f x = MatchingFuncTraits::barycentricBestApproximation(A,b);

                Eigen::VectorXf newF = A*x;

                if (normalized) {
                    newF.normalize();
                }

                constexpr bool manage = false;
                Multidim::Array<float,1> interpolated(newF.data(), {nFeatures}, {1}, manage);

                float cand = MatchingFuncTraits::featureComparison(features, interpolated);

                if (matchFuncIsScore) {
                    if (cand > current) {
                        current = cand;
                        d = dI + x[1];
                    }
                } else {
                    if (cand < current) {
                        current = cand;
                        d = dI + x[1];
                    }
                }
            }
        }

        deltas[i] = d;

    }

    return deltas;
}


template<typename ImageT, typename CovKer, typename ComputeT = float>
class CovarianceKernelProbEstimate{
public:

    struct Result {
        ComputeT neglogProb;
        ComputeT dnegLogProb_ddx;
        ComputeT dnegLogProb_ddy;
    };

    static Result logProb(Multidim::Array<ImageT,2> const& patch1,
                          Multidim::Array<ImageT,2> const& patch2,
                          CovKer const& kernel,
                          ComputeT mean,
                          ComputeT sigma0,
                          ComputeT dx, ComputeT dy) {

        int nElements = patch1.shape()[0];
        int nChannels = patch1.shape()[1];

        int mSize = 2*nElements*nChannels;

        using VType = Eigen::Matrix<ComputeT,Eigen::Dynamic,1>;
        using MType = Eigen::Matrix<ComputeT,Eigen::Dynamic,Eigen::Dynamic>;

        VType I(mSize);
        MType Sigma = MType::Zero(mSize,mSize);
        MType dSigma_ddx = MType::Zero(mSize,mSize);
        MType dSigma_ddy = MType::Zero(mSize,mSize);

        for (int i = 0; i < nElements; i++) {
            for (int c = 0; c < nChannels; c++) {
                int patch1Idx = nChannels*i+c;
                int patch2Idx = nElements*nChannels+nChannels*i+c;
                I[patch1Idx] = patch1.valueUnchecked(i,c) - mean;
                I[patch2Idx] = patch2.valueUnchecked(i,c) - mean;
            }

            for (int j = 0; j < nElements; j++) {

                ComputeT dh_row = i-j;
                ComputeT d_row = sqrt(dh_row*dh_row);
                ComputeT dh = i-j+dx;
                ComputeT d_rowline = sqrt(dh*dh + ComputeT(dy*dy));

                ComputeT dd_rowline_ddx = dh/d_rowline;

                ComputeT dd_rowline_ddy = dy/d_rowline;

                for (int c = 0; c < nChannels; c++) {
                    int patch1Idx = nChannels*i+c;
                    int patch2Idx = nElements*nChannels+nChannels*i+c;

                    int patch1Idx2 = nChannels*j+c;
                    int patch2Idx2 = nElements*nChannels+nChannels*j+c;

                    ComputeT sigma_row = kernel(d_row);
                    ComputeT sigma_rowline = kernel(d_rowline);

                    Sigma(patch1Idx,patch1Idx2) = sigma_row;
                    Sigma(patch1Idx2,patch1Idx) = sigma_row;
                    Sigma(patch2Idx,patch2Idx2) = sigma_row;
                    Sigma(patch2Idx2,patch2Idx) = sigma_row;

                    Sigma(patch1Idx,patch2Idx2) = sigma_rowline;
                    Sigma(patch2Idx2,patch1Idx) = sigma_rowline;

                    ComputeT d_sigma = kernel.diff(d_rowline);

                    //pixels on the same row are not influenced by the dx and dy parameters
                    dSigma_ddx(patch1Idx,patch1Idx2) = 0;
                    dSigma_ddx(patch1Idx2,patch1Idx) = 0;
                    dSigma_ddx(patch2Idx,patch2Idx2) = 0;
                    dSigma_ddx(patch2Idx2,patch2Idx) = 0;

                    dSigma_ddy(patch1Idx,patch1Idx2) = 0;
                    dSigma_ddy(patch1Idx2,patch1Idx) = 0;
                    dSigma_ddy(patch2Idx,patch2Idx2) = 0;
                    dSigma_ddy(patch2Idx2,patch2Idx) = 0;

                    dSigma_ddx(patch1Idx,patch2Idx2) = d_sigma*dd_rowline_ddx;
                    dSigma_ddx(patch2Idx2,patch1Idx) = d_sigma*dd_rowline_ddx;

                    dSigma_ddy(patch1Idx,patch2Idx2) = d_sigma*dd_rowline_ddy;
                    dSigma_ddy(patch2Idx2,patch1Idx) = d_sigma*dd_rowline_ddy;

                }
            }
        }

        Sigma *= sigma0;
        dSigma_ddx *= sigma0;
        dSigma_ddy *= sigma0;

        auto decomposition = Sigma.partialPivLu();

        MType SigmaInv = decomposition.inverse();

        Result ret;

        ComputeT det = decomposition.determinant();

        ret.neglogProb = (std::log(det) + I.transpose()*SigmaInv*I);
        // the formula for the derivative is obtained first by linearity to treat the log and quadratic form appart and taking out the 0.5 scale factor
        // d log(f(x)) / d x = 1/x * d f(x) / dx, the derivative of the determinant, via Jacobi's formula, is given by
        // d det(M(x)) / d x = det(M(x)) * tr(M^-1(x) d M(x) / d x) which implies that
        // d log(det(M(x))) / d x = tr(M^-1(x) d M(x) / d x) = sum(M^-T(x) cwiseprod d M(x) / d x) = sum(M^-1(x) cwiseprod d M(x) / d x)
        // d I^T*Sigma(x)^-1*I / d Sigma is given by sum(d I.transpose()*Sigma(x)^-1*I / d Sigma cwiseprod d Sigma(x) / d x)
        // d I^T*Sigma(x)^-1*I / d Sigma is given by Sigma(x)^-1 I I^T Sigma(x)^-1
        // (remember that Sigma(x) = Sigma(x)^T and Sigma(x)^-1 = Sigma(x)^-T, else the relations are not correct)
        // we also used the relation d M^-1(x) / d x = -M^-1(x) d M(x) / d x M^-1(x), which implies that d a^T M^-1 b / d d M = - M^-T a b^T M^-T
        auto diffMat = SigmaInv - SigmaInv*I*I.transpose()*SigmaInv;
        ret.dnegLogProb_ddx = diffMat.cwiseProduct(dSigma_ddx).sum();
        ret.dnegLogProb_ddy = diffMat.cwiseProduct(dSigma_ddy).sum();

        return ret;
    }
};

enum class PushBroomInflexionPointsIntensityPerLineFlags {
    Relative = 1
};

/*!
 * \brief estimateInflexionPointsIntensityPerLine estimate the strenght of inflexions points per line in the image
 * \param image the input push broom image
 * \param countWeight the weighting on the count of inflexion points
 * \param magnitudeWeight the weighting on the intensity of inflexion points
 * \return for each line except the first and last, an estimate of the number/total intensity of the inflexion points on the line.
 *
 * The idea is that a large number of inflexion points is indicative of distortions caused by pitch motion
 * Spike in the number of inflexion points can be detected and then used to rectify the image by skipping lines.
 */
template<typename T, int flags = int(PushBroomInflexionPointsIntensityPerLineFlags::Relative)>
std::vector<float> estimatePushBroomInflexionPointsIntensityPerLine(Multidim::Array<T,3> const& image,
                                                                    Multidim::Array<bool,2> const& mask,
                                                                    float countWeight = 1,
                                                                    float magnitudeWeight = 0,
                                                                    int linesAxis = 0,
                                                                    int samplesAxis = 1,
                                                                    int bandsAxis = 2) {

    auto shape = image.shape();

    int nLines = shape[linesAxis];
    int nSamples = shape[samplesAxis];
    int nBands = shape[bandsAxis];

    std::vector<float> ret(nLines-2);

    #pragma omp parallel for
    for(int i = 1; i < nLines-1; i++) {

        int ri = i-1;

        ret[ri] = 0;

        for (int j = 0; j < nSamples; j++) {

            if (!mask.empty()) {
                std::array<int,2> m_idx;
                m_idx[samplesAxis] = j;

                m_idx[linesAxis] = i;
                int maskCurrent = mask.valueUnchecked(m_idx);
                m_idx[linesAxis] = i-1;
                int maskPrevious = mask.valueUnchecked(m_idx);
                m_idx[linesAxis] = i+1;
                int maskNext = mask.valueUnchecked(m_idx);

                if (!(maskCurrent and maskPrevious and maskNext)) {
                    continue; //all lines need to be within the mask to count this entry
                }
            }

            std::array<int,3> idx;
            idx[samplesAxis] = j;

            for (int c = 0; c < nBands; c++) {

                idx[linesAxis] = i;
                idx[bandsAxis] = c;

                float center = image.valueUnchecked(idx);
                idx[linesAxis] = i-1;
                float prev = image.valueUnchecked(idx);
                idx[linesAxis] = i+1;
                float next = image.valueUnchecked(idx);

                float inflexion_intensity = 0;

                if ((center < prev and center < next) or
                    (center > prev and center > next)) {
                    inflexion_intensity += std::min(std::abs(center - prev), std::abs(center - next));
                } else {
                    inflexion_intensity -= std::min(std::abs(center - prev), std::abs(center - next));
                }

                if (inflexion_intensity > 0) {
                    ret[ri] += countWeight + magnitudeWeight*inflexion_intensity;
                }
            }
        }

        if (flags & static_cast<int>(PushBroomInflexionPointsIntensityPerLineFlags::Relative)) {
            float totalHorizontal = 0;

            for (int j = 1; j < nSamples-1; j++) {

                if (!mask.empty()) {
                    std::array<int,2> m_idx;
                    m_idx[samplesAxis] = j;

                    m_idx[linesAxis] = i;
                    int maskCurrent = mask.valueUnchecked(m_idx);
                    m_idx[linesAxis] = i-1;
                    int maskPrevious = mask.valueUnchecked(m_idx);
                    m_idx[linesAxis] = i+1;
                    int maskNext = mask.valueUnchecked(m_idx);

                    if (!(maskCurrent and maskPrevious and maskNext)) {
                        continue; //all lines need to be within the mask to count this entry
                    }
                }

                std::array<int,3> idx;
                idx[samplesAxis] = j;

                for (int c = 0; c < nBands; c++) {
                    idx[linesAxis] = i;
                    idx[bandsAxis] = c;

                    float center = image.valueUnchecked(idx);
                    idx[samplesAxis] = j-1;
                    float prev = image.valueUnchecked(idx);
                    idx[samplesAxis] = j+1;
                    float next = image.valueUnchecked(idx);

                    float inflexion_intensity = 0;

                    if ((center < prev and center < next) or
                        (center > prev and center > next)) {
                        inflexion_intensity += std::min(std::abs(center - prev), std::abs(center - next));
                    } else {
                        inflexion_intensity -= std::min(std::abs(center - prev), std::abs(center - next));
                    }

                    if (inflexion_intensity > 0) {
                        totalHorizontal += countWeight + magnitudeWeight*inflexion_intensity;
                    }
                }
            }

            ret[ri] /= std::max(1.f,totalHorizontal);
        }
    }

    return ret;

}

template<typename T, int flags = int(PushBroomInflexionPointsIntensityPerLineFlags::Relative)>
std::vector<float> estimatePushBroomInflexionPointsIntensityPerLine(Multidim::Array<T,3> const& image,
                                                                    float countWeight = 1,
                                                                    float magnitudeWeight = 0,
                                                                    int linesAxis = 0,
                                                                    int samplesAxis = 1,
                                                                    int bandsAxis = 2) {
    return estimatePushBroomInflexionPointsIntensityPerLine(image,
                                                            Multidim::Array<bool,2>(),
                                                            countWeight,
                                                            magnitudeWeight,
                                                            linesAxis,
                                                            samplesAxis,
                                                            bandsAxis);
}

/*!
 * \brief filterPushBroomLinesWithInflexion select a subset of lines as to minimize noise caused by
 * \param inflexionData
 * \param peakThreshold
 * \param maxInterval
 * \return
 */
inline std::vector<int> filterPushBroomLinesWithInflexion(std::vector<float> const& inflexionData,
                                                          float peakThreshold,
                                                          int maxInterval) {

    std::vector<int> ret;
    ret.reserve(inflexionData.size()+2);
    ret.push_back(0);

    auto isIndexPeak = [&inflexionData, peakThreshold] (int id) {

        if (inflexionData[id] > peakThreshold) {
            if (inflexionData[id] > inflexionData[std::max<int>(id-1,0)] and
                inflexionData[id] > inflexionData[std::min<int>(id+1,inflexionData.size()-1)]) {
                return true;
            }
        }
        return false;
    };

    for (int i = 0; i < inflexionData.size(); i++) {

        ret.push_back(i+1);

        bool isPeak = isIndexPeak(i);

        if (isPeak) {
            int deltaK1 = 0;
            for (int k = 2; k <= maxInterval; k++) {
                if (isIndexPeak(i+k)) {
                    deltaK1 = k;
                    break;
                }
            }
            int deltaK2 = 0;
            for (int k = 2; k <= maxInterval; k++) {
                if (isIndexPeak(i+deltaK1+k)) {
                    deltaK2 = k;
                    break;
                }
            }

            if (deltaK1 > 0 and deltaK2 > 0) {
                i += deltaK1 + deltaK2;
            }
        }

    }

    ret.push_back(inflexionData.size()+1);
    return ret;
}

struct LineShiftInfos {
    float dx; //! \brief relative shift, in pixel, of the line in the rectified image in the across track direction
    float y; //! \brief absolute position, in pixel, of the line in the rectified image in the along track direction
};

template<typename T, Multidim::ArrayDataAccessConstness constNess>
inline double estimateMaternScaleFromPushBroom(Multidim::Array<T,3,constNess> const& image,
                                               std::vector<double> const& channelsMeans,
                                               std::vector<double> const& channelsVariances,
                                               double initialGuess,
                                               int nIterations = 50,
                                               double residual_treshold = 1e-5,
                                               int linesAxis = 0,
                                               int samplesAxis = 1,
                                               int bandsAxis = 2) {

    //The model we optimize is log(p(Delta I|rho)) = - Delta I^2/sigma^2(rho), where sigma^2 is the matern covariance
    //We are searching for rho maximazing the likelyhood, using numerical method

    if (image.empty()) {
        return initialGuess;
    }

    std::array<int,3> shape = image.shape();

    if (shape[samplesAxis] <= 1) {
        return initialGuess; //canot guess anything with just 1 px
    }

    double guess = std::max<double>(1,initialGuess);
    constexpr double nu = 1.5; //use 3./2. by default;

    using ComputeT = double;
    using ParamsVecT = Eigen::Matrix<ComputeT,1,1>;

    constexpr int M = 1; //memory of the lBGFSMinimizationProblem
    constexpr int NParams = 1;

    class ObjectiveFunction
    {
    public:

        ObjectiveFunction(Multidim::Array<T,3,constNess> const& pImage,
                          std::vector<double> const& pChannelsMean,
                          std::vector<double> const& pChannelsVariances,
                          int pLinesAxis = 0,
                          int pSamplesAxis = 1,
                          int pBandsAxis = 2) :
            image(&pImage),
            channelsMeans(&pChannelsMean),
            channelsVariances(&pChannelsVariances),
            shape(pImage.shape()),
            stepDist(1,shape[pSamplesAxis]/2),
            channelDist(0,shape[pBandsAxis]-1),
            linesAxis(pLinesAxis),
            samplesAxis(pSamplesAxis),
            bandsAxis(pBandsAxis)
        {

        }

        Multidim::Array<T,3,constNess> const* image;
        std::vector<double> const* channelsMeans;
        std::vector<double> const* channelsVariances;
        std::array<int,3> shape;
        mutable std::minstd_rand generator; //use a linear rng engine, should be reasonably fast for our application
        mutable std::uniform_int_distribution<int> stepDist;
        mutable std::uniform_int_distribution<int> channelDist;
        int linesAxis;
        int samplesAxis;
        int bandsAxis;

        ComputeT objective(ParamsVecT const& parameters) const {
            generator.seed(shape[0] + shape[1] + shape[2]);
            double val = 0;

            double d = parameters[0]*parameters[0];

            for (int i = 0; i < shape[linesAxis]; i++) {
                //for each line
                int current = 0;
                int next = stepDist(generator);
                int channel = channelDist(generator);

                do {

                    double sigma0 = channelsVariances->at(channel);

                    double dist = next - current;
                    double I1 = image->valueUnchecked(i,current,channel) - channelsMeans->at(channel);
                    double I2 = image->valueUnchecked(i,next,channel) - channelsMeans->at(channel);

                    double sigma2 = StereoVision::Statistics::CovarianceKernels::Matern<double>::corrFunction(nu, d, dist);

                    double det = 1-sigma2*sigma2;
                    val += std::log(sigma0*det) + (I1*I1 - 2*sigma2*I1*I2 + I2*I2)/(sigma0*det);

                    current = std::max(current+1,(current+next)/2);
                    next = std::min(shape[samplesAxis]-1, current + stepDist(generator));
                    channel = channelDist(generator);

                } while (current < next);

            }

            return val;

        }

        ParamsVecT gradient(ParamsVecT const& parameters) const {
            generator.seed(shape[0] + shape[1] + shape[2]);
            ParamsVecT gradient;
            gradient[0] = 0;

            double d = parameters[0]*parameters[0];

            for (int i = 0; i < shape[linesAxis]; i++) {
                //for each line
                int current = 0;
                int next = stepDist(generator);
                int channel = channelDist(generator);

                do {

                    double sigma0 = channelsVariances->at(channel);

                    double dist = next - current;
                    double I1 = image->valueUnchecked(i,current,channel) - channelsMeans->at(channel);
                    double I2 = image->valueUnchecked(i,next,channel) - channelsMeans->at(channel);

                    double sigma2 = StereoVision::Statistics::CovarianceKernels::Matern<double>::corrFunction(nu, d, dist);
                    double dsigma2 = StereoVision::Statistics::CovarianceKernels::Matern<double>::diffCorrFunctionRho(nu, d, dist);


                    double det = 1-sigma2*sigma2;
                    double d_det = -2*sigma2*dsigma2;

                    //std::log(sigma0*det)
                    gradient[0] += 1/det * d_det;
                    //(I1*I1 - 2*sigma2*I1*I2 + I2*I2)/(sigma0*det);
                    gradient[0] -= I1*I1/(sigma0*det*det) * d_det;
                    gradient[0] -= 2*I1*I2/(sigma0*det) * dsigma2 - 2*sigma2*I1*I2/(sigma0*det*det) * d_det;
                    gradient[0] -= I2*I2/(sigma0*det*det) * d_det;

                    current = std::max(current+1,(current+next)/2);
                    next = std::min(shape[samplesAxis]-1, current + stepDist(generator));
                    channel = channelDist(generator);

                } while (current < next);

            }

            gradient *= 2*parameters[0]; //dd/dx
            return gradient;

        }

        ParamsVecT initialDiagonal(ParamsVecT  const& parameters) const {
            ParamsVecT initial = ParamsVecT(1);
            initial[0] = 1;
            return initial;
        }
    };

    using Optimizer = StereoVision::Optimization::lBFGSMinimizationProblem<ComputeT, ObjectiveFunction, M, NParams>;
    Optimizer opt(image, channelsMeans, channelsVariances,
                  linesAxis,
                  samplesAxis,
                  bandsAxis);

    typename Optimizer::CallBackFunc callBack = typename Optimizer::CallBackFunc();

    ParamsVecT estimate;
    estimate[0] = guess;
    StereoVision::ConvergenceType cType = opt.run(nIterations, residual_treshold, estimate, callBack);

    return opt.solution()[0]*opt.solution()[0];

}

template<typename T, Multidim::ArrayDataAccessConstness constNess>
inline std::tuple<std::vector<double>,std::vector<double>>
estimateGaussianProcessMeanAndVar(Multidim::Array<T,3,constNess> const& image,
                                  int linesAxis = 0,
                                  int samplesAxis = 1,
                                  int bandsAxis = 2) {

    if (image.empty()) {
        return std::make_tuple(std::vector<double>{}, std::vector<double>{});
    }

    std::array<int,3> shape = image.shape();

    std::vector<double> mean(shape[bandsAxis]);
    std::vector<double> ret(shape[bandsAxis]);

    size_t nSamples = 1;
    for (int s : shape) {
        nSamples *= s;
    }

    std::fill(mean.begin(), mean.end(), 0);

    for (int i = 0; i < shape[linesAxis]; i++) {
        for (int j = 0; j < shape[samplesAxis]; j++) {
            for (int c = 0; c < shape[bandsAxis]; c++) {
                double val = image.valueUnchecked(i,j,c);
                mean[c] += val;
            }
        }
    }

    for (int c = 0; c < shape[bandsAxis]; c++) {
        mean[c] /= nSamples;
    }

    std::fill(ret.begin(), ret.end(), 0);

    for (int i = 0; i < shape[linesAxis]; i++) {
        for (int j = 0; j < shape[samplesAxis]; j++) {
            for (int c = 0; c < shape[bandsAxis]; c++) {
                double delta = image.valueUnchecked(i,j,c) - mean[c];
                ret[c] += delta*delta;
            }
        }
    }

    for (int c = 0; c < shape[bandsAxis]; c++) {
        ret[c] /= nSamples;
    }

    return std::make_tuple(mean, ret);
}

template<bool verbose = false, typename T, Multidim::ArrayDataAccessConstness constNess>
StereoVision::IterativeNumericalAlgorithmOutput<std::vector<float>> estimatePushBroomHorizontalShiftBayesian(
    Multidim::Array<T,3,constNess> const& image,
    int hwindow,
    double dx_pos_lambda,
    int nIterations = 500,
    double residual_treshold = 1e-5,
    int linesAxis = 0,
    int samplesAxis = 1,
    int bandsAxis = 2) {

    using RType = StereoVision::IterativeNumericalAlgorithmOutput<std::vector<float>>;
    using ComputeT = double;
    using ParamsVecT = Eigen::Matrix<ComputeT, Eigen::Dynamic, 1>;
    using CovKer = StereoVision::Statistics::CovarianceKernels::Matern<ComputeT>;
    using CovKerProbEstimate = CovarianceKernelProbEstimate<T, CovKer, ComputeT>;

    if (image.empty()) {
        return RType(
            std::vector<float>(),
            StereoVision::ConvergenceType::Failed);
    }

    std::array<int,3> shape = image.shape();

    int nLines = shape[linesAxis];

    std::vector<float> ret(nLines-1);
    StereoVision::ConvergenceType convType = StereoVision::ConvergenceType::Converged;

    int nParams = nLines-1; //only 1 dx per line pairs

    std::vector<float> params;
    params.resize(nParams);

    for (int i = 0; i < nParams; i++) {
        params[i] = 0; //horizontal shifts
    }

    double expectedScale = 10.;
    int scaleNIteration = std::min(50,nIterations);
    double scaleResidualThreshold = std::max(1e-5,residual_treshold);

    auto tuple = estimateGaussianProcessMeanAndVar(image, linesAxis, samplesAxis, bandsAxis);
    std::vector<double>& means = std::get<0>(tuple);
    std::vector<double>& vars = std::get<1>(tuple);

    double optimizedScale = estimateMaternScaleFromPushBroom(image, means, vars, expectedScale,
                                                             scaleNIteration, scaleResidualThreshold,
                                                             linesAxis, samplesAxis, bandsAxis);

    if (verbose) {
        std::cout << "Estimated optimized scale: " << optimizedScale << std::endl;
    }

    if (!std::isfinite(optimizedScale)) {
        optimizedScale = expectedScale;
    }

    constexpr double nu = 1.5; //select by default
    StereoVision::Statistics::CovarianceKernels::Matern<double> kernel(nu, optimizedScale);

    constexpr int M = 20; //memory of the lBGFSMinimizationProblem
    class ObjectiveFunction
    {
    public:

        ObjectiveFunction(
            double pdx_pos_lambda,
            int ps_j,
            int pjJump,
            int phwindow,
            std::array<int,3> const& pshape,
            StereoVision::Statistics::CovarianceKernels::Matern<double> const& pkernel,
            std::vector<double> const& pvars,
            std::vector<double> const& pmeans,
            Multidim::Array<T,2,constNess> const& line1,
            Multidim::Array<T,2,constNess> const& line2,
            int plinesAxis,
            int psamplesAxis,
            int pbandsAxis) :
            dx_pos_lambda(pdx_pos_lambda),
            s_j(ps_j),
            jJump(pjJump),
            hwindow(phwindow),
            shape(pshape),
            kernel(pkernel),
            vars(pvars),
            means(pmeans),
            line1(&line1),
            line2(&line2),
            linesAxis(plinesAxis),
            samplesAxis(psamplesAxis),
            bandsAxis(pbandsAxis)
        {

        }

        double dx_pos_lambda;
        int s_j;
        int jJump;
        int hwindow;
        std::array<int,3> shape;
        StereoVision::Statistics::CovarianceKernels::Matern<double> kernel;
        std::vector<double> vars;
        std::vector<double> means;
        Multidim::Array<T,2,constNess> const* line1;
        Multidim::Array<T,2,constNess> const* line2;
        int linesAxis;
        int samplesAxis;
        int bandsAxis;

        ComputeT objective(ParamsVecT const& parameters) const {

            ComputeT objective = 0;

            T dx = parameters[0];

            //relative priors

            objective += dx_pos_lambda * dx*dx;

            //measurements

            for (int j = s_j; j <= shape[samplesAxis]-jJump; j+= jJump) {

                for (int c = 0; c < shape[bandsAxis]; c++) {
                    Multidim::Array<T,2> patch1 = line1->subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));
                    Multidim::Array<T,2> patch2 = line2->subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));

                    double sigma0 = vars[c];
                    double mean = means[c];

                    auto probData = CovKerProbEstimate::logProb(patch1,
                                                                patch2,
                                                                kernel,
                                                                mean,
                                                                sigma0,
                                                                dx, 1);

                    objective += probData.neglogProb;
                }
            }

            return objective;
        }

        ParamsVecT gradient(ParamsVecT const& parameters) const {

            ParamsVecT gradient = ParamsVecT::Zero(1);

            T dx = parameters[0];

            //relative priors
            gradient[0] += 2*dx_pos_lambda * dx;

            //measurements

            for (int j = s_j; j <= shape[samplesAxis]-jJump; j+= jJump) {

                for (int c = 0; c < shape[bandsAxis]; c++) {
                    Multidim::Array<T,2> patch1 = line1->subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));
                    Multidim::Array<T,2> patch2 = line2->subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));

                    double sigma0 = vars[c];
                    double mean = means[c];

                    auto probData = CovKerProbEstimate::logProb(patch1,
                                                                patch2,
                                                                kernel,
                                                                mean,
                                                                sigma0,
                                                                dx, 1);

                    gradient[0] += probData.dnegLogProb_ddx;
                }
            }

            return gradient;
        }

        //assume that the absolute priors might be an important aspect of the hessian, initialize the guess with them.
        ParamsVecT initialDiagonal(ParamsVecT  const& parameters) const {
            ParamsVecT initial = ParamsVecT(1);
            initial[0] = 1;
            return initial;
        }


    protected:
    };

    constexpr int stepNParams = 1;
    using StepParamsVecT = Eigen::Matrix<ComputeT, stepNParams, 1>;
    using Optimizer = StereoVision::Optimization::lBFGSMinimizationProblem<ComputeT, ObjectiveFunction, M, stepNParams>;

    int s_j = (shape[samplesAxis] % hwindow) / 2;
    int jJump = std::min(hwindow,shape[samplesAxis]);

    for (int i = 0; i < ret.size(); i++) {
        int lineId0 = i;
        int lineId1 = i+1;

        Multidim::Array<T,2,constNess> line1 = image.sliceView(linesAxis,i);
        Multidim::Array<T,2,constNess> line2 = image.sliceView(linesAxis,i+1);

        Optimizer opt(
            dx_pos_lambda,
            s_j,
            jJump,
            hwindow,
            shape,
            kernel,
            vars,
            means,
            line1,
            line2,
            linesAxis,
            samplesAxis,
            bandsAxis);

        typename Optimizer::CallBackFunc callBack = typename Optimizer::CallBackFunc();

        if (verbose) {
            callBack = [i] (Optimizer const& opt) {
                auto gradient = opt.currentGradientValue();
                auto previous_delta = opt.previousSolutionDelta();
                std::cout << "\t\tIteration " << (opt.nIterations()+1) << ": objective = " << opt.objectiveHistory().back()
                          << " gradient norm = " << gradient.norm()
                          << " delta norm = " << previous_delta.norm() << std::endl;
            };
        }

        if (verbose) {
            std::cout << "\tLine " << i << ":\n";
        }

        StepParamsVecT dx;
        dx[0] = 0;
        StereoVision::ConvergenceType cType = opt.run(nIterations, residual_treshold, dx, callBack);

        params[i] = opt.solution()[0];
        convType = std::max(convType, opt.convergenceType());

    }

    return RType( params, convType);

}

/*!
 * \brief estimatePushBroomVerticalReorderBayesian estimate the most likely order of the lines in an image based on a stochastic model
 * \param image the image
 * \param hwindow the horizontal window to compute the stochastic model
 * \param searchRadius the maximal distance any line is considered to still be a neighboor of.
 * \param linesAxis the axis representing sucesive lines
 * \param samplesAxis the axis representing different pixels in the sensor
 * \param bandsAxis the axis representing the channels in the data.
 * \return the estimated order of the lines
 *
 * This function transform the estimation problem into a traveling salesman problem and uses the Bellman–Held–Karp algorithm to solve it.
 * It also uses the assumption that any line cannot be matched past a certain distance to prune the search space a lot, reducing the algorithm complexity.
 *
 * More precisely, assuming the lines in the image are labelled 0 to n, and the start and end point is -1 (and n+1), the order in which the nodes are explored
 * in the traveling salesman solution is the order in which the image lines will be sorted. We enforce that any path finishing at node i has length between
 * i - r and i + r (i.e. enforece that the rectified line index of unrectified line i is between i-r and i+r). This impose that paths of lengths j contains
 * at least all nodes up to j - r. This drastically reduces the number of path to consider in the Bellman–Held–Karp algorithm, as the subset of points that
 * can either be or not be within a path of lenght j is limited between j-r and j+r. So when considering the candidate paths in the forward search of the
 * Bellman–Held–Karp algorithm, we need to consider 2r+1 possible end nodes. Since j+r can only be at the end of a path of lenght j (since it cannot be part of
 * a path smaller than j), and j-r has to be in all paths of lenght j (still needs to be considered as possible end node) only (2r-2)!/() possible intermediate nodes
 * needs to be considered for each end node and each node length.
 *
 * The overall complexity of the modified algorithm is O(l*(2*r+1)(2r-2)!), i.e., it should be able to process large images, but only perform a search at reasonably small range.
 */
template<bool verbose = false, typename T, Multidim::ArrayDataAccessConstness constNess>
std::vector<int> estimatePushBroomVerticalReorderBayesian(
    Multidim::Array<T,3,constNess> const& image,
    std::vector<double> const& horizontalShifts,
    int hwindow,
    int searchRadius,
    int linesAxis = 0,
    int samplesAxis = 1,
    int bandsAxis = 2) {

    using ComputeT = double;
    using ParamsVecT = Eigen::Matrix<ComputeT, Eigen::Dynamic, 1>;
    using CovKer = StereoVision::Statistics::CovarianceKernels::Matern<ComputeT>;
    using CovKerProbEstimate = CovarianceKernelProbEstimate<T, CovKer, ComputeT>;

    auto shape = image.shape();
    int const& nLines = shape[linesAxis];

    if (nLines == 0) {
        return std::vector<int> {};
    }

    if (nLines == 1) {
        return std::vector<int> {0};
    }

    if (nLines == 2) {
        return std::vector<int> {0,1};
    }

    struct internalDetails {

        /*
         * \brief choicesToShifts convert the choices index to actual shifts
         * \param choices the choices vector produced by choicesFromIdx
         * \param dr the current shift under consideration
         * \return the shifts for the node constituting the intermediate node of the considered path
         *
         */
        static inline std::vector<int> choicesToShifts(std::vector<int> const& choices, int dr, int searchRadius) {
            int nExtras = 1;
            if (dr == -searchRadius) {
                nExtras = 0;
            }

            std::vector<int> ret(choices.size()+nExtras);
            int di = 0;

            if (dr != -searchRadius) {
                ret[0] = -searchRadius;
                di = 1;
            }

            for (int i = 0; i < choices.size(); i++) {
                int& current = ret[i+di];

                current = choices[i] - searchRadius + 1;

                if (current >= dr and dr != -searchRadius) {
                    current += 1;
                }
            }
            return ret;
        }
        static inline std::vector<int> shiftsToChoices(std::vector<int> const& shifts, int dr, int searchRadius) {
            int nRedundant = 1;
            if (dr == -searchRadius) {
                nRedundant = 0;
            }
            std::vector<int> ret(shifts.size()-nRedundant);
            int di = (nRedundant > 0) ? 1 : 0;
            for (int i = 0; i < ret.size(); i++) {
                int s = shifts[i+di];
                s += searchRadius-1;

                for (int j = i-1; j >= 0; j++) {
                    if (s >= ret[j]) {
                        s--;
                    }
                }
                ret[i] = s;
            }
            return ret;
        }

    };

    struct PathInfos {
        std::vector<int> intermediateDeltas;
        int finalDelta;
    };

    //the set of of all lines that can be mapped to line j with our constraints has size 2*searchRadius+1,
    // all the lines up to j-searchradius-1 have to have been mapped at that point. (i.e. the nodes in all paths of len j-searchRadius-1 are known)
    // to get a path of lenght j, we still need to choose between searchRadius+1 nodes.

    //in a normal step, we have the node terminating the path, and we need to include the node j-searchRadius
    // (since all path of size j+1 have to contain it, when investigating paths of length j they would discard any path not containing it).
    // finally, if it is not the last node, the node j+searchRadius cannot be present, as it cannot be a part of any path of len smaller than j
    //This mean 2 nodes are already selected (we need only searchRadius-1 more nodes) and the search space is 2*searchRadius-2 nodes
    StereoVision::Combinatorial::ChooseInSetIndexer standardChoiceIndexer(2*searchRadius-2,searchRadius-1);
    int nSubPathsPerStep = standardChoiceIndexer.nChoices();
    //in the case the node j-searchRadius is the last in the path then it does not have to be selected in the intermediate nodes
    //meaning that the selectable nodes sets is 1 node larger and the number of choices we have to do is 1 more
    StereoVision::Combinatorial::ChooseInSetIndexer firstChoiceIndexer(2*searchRadius-1,searchRadius);
    int nSubPathsPerFirstStep = firstChoiceIndexer.nChoices();
    //If the last node is j+searchRadius, the search space increase by 1 node, but we still need to select only searchRadius-1 nodes
    //as j-searchRadius still has to be integrated in the path
    StereoVision::Combinatorial::ChooseInSetIndexer lastChoiceIndexer(2*searchRadius-1,searchRadius-1);
    int nSubPathsPerLastStep = lastChoiceIndexer.nChoices();

    int nPathsPerStep = (2*searchRadius-1)*nSubPathsPerStep + nSubPathsPerFirstStep + nSubPathsPerLastStep;

    std::vector<StereoVision::Combinatorial::ChooseInSetIndexer*> indexerPerDelta(2*searchRadius+1);
    std::fill(indexerPerDelta.begin(), indexerPerDelta.end(), &standardChoiceIndexer);
    indexerPerDelta[0] = &firstChoiceIndexer;
    indexerPerDelta.back() = &lastChoiceIndexer;

    auto drFromPathIdx = [nSubPathsPerStep, nSubPathsPerFirstStep, nSubPathsPerLastStep, searchRadius] (int i) {
        if (i < nSubPathsPerFirstStep) {
            return -searchRadius;
        } else if (i >= nSubPathsPerFirstStep + (2*searchRadius-1)*nSubPathsPerStep) {
            return searchRadius;
        } else {
            return (i - nSubPathsPerFirstStep)/nSubPathsPerStep - searchRadius + 1;
        }
    };

    auto drIdxZero = [nSubPathsPerStep, nSubPathsPerFirstStep, nSubPathsPerLastStep, searchRadius] (int dr) {
        if (dr == -searchRadius) {
            return 0;
        } else if (dr == searchRadius) {
            return nSubPathsPerFirstStep + (2*searchRadius-1)*nSubPathsPerStep;
        } else {
            return nSubPathsPerFirstStep + dr*nSubPathsPerStep;
        }
    };

    //build the cost matrix
    int nComps = 2*searchRadius;

    Multidim::Array<float,2> logProbs(nLines, nComps);

    int s_j = (shape[samplesAxis] % hwindow) / 2;
    int jJump = std::min(hwindow,shape[samplesAxis]);

    auto tuple = estimateGaussianProcessMeanAndVar(image, linesAxis, samplesAxis, bandsAxis);
    std::vector<double>& means = std::get<0>(tuple);
    std::vector<double>& vars = std::get<1>(tuple);

    constexpr int nIterations = 50;
    double expectedScale = 10.;
    double scaleResidualThreshold = 1e-5;

    double optimizedScale = estimateMaternScaleFromPushBroom(image, means, vars, expectedScale,
                                                             nIterations, scaleResidualThreshold,
                                                             linesAxis, samplesAxis, bandsAxis);

    constexpr double nu = 1.5; //select by default
    StereoVision::Statistics::CovarianceKernels::Matern<double> kernel(nu, optimizedScale);

    for (int i = 0; i < nLines; i++) {
        for (int c = 0; c < nComps; c++) {

            int j = i + c + 1;

            if (j >= nLines) {
                logProbs.atUnchecked(i,c) = std::numeric_limits<float>::infinity();
                continue;
            }

            Multidim::Array<T,2,constNess> line1 = image.sliceView(linesAxis,i);
            Multidim::Array<T,2,constNess> line2 = image.sliceView(linesAxis,j);

            //measurements
            double objective = 0;
            double dx = horizontalShifts[i]-horizontalShifts[j];

            for (int j = s_j; j <= shape[samplesAxis]-jJump; j+= jJump) {

                for (int c = 0; c < shape[bandsAxis]; c++) {
                    Multidim::Array<T,2> patch1 = line1.subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));
                    Multidim::Array<T,2> patch2 = line2.subView(Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));

                    double sigma0 = vars[c];
                    double mean = means[c];

                    auto probData = CovKerProbEstimate::logProb(patch1,
                                                                patch2,
                                                                kernel,
                                                                mean,
                                                                sigma0,
                                                                dx, 1);
                    objective += probData.neglogProb;
                }

            }
            logProbs.atUnchecked(i,c) = objective;
        }
    }

    //forward pass
    std::vector<double> previousCosts(nPathsPerStep);
    std::vector<double> currentCosts(nPathsPerStep);

    Multidim::Array<int,2> previousPathIdx(nLines-1, nPathsPerStep);

    //init the costs

    for (int i = 0; i < nPathsPerStep; i++) {
        int finalDelta = drFromPathIdx(i);
        if (finalDelta >= 0) { //any line in the range 0 to search radius can be the first
            previousCosts[i] = 0; //cost is assumed to be 0 to be in first alone
        } else {
            previousCosts[i] = std::numeric_limits<double>::infinity();
        }
    }

    if (verbose) {
        std::cout << "Reorder estimation:" << std::endl;
    }

    for (int pathLen = 1; pathLen < nLines; pathLen++) { //for increasing path len

        if (verbose) {
            std::cout << "\tConsidering path len " << pathLen << std::endl;
        }

        for (int i = 0; i < nPathsPerStep; i++) {

            int dr = drFromPathIdx(i);
            auto& indexer = *(indexerPerDelta[dr+searchRadius]);
            int choiceIdx = i - drIdxZero(dr);
            std::vector<int> choice = indexer.idx2set(choiceIdx);
            int nItems = indexer.itemsSetSize();
            int nChoices = indexer.choiceSetSize();

            int currentLineId = pathLen+dr;

            std::vector<int> intermediateDeltas = internalDetails::choicesToShifts(choice, dr, searchRadius);
            std::vector<int> possiblesSecondToLast(intermediateDeltas.size()+1);
            possiblesSecondToLast[0] = -searchRadius; // the node search radius away is always present
            bool lastIsPresent = false;
            for (int i = 1; i <= intermediateDeltas.size(); i++) {
                int prev = intermediateDeltas[i-1]+1;
                possiblesSecondToLast[i] = prev; //consider the deltas with respect to the last path len, so +1
                if (prev == searchRadius) {
                    lastIsPresent = true;
                }
            }

            double currentCost = std::numeric_limits<double>::infinity();
            int currentBestSecond2LastIdx = -1;

            for (int secToLast : possiblesSecondToLast) { //for all possible second to last nodes

                if (lastIsPresent and secToLast != searchRadius) { //if the last possible node for a path of size j-1 is present, then it has to be the second to last..., as it cannot be part of a path of size j-2
                    continue;
                }

                std::vector<int> previousIntermediateDelta;
                previousIntermediateDelta.reserve(intermediateDeltas.size());

                for (int prev : possiblesSecondToLast) {
                    if (prev == secToLast) {
                        continue;
                    }
                    previousIntermediateDelta.push_back(prev);
                }

                int prevNItems = 2*searchRadius-2;

                if (secToLast == -searchRadius) {
                    prevNItems = 2*searchRadius-1;
                }

                auto& prev_indexer = *(indexerPerDelta[secToLast+searchRadius]);

                auto choice = internalDetails::shiftsToChoices(previousIntermediateDelta, secToLast, searchRadius);
                int prevChoiceIdx = prev_indexer.set2idx(choice);

                int prevIdx = drIdxZero(secToLast) + prevChoiceIdx;


                int secToLastLineId = pathLen-1+secToLast;
                assert(secToLastLineId != currentLineId);

                int minLineId = std::min(secToLastLineId, currentLineId);
                int maxLineId = std::max(secToLastLineId, currentLineId);

                double costCand = previousCosts[prevIdx];

                if (pathLen < nLines) {
                    costCand += logProbs.valueOrAlt({minLineId,maxLineId-minLineId-1},
                                                    std::numeric_limits<double>::infinity());
                }

                if (costCand < currentCost) {
                    currentCost = costCand;
                    currentBestSecond2LastIdx = prevIdx;
                }
            }

            currentCosts[i] = currentCost;

            previousPathIdx.atUnchecked(pathLen-1,i) = currentBestSecond2LastIdx;
        }

        std::swap(currentCosts, previousCosts);

    }

    //reconstruct path backward

    int lastId = -1;
    double bestCost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < nPathsPerStep; i++) {
        if (previousCosts[i] < bestCost) {
            bestCost = previousCosts[i];
            lastId = i;
        }
    }

    std::vector<int> path;
    path.reserve(nLines);
    for (int pathLen = nLines-1; pathLen >= 0; pathLen--) {

        int dr = drFromPathIdx(lastId);
        int lineId = pathLen+dr;
        path.push_back(lineId);
        if (pathLen > 0) {
            lastId = previousPathIdx.valueUnchecked(pathLen-1,lastId);
        }
    }

    std::reverse(path.begin(), path.end());

    return path;

}

template<bool verbose = false, typename T, Multidim::ArrayDataAccessConstness constNess>
inline StereoVision::IterativeNumericalAlgorithmOutput<std::vector<LineShiftInfos>>
estimateGlobalPushBroomPreRectification(
    Multidim::Array<T,3,constNess> const& image,
    int hwindow,
    int vradius,
    double x_pos_lambda,
    double y_pos_lambda,
    double dx_pos_lambda,
    double dy_pos_lambda,
    double I_lambda,
    int nIterations = 500,
    double residual_treshold = 1e-5,
    double damping_factor = 1.,
    int linesAxis = 0,
    int samplesAxis = 1,
    int bandsAxis = 2)
{

    using RType = StereoVision::IterativeNumericalAlgorithmOutput<std::vector<LineShiftInfos>>;
    using ComputeT = double;
    using ParamsVecT = Eigen::Matrix<ComputeT, Eigen::Dynamic, 1>;
    using CovKer = StereoVision::Statistics::CovarianceKernels::Matern<ComputeT>;
    using CovKerProbEstimate = CovarianceKernelProbEstimate<T, CovKer, ComputeT>;

    if (image.empty()) {
        return RType(
            std::vector<LineShiftInfos>(),
            StereoVision::ConvergenceType::Failed);
    }

    std::array<int,3> shape = image.shape();

    int s_j = (shape[samplesAxis]%hwindow)/2;
    int jJump = std::min(hwindow,shape[samplesAxis]);

    int nDirParams = shape[linesAxis];
    int nParams = 2*nDirParams; //on vertical and horizontal param per line

    ParamsVecT params;
    params.resize(nParams);

    //initialize the optimized with horizontally undistorted image
    auto horizontalRectificationDataInfos = estimatePushBroomHorizontalShiftBayesian<verbose>(
        image,
        hwindow,
        dx_pos_lambda,
        nIterations,
        residual_treshold,
        linesAxis,
        samplesAxis,
        bandsAxis);

    if (verbose) {
        std::cout << '\n';
    }

    constexpr bool meanZero = true;
    auto accumulated = computeAccumulatedFromRelativeShifts<meanZero>(horizontalRectificationDataInfos.value());

    for (int i = 0; i < nDirParams; i++) {
        params[i] = -accumulated.accumulatedShifts[i]; //horizontal shifts
        params[i+nDirParams] = i; //vertical position
    }

    double expectedScale = 10.;
    int scaleNIteration = std::min(50,nIterations);
    double scaleResidualThreshold = std::max(1e-5,residual_treshold);

    auto tuple = estimateGaussianProcessMeanAndVar(image, linesAxis, samplesAxis, bandsAxis);
    std::vector<double>& means = std::get<0>(tuple);
    std::vector<double>& vars = std::get<1>(tuple);

    double optimizedScale = estimateMaternScaleFromPushBroom(image, means, vars, expectedScale,
                                                             scaleNIteration, scaleResidualThreshold,
                                                             linesAxis, samplesAxis, bandsAxis);

    if (verbose) {
        std::cout << "Estimated optimized scale: " << optimizedScale << std::endl;
    }

    if (!std::isfinite(optimizedScale)) {
        optimizedScale = expectedScale;
    }

    constexpr double nu = 1.5; //select by default
    StereoVision::Statistics::CovarianceKernels::Matern<double> kernel(nu, optimizedScale);

    constexpr double minVar = 1e-2;
    for (int i = 0; i < vars.size(); i++) {
        vars[i] = std::max(vars[i],minVar);
    }

    if (verbose) {
        std::cout << "Estimated channels variances:";
        for (int i = 0; i < vars.size(); i++) {
            std::cout << ' ' << i << ":" << vars[i];
        }
        std::cout << std::endl;
    }

    constexpr int M = 20; //memory of the lBGFSMinimizationProblem
    class ObjectiveFunction
    {
    public:

        ObjectiveFunction(
            int pnParams,
            int pnDirParams,
            double px_pos_lambda,
            double py_pos_lambda,
            double pdx_pos_lambda,
            double pdy_pos_lambda,
            double pI_lambda,
            int ps_j,
            int pjJump,
            int pvradius,
            int phwindow,
            std::array<int,3> const& pshape,
            StereoVision::Statistics::CovarianceKernels::Matern<double> const& pkernel,
            std::vector<double> const& pvars,
            std::vector<double> const& pmeans,
            Multidim::Array<T,3,constNess> const& pimage,
            int plinesAxis,
            int psamplesAxis,
            int pbandsAxis) :
            nParams(pnParams),
            nDirParams(pnDirParams),
            x_pos_lambda(px_pos_lambda),
            y_pos_lambda(py_pos_lambda),
            dx_pos_lambda(pdx_pos_lambda),
            dy_pos_lambda(pdy_pos_lambda),
            I_lambda(pI_lambda),
            s_j(ps_j),
            jJump(pjJump),
            vradius(pvradius),
            hwindow(phwindow),
            shape(pshape),
            kernel(pkernel),
            vars(pvars),
            means(pmeans),
            image(&pimage),
            linesAxis(plinesAxis),
            samplesAxis(psamplesAxis),
            bandsAxis(pbandsAxis)
        {

        }

        int nParams;
        int nDirParams;
        double x_pos_lambda;
        double y_pos_lambda;
        double dx_pos_lambda;
        double dy_pos_lambda;
        double I_lambda;
        int s_j;
        int jJump;
        int vradius;
        int hwindow;
        std::array<int,3> shape;
        StereoVision::Statistics::CovarianceKernels::Matern<double> kernel;
        std::vector<double> vars;
        std::vector<double> means;
        Multidim::Array<T,3,constNess> const* image;
        int linesAxis;
        int samplesAxis;
        int bandsAxis;

        ComputeT objective(ParamsVecT const& parameters) const {

            ComputeT objective = 0;

            //absolute priors
            for (int i = 0; i < nDirParams; i++) {
                int idx;
                //lambda_x * xi^2
                idx = i;
                objective += x_pos_lambda*(parameters[idx]*parameters[idx]);

                //lambda_y * (y_i - i)^2
                idx = i+nDirParams;
                T tmp = (parameters[idx] - i);
                objective += y_pos_lambda*tmp*tmp;
            }

            //relative priors
            for (int i = 0; i < nDirParams-1; i++) {
                int idx1;
                int idx2;
                //lambda_dx * (xi+1 - xi)^2
                idx1 = i;
                idx2 = i+1;
                T tmp = (parameters[idx2] - parameters[idx1]);
                objective += dx_pos_lambda * tmp*tmp;

                //lambda_dy * (yi+1 - yi - 1)^2;
                idx1 = i+nDirParams;
                idx2 = i+1+nDirParams;
                tmp = (parameters[idx2] - parameters[idx1] - 1);
                objective += dy_pos_lambda * tmp*tmp;
            }

            //measurements
            for (int i = 0; i < shape[linesAxis]; i++) {
                int idx1 = i;

                for (int di = 1; i+di < shape[linesAxis] and di <= vradius; di++) {
                    int idx2 = i+di;
                    std::array<int,4> idxs = {idx1,idx2,idx1+nDirParams,idx2+nDirParams};

                    for (int j = s_j; j <= shape[samplesAxis]-jJump; j+= jJump) {

                        double dx = parameters[idx2] - parameters[idx1];
                        double dy = parameters[idx2+nDirParams] - parameters[idx1+nDirParams];

                        double ddx_dx1 = -1;
                        double ddx_dx2 = 1;

                        double ddy_dy1 = -1;
                        double ddy_dy2 = 1;


                        for (int c = 0; c < shape[bandsAxis]; c++) {
                            Multidim::Array<T,2> patch1 = image->subView(Multidim::DimIndex(i), Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));
                            Multidim::Array<T,2> patch2 = image->subView(Multidim::DimIndex(i+di), Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));

                            double sigma0 = vars[c];
                            double mean = means[c];

                            auto probData = CovKerProbEstimate::logProb(patch1,
                                                                        patch2,
                                                                        kernel,
                                                                        mean,
                                                                        sigma0,
                                                                        dx, dy);

                            objective += I_lambda*probData.neglogProb;
                        }
                    }
                }
            }

            return objective;
        }

        ParamsVecT gradient(ParamsVecT const& parameters) const {
            ParamsVecT gradient = ParamsVecT::Zero(nParams);

            //absolute priors
            for (int i = 0; i < nDirParams; i++) {
                int idx;
                //lambda_x * xi^2
                idx = i;
                gradient[idx] += 2*x_pos_lambda*parameters[idx];

                //lambda_y * (y_i - i)^2
                idx = i+nDirParams;
                T tmp = (parameters[idx] - i);
                gradient[idx] += 2*y_pos_lambda*tmp;
            }

            //relative priors
            for (int i = 0; i < nDirParams-1; i++) {
                int idx1;
                int idx2;
                //lambda_dx * (xi+1 - xi)^2
                idx1 = i;
                idx2 = i+1;
                T tmp = (parameters[idx2] - parameters[idx1]);
                gradient[idx1] -= 2*dx_pos_lambda * tmp;
                gradient[idx2] += 2*dx_pos_lambda * tmp;

                //lambda_dy * (yi+1 - yi - 1)^2;
                idx1 = i+nDirParams;
                idx2 = i+1+nDirParams;
                tmp = (parameters[idx2] - parameters[idx1] - 1);
                gradient[idx1] -= 2*dx_pos_lambda * tmp;
                gradient[idx2] += 2*dx_pos_lambda * tmp;
            }

            //measurements
            for (int i = 0; i < shape[linesAxis]; i++) {
                int idx1 = i;

                for (int di = 1; i+di < shape[linesAxis] and di <= vradius; di++) {
                    int idx2 = i+di;
                    std::array<int,4> idxs = {idx1,idx2,idx1+nDirParams,idx2+nDirParams};

                    for (int j = s_j; j <= shape[samplesAxis]-jJump; j+= jJump) {

                        double dx = parameters[idx2] - parameters[idx1];
                        double dy = parameters[idx2+nDirParams] - parameters[idx1+nDirParams];

                        double ddx_dx1 = -1;
                        double ddx_dx2 = 1;

                        double ddy_dy1 = -1;
                        double ddy_dy2 = 1;


                        for (int c = 0; c < shape[bandsAxis]; c++) {
                            Multidim::Array<T,2> patch1 = image->subView(Multidim::DimIndex(i), Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));
                            Multidim::Array<T,2> patch2 = image->subView(Multidim::DimIndex(i+di), Multidim::DimSlice(j,j+hwindow), Multidim::DimSlice(c,c+1));

                            double sigma0 = vars[c];
                            double mean = means[c];

                            auto probData = CovKerProbEstimate::logProb(patch1,
                                                                        patch2,
                                                                        kernel,
                                                                        mean,
                                                                        sigma0,
                                                                        dx, dy);

                            double df_d_dx1 = probData.dnegLogProb_ddx * ddx_dx1;
                            double df_d_dx2 = probData.dnegLogProb_ddx * ddx_dx2;
                            double df_d_dy1 = probData.dnegLogProb_ddy * ddy_dy1;
                            double df_d_dy2 = probData.dnegLogProb_ddy * ddy_dy2;


                            gradient[idx1] += I_lambda*df_d_dx1;
                            gradient[idx1+nDirParams] += I_lambda*df_d_dy1;
                            gradient[idx2] += I_lambda*df_d_dx2;
                            gradient[idx2+nDirParams] += I_lambda*df_d_dy2;
                        }
                    }
                }
            }

            return gradient;
        }

        //assume that the absolute priors might be an important aspect of the hessian, initialize the guess with them.
        ParamsVecT initialDiagonal(ParamsVecT  const& parameters) const {
            ParamsVecT initial = ParamsVecT(nParams);

            //consider only absolute priors when estimating the initial inverse hessian
            for (int i = 0; i < nDirParams; i++) {
                int idx;
                //lambda_x * xi^2
                idx = i;
                initial[idx] = 1./std::max<double>(1,x_pos_lambda);

                //lambda_y * (y_i - i)^2
                idx = i+nDirParams;
                initial[idx] = 1./std::max<double>(1,y_pos_lambda);
            }
            return initial;
        }


    protected:
    };

    using Optimizer = StereoVision::Optimization::lBFGSMinimizationProblem<ComputeT, ObjectiveFunction, M, Eigen::Dynamic>;

    Optimizer opt(
        nParams,
        nDirParams,
        x_pos_lambda,
        y_pos_lambda,
        dx_pos_lambda,
        dy_pos_lambda,
        I_lambda,
        s_j,
        jJump,
        vradius,
        hwindow,
        shape,
        kernel,
        vars,
        means,
        image,
        linesAxis,
        samplesAxis,
        bandsAxis);

    typename Optimizer::CallBackFunc callBack = typename Optimizer::CallBackFunc();

    if (verbose) {
        callBack = [] (Optimizer const& opt) {
            auto gradient = opt.currentGradientValue();
            auto previous_delta = opt.previousSolutionDelta();
            std::cout << "\tIteration " << (opt.nIterations()+1) << ": objective = " << opt.objectiveHistory().back()
                      << " gradient norm = " << gradient.norm()
                      << " delta norm = " << previous_delta.norm() << std::endl;
        };
    }

    StereoVision::ConvergenceType cType = opt.run(nIterations, residual_treshold, params, callBack);

    params = opt.solution();

    std::vector<LineShiftInfos> optimized(nDirParams);

    for (int i = 0; i < nDirParams; i++) {
        optimized[i] = LineShiftInfos{.dx = -float(params[i]), .y = float(params[i+nDirParams])};
    }

    return RType(optimized, cType);

}


template<bool shiftsAreAccumulated = false, typename T>
Multidim::Array<T,3> computeHorizontallyRectifiedImage(Multidim::Array<T,3> const& image,
                                                        std::vector<float> const& shifts,
                                                        int linesAxis = 0,
                                                        int samplesAxis = 1,
                                                        int bandsAxis = 2)
{
    auto shape = image.shape();

    int nLines = shape[linesAxis];
    int nSamples = shape[samplesAxis];
    int nBands = shape[bandsAxis];

    int outWidth = nSamples;

    AccumulatedShiftsInfos<float> accumulated;

    if (shiftsAreAccumulated) {

        accumulated.accumulatedShifts = shifts;
        accumulated.minDelta = shifts[0];
        accumulated.maxDelta = shifts[0];

        for (int i = 1; i < shifts.size(); i++) {
            accumulated.minDelta = std::min<float>(accumulated.minDelta, shifts[i]);
            accumulated.maxDelta = std::max<float>(accumulated.maxDelta, shifts[i]);
        }

    } else {

        accumulated = computeAccumulatedFromRelativeShifts(shifts);
    }

    int delta_w = accumulated.maxDelta - accumulated.minDelta;

    outWidth += delta_w;

    std::array<int,3> out_shape;

    out_shape[linesAxis] = nLines;
    out_shape[samplesAxis] = outWidth;
    out_shape[bandsAxis] = nBands;

    Multidim::Array<T,3> out(out_shape);

    #pragma omp parallel for
    for (int i = 0; i < nLines; i++) {

        float& accumulatedShift = accumulated.accumulatedShifts[i];

        for (int b = 0; b < nBands; b++) {

            std::array<int,2> subCoords;
            subCoords[(linesAxis > samplesAxis) ? linesAxis - 1 : linesAxis] = i;
            subCoords[(bandsAxis > samplesAxis) ? bandsAxis - 1 : bandsAxis] = b;

            Multidim::Array<T,1> samples = image.indexDimView(samplesAxis, subCoords);

            for (int j = 0; j < outWidth; j++) {
                float fracCoord = j - accumulatedShift;

                std::array<int,3> outCoords;
                outCoords[linesAxis] = i;
                outCoords[samplesAxis] = j;
                outCoords[bandsAxis] = b;

                constexpr T(*kernel)(std::array<T,1> const&) = StereoVision::Interpolation::pyramidFunction<T,1>;
                constexpr int kernelRadius = 1;
                constexpr StereoVision::Interpolation::BorderCondition bCond = StereoVision::Interpolation::BorderCondition::Zero;

                T interpolated = StereoVision::Interpolation::interpolateValue<1, T, kernel, kernelRadius, bCond>
                    (samples, {fracCoord});

                out.atUnchecked(outCoords) = interpolated;

            }
        }

    }

    return out;
}

template<typename T, Multidim::ArrayDataAccessConstness constNess>
Multidim::Array<T,3> computeHorizontallyRectifiedVerticallyReorderedImage(Multidim::Array<T,3,constNess> const& image,
                                                                           std::vector<LineShiftInfos> const& shifts,
                                                                           int linesAxis = 0,
                                                                           int samplesAxis = 1,
                                                                           int bandsAxis = 2)
{

    if (image.empty()) {
        return image;
    }

    auto shape = image.shape();

    int nLines = shape[linesAxis];
    int nSamples = shape[samplesAxis];
    int nBands = shape[bandsAxis];

    int outWidth = nSamples;

    float maxDelta = 0;
    float minDelta = 0;

    for (auto const& shiftInfo : shifts) {
        maxDelta = std::max(maxDelta, shiftInfo.dx);
        minDelta = std::min(minDelta, shiftInfo.dx);
    }

    int delta_w = static_cast<int>(std::ceil(maxDelta)) -
                  static_cast<int>(std::floor(minDelta));

    int baseOffset = std::floor(minDelta);
    outWidth += delta_w;

    std::vector<int> reorderedIdxs(shifts.size());
    for (int i = 0; i < reorderedIdxs.size(); i++) {
        reorderedIdxs[i] = i;
    }

    std::sort(reorderedIdxs.begin(), reorderedIdxs.end(), [&shifts] (int i1, int i2) {
        return shifts[i1].y < shifts[i2].y;
    });

    std::array<int,3> out_shape;

    out_shape[linesAxis] = nLines;
    out_shape[samplesAxis] = outWidth;
    out_shape[bandsAxis] = nBands;

    Multidim::Array<T,3> out(out_shape);

#pragma omp parallel for
    for (int i = 0; i < nLines; i++) {

        int idx = reorderedIdxs[i];

        float const& accumulatedShift = shifts[idx].dx;

        for (int b = 0; b < nBands; b++) {

            std::array<int,2> subCoords;
            subCoords[(linesAxis > samplesAxis) ? linesAxis - 1 : linesAxis] = idx;
            subCoords[(bandsAxis > samplesAxis) ? bandsAxis - 1 : bandsAxis] = b;

            Multidim::Array<T,1> samples = image.indexDimView(samplesAxis, subCoords);

            for (int j = 0; j < outWidth; j++) {
                float fracCoord = j + baseOffset - accumulatedShift;

                std::array<int,3> outCoords;
                outCoords[linesAxis] = idx;
                outCoords[samplesAxis] = j;
                outCoords[bandsAxis] = b;

                constexpr T(*kernel)(std::array<T,1> const&) = StereoVision::Interpolation::pyramidFunction<T,1>;
                constexpr int kernelRadius = 1;
                constexpr StereoVision::Interpolation::BorderCondition bCond = StereoVision::Interpolation::BorderCondition::Zero;

                T interpolated = StereoVision::Interpolation::interpolateValue<1, T, kernel, kernelRadius, bCond>
                    (samples, {fracCoord});

                out.atUnchecked(outCoords) = interpolated;

            }
        }

    }

    return out;
}


template<typename T>
Multidim::Array<bool,2> computeHorizontallyRectifiedImageMask(Multidim::Array<T,3> const& image,
                                                        std::vector<float> const& shifts,
                                                        int linesAxis = 0,
                                                        int samplesAxis = 1,
                                                        int bandsAxis = 2)
{

    int maskLineAxis = linesAxis;
    int maskSampleAxis = samplesAxis;

    if (bandsAxis < linesAxis) {
        maskLineAxis--;
    }

    if (bandsAxis < samplesAxis) {
        maskSampleAxis--;
    }

    auto shape = image.shape();

    int nLines = shape[linesAxis];
    int nSamples = shape[samplesAxis];

    int outWidth = nSamples;

    AccumulatedShiftsInfos<float> accumulated = computeAccumulatedFromRelativeShifts(shifts);

    int delta_w = accumulated.maxDelta - accumulated.minDelta;

    outWidth += delta_w;

    std::array<int,2> out_shape;

    out_shape[maskLineAxis] = nLines;
    out_shape[maskSampleAxis] = outWidth;

    Multidim::Array<bool,2> out(out_shape);

    #pragma omp parallel for
    for (int i = 0; i < nLines; i++) {

        float& accumulatedShift = accumulated.accumulatedShifts[i];

        for (int j = 0; j < outWidth; j++) {

            float fracCoord = j - accumulatedShift;

            std::array<int,2> outCoords;
            outCoords[maskLineAxis] = i;
            outCoords[maskSampleAxis] = j;

            out.atUnchecked(outCoords) = fracCoord >= 0 and fracCoord < nSamples;
        }

    }

    return out;
}

template<typename T>
Multidim::Array<T,3> removeUnfilteredLinesFromPushBroomImage(Multidim::Array<T,3> const& image,
                                                              std::vector<int> const& selectedLines,
                                                              int linesAxis = 0,
                                                              int samplesAxis = 1,
                                                              int bandsAxis = 2) {
    auto shapeIn = image.shape();

    int nLines = selectedLines.size();
    int nSamples = shapeIn[samplesAxis];
    int nBands = shapeIn[bandsAxis];

    std::array<int,3> shapeOut;

    shapeOut[linesAxis] = nLines;
    shapeOut[samplesAxis] = nSamples;
    shapeOut[bandsAxis] = nBands;

    Multidim::Array<T,3> out(shapeOut);

    #pragma omp parallel for
    for (int i = 0; i < nLines; i++) {

        int l = selectedLines[i];

        std::array<int,3> idxIn;
        idxIn[linesAxis] = l;

        std::array<int,3> idxOut;
        idxOut[linesAxis] = i;

        for (int j = 0; j < nSamples; j++) {

            idxIn[samplesAxis] = j;
            idxOut[samplesAxis] = j;

            for (int b = 0; b < nBands; b++) {

                idxIn[bandsAxis] = b;
                idxOut[bandsAxis] = b;

                out.atUnchecked(idxOut) =  image.valueUnchecked(idxIn);
            }
        }

    }

    return out;

}

class RelativeShiftConverter {

public:

    RelativeShiftConverter();
    template<typename VLineIterator, typename HLinesIterator>
    RelativeShiftConverter(VLineIterator vLineMatchBegin, VLineIterator vLineMatchEnd,
                           HLinesIterator hLineBegin, HLinesIterator hLineEnd) :
        _verticalLinesMatch(vLineMatchBegin, vLineMatchEnd),
        _horizontalLinesOffsets(hLineBegin, hLineEnd)
    {

    }
    RelativeShiftConverter(std::vector<int> const& vLineMatch,
                           std::vector<float> const& hLines) :
        _verticalLinesMatch(vLineMatch),
        _horizontalLinesOffsets(hLines)
    {

    }
    RelativeShiftConverter(std::vector<int> && vLineMatch,
                           std::vector<float> && hLines) :
        _verticalLinesMatch(vLineMatch),
        _horizontalLinesOffsets(hLines)
    {

    }

    RelativeShiftConverter(RelativeShiftConverter const& other) = default;
    RelativeShiftConverter(RelativeShiftConverter && other) = default;

    RelativeShiftConverter& operator=(RelativeShiftConverter const& other) = default;
    RelativeShiftConverter& operator=(RelativeShiftConverter && other) = default;

    /*!
     * \brief mapToOriginalSequence
     * \param source the point in the rectified space (must be of a type with operator[] declared source[0] is x coordinate, source[1] is y.
     * \return the point in the original space
     */
    template<typename PtT>
    std::array<float,2> mapToOriginalSequence(PtT const& source) const {
        int line = std::round(source[1]);

        int deltaLine = 0;

        if (line < 0) {
            deltaLine = line;
            line = 0;
        } else if (line >= _verticalLinesMatch.size()) {
            deltaLine = line - _verticalLinesMatch.size() + 1;
            line = _verticalLinesMatch.size()-1;
        }

        int originalLine = _verticalLinesMatch[line];

        int horizontalOffsetId = std::max(0, std::min<int>(_horizontalLinesOffsets.size()-1, originalLine));

        std::array<float,2> ret {float(source[0] - _horizontalLinesOffsets[horizontalOffsetId]),
                                 float(originalLine + deltaLine)};

        return ret;
    }

    /*!
     * \brief mapToOriginalSequence
     * \param source the point in the original space (must be of a type with operator[] declared source[0] is x coordinate, source[1] is y.
     * \return the point in the rectified space
     */
    template<typename PtT>
    std::array<float,2> mapFromOriginalSequence(PtT const& source) const {
        int line = std::round(source[1]);

        float y = line;

        if (line > 0 and line < _verticalLinesMatch.back()) {

            for (int i = 1; i < _verticalLinesMatch.size(); i++) {
                if (_verticalLinesMatch[i-1] <= line and _verticalLinesMatch[i] >= line) {
                    float alpha = float(line - _verticalLinesMatch[i-1])/
                                  float(_verticalLinesMatch[i] - _verticalLinesMatch[i-1]);

                    y = i-1+alpha;
                }
            }

        } else if (line >= _verticalLinesMatch.back()) {
            int delta = line - _verticalLinesMatch.back();
            y = _verticalLinesMatch.size() + delta;
        }

        int horizontalOffsetId = std::max(0, std::min<int>(_horizontalLinesOffsets.size()-1, line));

        float x = source[0] + _horizontalLinesOffsets[horizontalOffsetId];

        return std::array<float,2>{x,y};
    }

protected:

    std::vector<int> _verticalLinesMatch;
    std::vector<float> _horizontalLinesOffsets;
};

} // namespace PushBroomRelativeOffsets

} // namespace PikaLTools

#endif // RELATIVEOFFSETSESTIMATOR_H
