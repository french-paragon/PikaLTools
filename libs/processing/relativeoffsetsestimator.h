#ifndef RELATIVEOFFSETSESTIMATOR_H
#define RELATIVEOFFSETSESTIMATOR_H

//this header file contain functions for estimating relative shifts between push-broom data lines to pre-rectify the images (e.g. for tie points search).

#include <vector>

#include <Eigen/Core>
#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/correlation/matching_costs.h>
#include <StereoVision/interpolation/interpolation.h>

namespace PikaLTools {

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

template<typename T>
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

    float shiftMin = 0;
    float shiftMax = 0;
    float accumulatedShift = 0;

    for (float shift : shifts) {
        accumulatedShift -= shift; //disparity of +n pix mean the object is further on the right, so the line has to be pushed on the left to compensate.
        shiftMin = std::min(shiftMin, accumulatedShift);
        shiftMax = std::max(shiftMax, accumulatedShift);
    }

    int minDelta = std::floor(shiftMin);
    int maxDelta = std::ceil(shiftMax);

    int delta_w = maxDelta - minDelta;

    outWidth += delta_w;

    std::array<int,3> out_shape;

    out_shape[linesAxis] = nLines;
    out_shape[samplesAxis] = outWidth;
    out_shape[bandsAxis] = nBands;

    Multidim::Array<T,3> out(out_shape);

    accumulatedShift = -minDelta; //start at -minDelta, so that when reaching the minDelta region, the pixels starts at index 0

    #pragma omp parallel for
    for (int i = 0; i < nLines; i++) {
        float shift = 0;

        if (i > 0) {
            shift = shifts[i-1];
        }

        accumulatedShift -= shift;

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

} // namespace PikaLTools

#endif // RELATIVEOFFSETSESTIMATOR_H
