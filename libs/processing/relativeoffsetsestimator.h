#ifndef RELATIVEOFFSETSESTIMATOR_H
#define RELATIVEOFFSETSESTIMATOR_H

//this header file contain functions for estimating relative shifts between push-broom data lines to pre-rectify the images (e.g. for tie points search).

#include <vector>

#include <Eigen/Core>
#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/correlation/matching_costs.h>
#include <StereoVision/interpolation/interpolation.h>

namespace PikaLTools {

namespace PushBroomRelativeOffsets {

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

            for (int c = 0; c < nBands; c++) {
                std::array<int,3> idx;
                idx[linesAxis] = i;
                idx[samplesAxis] = j;
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

                for (int c = 0; c < nBands; c++) {
                    std::array<int,3> idx;
                    idx[linesAxis] = i;
                    idx[samplesAxis] = j;
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

} // namespace PushBroomRelativeOffsets

} // namespace PikaLTools

#endif // RELATIVEOFFSETSESTIMATOR_H
