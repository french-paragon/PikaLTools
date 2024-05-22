#ifndef PIKALTOOLS_DARKFRAMESTOOLS_H
#define PIKALTOOLS_DARKFRAMESTOOLS_H

#include <MultidimArrays/MultidimArrays.h>
#include <MultidimArrays/MultidimIndexManipulators.h>

#include <StereoVision/utils/types_manipulations.h>

namespace PikaLTools {

template <typename T>
Multidim::Array<T,2> averageDarkFrame(Multidim::Array<T,3> const& darkFrame, int axis = 0) {

    using CT = StereoVision::TypesManipulations::accumulation_extended_t<T>;

    std::array<int,2> dFShape;
    std::array<int,2> dFIndex;

    if (axis == 0) {
        dFShape = {darkFrame.shape()[1],darkFrame.shape()[2]};
    }

    if (axis == 1) {
        dFShape = {darkFrame.shape()[0],darkFrame.shape()[2]};
    }

    if (axis == 2) {
        dFShape = {darkFrame.shape()[0],darkFrame.shape()[1]};
    }

    Multidim::Array<CT,2> avgDf(dFShape);


    for (int i = 0; i < avgDf.shape()[0]; i++) {
        for (int j = 0; j < avgDf.shape()[1]; j++) {
            avgDf.atUnchecked(i,j) = 0;
        }
    }


    int n = darkFrame.shape()[axis];

    if (darkFrame.shape()[axis] == 0) {
        return avgDf.template cast<T>();
    }

    for (int i = 0; i < darkFrame.shape()[0]; i++) {
        for (int j = 0; j < darkFrame.shape()[1]; j++) {
            for (int k = 0; k < darkFrame.shape()[2]; k++) {

                if (axis == 0) {
                    dFIndex = {j,k};
                }

                if (axis == 1) {
                    dFIndex = {i,k};
                }

                if (axis == 2) {
                    dFIndex = {i,j};
                }

                avgDf.atUnchecked(dFIndex) += darkFrame.valueUnchecked(i,j,k);

            }
        }
    }

    for (int i = 0; i < avgDf.shape()[0]; i++) {
        for (int j = 0; j < avgDf.shape()[1]; j++) {
            avgDf.atUnchecked(i,j) /= n;
        }
    }

    return avgDf.template cast<T>();

}

template <typename T>
Multidim::Array<T,3> subtractDarkFrame(Multidim::Array<T,3> const& image, Multidim::Array<T,2> const& darkFrame, int axis = 0) {

    Multidim::Array<T,3> subtracted(image.shape(), image.strides());

    for (int i = 0; i < image.shape()[0]; i++) {
        for (int j = 0; j < image.shape()[1]; j++) {
            for (int k = 0; k < image.shape()[2]; k++) {

                std::array<int,2> dFIndex;

                if (axis == 0) {
                    dFIndex = {j,k};
                }

                if (axis == 1) {
                    dFIndex = {i,k};
                }

                if (axis == 2) {
                    dFIndex = {i,j};
                }

                subtracted.atUnchecked(i,j,k) = image.valueUnchecked(i,j,k) - std::min(darkFrame.valueUnchecked(dFIndex), image.valueUnchecked(i,j,k));

            }
        }
    }

    return subtracted;

}

} // namespace PikaLTools

#endif // PIKALTOOLS_DARKFRAMESTOOLS_H
