#ifndef WRITE_ENVI_BIL_H
#define WRITE_ENVI_BIL_H

#include <cstdint>
#include <complex>
#include <iostream>
#include <fstream>

#include "./read_envi_bil.h"

/*
 * enum BilTypes {
    BilUint8T = 1,
    BilInt16T = 2,
    BilInt32T = 3,
    BilFloatT = 4,
    BilDoubleT = 5,
    BilComplexFloatT = 6,
    BilComplexDoubleT = 9,
    BilUint16T = 12,
    BilUint32T = 13,
    BilInt64T = 14,
    BilUint64T = 15
};
 */

template<typename T>
constexpr int typeEnvyBilCode() {
    if constexpr (std::is_same_v<T, uint8_t>) {
        return BilUint8T;
    }
    if constexpr (std::is_same_v<T, int16_t>) {
        return BilInt16T;
    }
    if constexpr (std::is_same_v<T, uint16_t>) {
        return BilUint16T;
    }
    if constexpr (std::is_same_v<T, int32_t>) {
        return BilInt32T;
    }
    if constexpr (std::is_same_v<T, uint32_t>) {
        return BilUint32T;
    }
    if constexpr (std::is_same_v<T, int64_t>) {
        return BilInt64T;
    }
    if constexpr (std::is_same_v<T, uint64_t>) {
        return BilUint64T;
    }
    if constexpr (std::is_same_v<T, float>) {
        return BilFloatT;
    }
    if constexpr (std::is_same_v<T, double>) {
        return BilDoubleT;
    }
    if constexpr (std::is_same_v<T, std::complex<float>>) {
        return BilComplexFloatT;
    }
    if constexpr (std::is_same_v<T, std::complex<double>>) {
        return BilComplexDoubleT;
    }
    return InvalidBilT;
}

struct enviBilHDRExtraField {
    std::string name;
    std::string value;
};

template <typename T>
bool write_envi_bil(Multidim::Array<T, 3> const& data,
                    std::string const& filename,
                    std::vector<enviBilHDRExtraField> const& extraFields = {},
                    int data_lineDim = LineAxis,
                    int data_sampleDim = SamplesAxis,
                    int data_bandsDim = BandsAxis) {

    constexpr int billTypeFlag = typeEnvyBilCode<T>();

    static_assert(billTypeFlag != InvalidBilT);

    if (data.empty()) {
        std::cerr << "Empty input image, nothing to write to bil" << std::endl;
        return false;
    }

    std::string hdrFileName = filename + ".hdr";

    std::fstream bilFile(filename, std::fstream::out);

    if (!bilFile.is_open()) {
        std::cerr << "Impossible to open bil file for bil " << filename << std::endl;
        return false;
    }

    std::fstream hdrFile(hdrFileName, std::fstream::out);

    if (!hdrFile.is_open()) {
        std::cerr << "Impossible to open hdr file for bil " << filename << std::endl;
        return false;
    }

    int channels = data.shape()[data_bandsDim];
    int samples = data.shape()[data_sampleDim];
    int lines = data.shape()[data_lineDim];;

    //write the file in bil order
    //store the first line of each band in order, then second line of each band and so on and so forth
    T maxPix = 0;

    std::array<int,3> idx;
    for (int l = 0; l < lines; l++) {
        idx[data_lineDim] = l;

        for (int c = 0; c < channels; c++) {
            idx[data_bandsDim] = c;

            for (int s = 0; s < samples; s++) {
                idx[data_sampleDim] = s;

                T pix = data.valueUnchecked(idx);

                maxPix = std::max(pix, maxPix);

                bilFile.write(reinterpret_cast<char*>(&pix), sizeof(T));
            }
        }
    }

    bilFile.close();

    hdrFile << "ENVI" << "\n";
    hdrFile << "interleave = bil" << "\n";
    hdrFile << "data type = " << billTypeFlag << "\n";
    hdrFile << "lines = " << lines << "\n";
    hdrFile << "samples = " << samples << "\n";
    hdrFile << "bands = " << channels << "\n";
    hdrFile << "ceiling = " << maxPix << "\n";

    for (enviBilHDRExtraField const& field : extraFields) {
        hdrFile << field.name << " = " << field.value << "\n";
    }

    hdrFile.close();

    return true;
}

#endif // WRITE_ENVI_BIL_H
