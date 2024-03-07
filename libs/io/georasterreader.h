#ifndef PIKALTOOLS_GEORASTERREADER_H
#define PIKALTOOLS_GEORASTERREADER_H

#include <optional>

#include <MultidimArrays/MultidimArrays.h>
#include <Eigen/Core>

#include <steviapp/geo/geoRaster.h>

namespace PikaLTools {

enum class DataTypeCode {
    Unknown = 0,
    UInt8 = 1,
    Int8 = 2,
    UInt16 = 3,
    Int16 = 4,
    UInt32 = 5,
    Int32 = 6,
    UInt64 = 7,
    Int64 = 8,
    Double = 9,
    Float = 10
};

template <typename T>
constexpr DataTypeCode getDataTypeCode() {

    if (std::is_same_v<T, uint8_t>) {
        return DataTypeCode::UInt8;
    }
    if (std::is_same_v<T, int8_t>) {
        return DataTypeCode::Int8;
    }

    if (std::is_same_v<T, uint16_t>) {
        return DataTypeCode::UInt16;
    }
    if (std::is_same_v<T, int16_t>) {
        return DataTypeCode::Int16;
    }

    if (std::is_same_v<T, uint32_t>) {
        return DataTypeCode::UInt32;
    }
    if (std::is_same_v<T, int32_t>) {
        return DataTypeCode::Int32;
    }

    if (std::is_same_v<T, uint64_t>) {
        return DataTypeCode::UInt64;
    }
    if (std::is_same_v<T, int64_t>) {
        return DataTypeCode::Int64;
    }

    if (std::is_same_v<T, float>) {
        return DataTypeCode::Float;
    }
    if (std::is_same_v<T, double>) {
        return DataTypeCode::Double;
    }

    return DataTypeCode::Unknown;
}

template <typename T, int nDim>
std::optional<StereoVisionApp::Geo::GeoRasterData<T, nDim>> readGeoRasterData(std::string const& filename, bool strictType = false);

} // namespace PikaLTools

#endif // PIKALTOOLS_GEORASTERREADER_H
