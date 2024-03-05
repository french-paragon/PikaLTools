#ifndef GEORASTERREADER_H
#define GEORASTERREADER_H


#ifndef PIKALTOOLS_GEORASTERREADER_H
#define PIKALTOOLS_GEORASTERREADER_H

#include <optional>

#include <MultidimArrays/MultidimArrays.h>
#include <Eigen/Core>

#include <gdal/gdal_priv.h>

#include <steviapp/geo/geoRaster.h>

namespace PikaLTools {

template <typename T>
constexpr GDALDataType gdalRasterTypeCode() {
    if (std::is_same_v<T, uint8_t>) {
        return GDT_Byte;
    }

    if (std::is_same_v<T, uint16_t>) {
        return GDT_UInt16;
    }

    if (std::is_same_v<T, uint32_t>) {
        return GDT_UInt32;
    }

    if (std::is_same_v<T, int16_t>) {
        return GDT_Int16;
    }

    if (std::is_same_v<T, int32_t>) {
        return GDT_Int32;
    }

    if (std::is_same_v<T, float>) {
        return GDT_Float32;
    }

    if (std::is_same_v<T, double>) {
        return GDT_Float64;
    }

    if (std::is_same_v<T, std::complex<int16_t>>) {
        return GDT_CInt16;
    }

    if (std::is_same_v<T, std::complex<int32_t>>) {
        return GDT_CInt32;
    }

    if (std::is_same_v<T, std::complex<float>>) {
        return GDT_CFloat32;
    }

    if (std::is_same_v<T, std::complex<double>>) {
        return GDT_CFloat64;
    }

    return GDT_Unknown;
}

template <typename T, int nDim>
std::optional<StereoVisionApp::Geo::GeoRasterData<T, nDim>> readGeoRasterData(std::string const& filename, bool strictType = false);

} // namespace PikaLTools

#endif // PIKALTOOLS_GEORASTERREADER_H


#endif // GEORASTERREADER_H
