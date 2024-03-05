#include "georasterreader.h"

namespace PikaLTools {

template <typename T, int nDim>
std::optional<StereoVisionApp::Geo::GeoRasterData<T, nDim>> readGeoRasterData(std::string const& filename, bool strictType) {

    static_assert (nDim == 2 or nDim == 3, "a georaster can only have 2 or three dimensions");

    using IdxArray = std::array<int, nDim>;

    constexpr GDALAccess access = GA_ReadOnly;

    GDALDatasetUniquePtr dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen( filename.c_str(), access )));

    if (!dataset) {
        return std::nullopt;
    }

    StereoVisionApp::Geo::GeoRasterData<T,nDim> ret;

    ret.crsInfos = dataset->GetProjectionRef();
    OGRSpatialReference ogrSpatialRef(ret.crsInfos.c_str());

    bool invertXY = ogrSpatialRef.EPSGTreatsAsLatLong();

    if (ret.crsInfos.empty()) {
        return std::nullopt;
    }

    double geoTransform[6];
    CPLErr err = dataset->GetGeoTransform( geoTransform );

    if (err != CE_None) {
        return std::nullopt;
    }

    int sX = dataset->GetRasterXSize();
    int sY = dataset->GetRasterYSize();
    int sC = dataset->GetRasterCount();

    if (nDim == 2 and sC != 1) {
        return std::nullopt;
    }

    IdxArray shape;
    IdxArray strides;

    shape[0] = sY;
    shape[1] = sX;

    if (nDim == 3) {
        shape[2] = sC;
    }

    if (nDim == 2) {
        strides[0] = sX;
        strides[1] = 1;
    }

    if (nDim == 3) {
        strides[0] = sX*sC;
        strides[1] = sC;
        strides[2] = 1;
    }

    auto bands = dataset->GetBands();

    if (bands.size() == 0) {
        return std::nullopt;
    }

    if (strictType) {
        if (bands[0]->GetRasterDataType() != gdalRasterTypeCode<T>()) {
            return std::nullopt;
        }
    }

    ret.geoTransform << geoTransform[1], geoTransform[2], geoTransform[0],
            geoTransform[4], geoTransform[5], geoTransform[3];

    if (invertXY) {
        Eigen::Matrix<double, 1,3> tmp = ret.geoTransform.template block<1,3>(0,0);
        ret.geoTransform.template block<1,3>(0,0) = ret.geoTransform.template block<1,3>(1,0);
        ret.geoTransform.template block<1,3>(1,0) = tmp;
    }

    ret.raster = Multidim::Array<T, nDim>(shape, strides);

    for (int r = 0; r < sY; r++) {
        for (int c = 0; c < sC; c++) {

            IdxArray idx;
            idx[0] = r;
            idx[1] = 0;

            if (nDim == 3) {
                idx[2] = c;
            }

            CPLErr e = bands[c]->RasterIO(GF_Read,0,r,sX,1,&ret.raster.atUnchecked(idx),sX,1,gdalRasterTypeCode<T>(),0,0);

            if(!(e == 0)) {
                return std::nullopt;
            }
        }
    }

    return ret;
}

template std::optional<StereoVisionApp::Geo::GeoRasterData<uint8_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<uint8_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);

template std::optional<StereoVisionApp::Geo::GeoRasterData<int8_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<int8_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);


template std::optional<StereoVisionApp::Geo::GeoRasterData<uint16_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<uint16_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);

template std::optional<StereoVisionApp::Geo::GeoRasterData<int16_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<int16_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);


template std::optional<StereoVisionApp::Geo::GeoRasterData<uint32_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<uint32_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);

template std::optional<StereoVisionApp::Geo::GeoRasterData<int32_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<int32_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);


template std::optional<StereoVisionApp::Geo::GeoRasterData<uint64_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<uint64_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);

template std::optional<StereoVisionApp::Geo::GeoRasterData<int64_t, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<int64_t, 3>> readGeoRasterData(std::string const& filename, bool strictType);


template std::optional<StereoVisionApp::Geo::GeoRasterData<float, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<double, 3>> readGeoRasterData(std::string const& filename, bool strictType);

} // namespace PikaLTools

