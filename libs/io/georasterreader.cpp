#include "georasterreader.h"

#include <geotiff/geotiff.h>
#include <geotiff/geo_normalize.h>
#include <geotiff/geotiffio.h>
#include <geotiff/xtiffio.h>

#include <StereoVision/io/image_io.h>

namespace PikaLTools {

/*!
 * \brief The GeoTiff class is a small encapsulator for libgeotiff data structs to leverage RAII
 */
class GeoTiff {
public:
    GeoTiff(std::string const& filename):
        tiffFile(nullptr),
        geoInfos(nullptr)
    {

        tiffFile = XTIFFOpen(filename.c_str(),"r");

        if (tiffFile == nullptr) {
            return;
        }

        geoInfos = GTIFNew(tiffFile);

    }

    ~GeoTiff() {
        if (tiffFile != nullptr) {
            TIFFClose(tiffFile);
            GTIFFree(geoInfos);
        }
    }

    bool isValid() const {
        return tiffFile != nullptr and geoInfos != nullptr;
    }

    TIFF* tiffFile;
    GTIF* geoInfos;
};

template <typename T, int nDim>
std::optional<StereoVisionApp::Geo::GeoRasterData<T, nDim>> readGeoRasterData(std::string const& filename, bool strictType) {

    static_assert (nDim == 2 or nDim == 3, "a georaster can only have 2 or three dimensions");

    using IdxArray = std::array<int, nDim>;

    GeoTiff geoTiffHandles(filename);

    if (!geoTiffHandles.isValid()) {
        return std::nullopt;
    }

    GTIFDefn def;

    int code = GTIFGetDefn(geoTiffHandles.geoInfos, &def);

    if (!code) {
        return std::nullopt;
    }

    int epsg = def.GCS;
    std::stringstream sstream;
    sstream << "EPSG:" << epsg;
    std::string epsgCode = sstream.str();

    //Proj respect the CRS axis ordering, but geotiff will always set X axis a longitude.
    bool invertXY = (def.Model == ModelTypeGeographic); //this detect if the CRS use latlon, thus the coordinates needs to be inverted.

    constexpr double coordScale = 100;

    double x0 = 0;
    double y0 = 0;

    double x1 = coordScale;
    double y1 = coordScale;

    code = GTIFImageToPCS(geoTiffHandles.geoInfos, &x0, &y0);

    if (!code) {
        return std::nullopt;
    }

    code = GTIFImageToPCS(geoTiffHandles.geoInfos, &x1, &y1);

    if (!code) {
        return std::nullopt;
    }

    double sX = (x1 - x0)/coordScale;
    double sY = (y1 - y0)/coordScale;

    double oX = x0;
    double oY = y0;

    Eigen::Matrix<double, 2,3> geoTransform = Eigen::Matrix<double, 2,3>::Zero();

    if (invertXY) {

        geoTransform(0,1) = sY;
        geoTransform(1,0) = sX;

        geoTransform(0,2) = oY;
        geoTransform(1,2) = oX;

    } else {

        geoTransform(0,0) = sX;
        geoTransform(1,1) = sY;

        geoTransform(0,2) = oX;
        geoTransform(1,2) = oY;
    }

    Multidim::Array<T, 3> imgData = StereoVision::IO::readImage<T>(filename);

    if (imgData.empty()) {
        return std::nullopt;
    }

    if (nDim == 2 and imgData.shape()[2] != 1) {
        return std::nullopt;
    }

    StereoVisionApp::Geo::GeoRasterData<T,nDim> ret;

    std::array<int, nDim> shape;
    std::array<int, nDim> strides;

    for (int i = 0; i < nDim; i++) {
        shape[i] = imgData.shape()[i];
        strides[i] = imgData.strides()[i];
    }

    bool managePtr = true;

    ret.raster = Multidim::Array<T, nDim>(imgData.takePointer(), shape, strides, managePtr); //move the data to an image of the correct dimension

    ret.crsInfos = epsgCode;

    if (ret.crsInfos.empty()) {
        return std::nullopt;
    }

    ret.geoTransform = geoTransform;

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
template std::optional<StereoVisionApp::Geo::GeoRasterData<float, 3>> readGeoRasterData(std::string const& filename, bool strictType);

template std::optional<StereoVisionApp::Geo::GeoRasterData<double, 2>> readGeoRasterData(std::string const& filename, bool strictType);
template std::optional<StereoVisionApp::Geo::GeoRasterData<double, 3>> readGeoRasterData(std::string const& filename, bool strictType);

} // namespace PikaLTools

