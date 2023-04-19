#ifndef PIKALTOOLS_READ_ENVI_BIL_H
#define PIKALTOOLS_READ_ENVI_BIL_H

#include "io_globals.h"

#include <MultidimArrays/MultidimArrays.h>

#include <map>
#include <string>
#include <optional>
#include <type_traits>
#include <complex>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>

std::optional<std::map<std::string, std::string>> readHeaderData(std::string const& filename);

template<typename T>
bool envi_bil_img_match_type(std::string const& filename) {

    auto header = readHeaderData(filename);

    if (!header.has_value()) {
        return false;
    }

    std::map<std::string, std::string> headerData = header.value();

    if (headerData.count("data type") <= 0) {
        return false;
    }

    int datatype;

    try {
        datatype = std::stoi(headerData["data type"]);
    }
    catch(std::invalid_argument const& e) {
        return false;
    }

    // source = https://www.l3harrisgeospatial.com/docs/enviheaderfiles.html
    switch (datatype) {
    case 1:
        return std::is_same_v<T, uint8_t>;
    case 2:
        return std::is_same_v<T, int16_t>;
    case 3:
        return std::is_same_v<T, int32_t>;
    case 4:
        return std::is_same_v<T, float>;
    case 5:
        return std::is_same_v<T, double>;
    case 6:
        return std::is_same_v<T, std::complex<float>>;
    case 9:
        return std::is_same_v<T, std::complex<double>>;
    case 12:
        return std::is_same_v<T, uint16_t>;
    case 13:
        return std::is_same_v<T, uint32_t>;
    case 14:
        return std::is_same_v<T, int64_t>;
    case 15:
        return std::is_same_v<T, uint64_t>;
    default:
        break;
    }

    return false;
}

template<typename T>
Multidim::Array<T, 3> read_envi_bil(std::string const& filename) {

    if (!envi_bil_img_match_type<T>(filename)) {
        return Multidim::Array<T, 3>();
    }

    auto header = readHeaderData(filename);

    if (!header.has_value()) {
        return Multidim::Array<T, 3>();
    }

    std::map<std::string, std::string> headerData = header.value();

    int lines;
    int samples;
    int bands;

    if (headerData.count("lines") <= 0) {
        return Multidim::Array<T, 3>();
    }

    if (headerData.count("samples") <= 0) {
        return Multidim::Array<T, 3>();
    }

    if (headerData.count("bands") <= 0) {
        return Multidim::Array<T, 3>();
    }

    try {
        lines = std::stoi(headerData["lines"]);
        samples = std::stoi(headerData["samples"]);
        bands = std::stoi(headerData["bands"]);
    }
    catch(std::invalid_argument const& e) {
        return Multidim::Array<T, 3>();
    }

    std::array<int,3> shape = {lines, samples, bands};


    if (headerData.count("interleave") <= 0) {
        return Multidim::Array<T, 3>();
    }

    std::string interleave = headerData["interleave"];

    std::array<int,3> strides;

    if (interleave == "bil") {
        strides = {samples*bands,1,samples};
    } else if (interleave == "bip") {
        strides = {bands,bands*lines,1};
    } else if (interleave == "bsq") {
        strides = {samples*bands,1,samples};
    } else {
        return Multidim::Array<T, 3>();
    }

    std::ifstream image(filename);

    if (image.fail()) {
        return Multidim::Array<T, 3>();
    }

    image.seekg(0, std::ios::end);
    size_t n_bytes = image.tellg();

    if (n_bytes % sizeof (T) != 0) {
        //invalid file size
        return Multidim::Array<T, 3>();
    }

    if (n_bytes/sizeof (T) != lines*samples*bands) {
        //invalid file size
        return Multidim::Array<T, 3>();
    }

    T* buffer = new T[n_bytes/sizeof (T)];

    image.seekg(0);
    image.read(reinterpret_cast<char*>(buffer), n_bytes);

    Multidim::Array<T, 3> ret(buffer, shape, strides, true);
    return ret;

}

Multidim::Array<float, 3> read_envi_bil_to_float(std::string const& filename);

std::vector<double> read_envi_bil_times(std::string const& filename);

struct EnviBilLcfLine {
    int nLine;
    double timeStamp;
    double roll;
    double pitch;
    double yaw;
    double lat;
    double lon;
    double height;
};

std::vector<EnviBilLcfLine> read_envi_bil_lcf_data(std::string const& filename);

#endif // PIKALTOOLS_READ_ENVI_BIL_H
