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

constexpr int LineAxis = 0;
constexpr int SamplesAxis = 1;
constexpr int BandsAxis = 2;

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

template<typename T>
Multidim::Array<T, 3> read_bil_sequence(std::vector<std::string> const& filenames) {

    for (std::string const& filename : filenames) {
        if (!envi_bil_img_match_type<T>(filename)) {
            return Multidim::Array<T, 3>();
        }
    }

    int lines = 0;
    int samples = -1;
    int bands = -1;


    std::vector<std::array<int,3>> imgs_shapes;
    std::vector<std::array<int,3>> imgs_strides;

    imgs_shapes.reserve(filenames.size());
    imgs_strides.reserve(filenames.size());

    for (std::string const& filename : filenames) {

        auto header = readHeaderData(filename);

        if (!header.has_value()) {
            return Multidim::Array<T, 3>();
        }

        std::map<std::string, std::string> headerData = header.value();

        if (headerData.count("lines") <= 0) {
            return Multidim::Array<T, 3>();
        }

        if (headerData.count("samples") <= 0) {
            return Multidim::Array<T, 3>();
        }

        if (headerData.count("bands") <= 0) {
            return Multidim::Array<T, 3>();
        }

        int c_lines;
        int c_samples;
        int c_bands;

        try {
            c_lines = std::stoi(headerData["lines"]);
            c_samples = std::stoi(headerData["samples"]);
            c_bands = std::stoi(headerData["bands"]);
        }
        catch(std::invalid_argument const& e) {
            return Multidim::Array<T, 3>();
        }

        imgs_shapes.push_back({c_lines, c_samples, c_bands});


        if (headerData.count("interleave") <= 0) {
            return Multidim::Array<T, 3>();
        }

        std::string interleave = headerData["interleave"];

        if (interleave == "bil") {
            imgs_strides.push_back({c_samples*c_bands,1,c_samples});
        } else if (interleave == "bip") {
            imgs_strides.push_back({c_bands,c_bands*c_lines,1});
        } else if (interleave == "bsq") {
            imgs_strides.push_back({c_samples*c_bands,1,c_samples});
        } else {
            return Multidim::Array<T, 3>();
        }


        lines += c_lines;

        if (samples > 0) {
            if (c_samples != samples) {
                return Multidim::Array<T, 3>();
            }
        } else {
            samples = c_samples;
        }

        if (bands > 0) {
            if (c_bands != bands) {
                return Multidim::Array<T, 3>();
            }
        } else {
            bands = c_bands;
        }

    }

    if (bands <= 0 or samples <= 0 or lines <= 0) {
        return Multidim::Array<T, 3>();
    }

    std::array<int, 3> shape = {lines, samples, bands};
    std::array<int, 3> strides = {samples*bands,1,samples};

    Multidim::Array<T, 3> ret(shape, strides);

    int nLinesCopied = 0;

    for (int i = 0; i < filenames.size(); i++) {

        std::ifstream image(filenames[i]);

        if (image.fail()) {
            return Multidim::Array<T, 3>();
        }

        image.seekg(0, std::ios::end);
        size_t n_bytes = image.tellg();

        if (n_bytes != lines*samples*bands*sizeof (T)) {
            //invalid file size
            return Multidim::Array<T, 3>();
        }

        T* buffer = new T[n_bytes/sizeof (T)];

        image.seekg(0);
        image.read(reinterpret_cast<char*>(buffer), n_bytes);

        Multidim::Array<T, 3> sub(buffer, imgs_shapes[i], imgs_strides[i], true);

        if (imgs_strides[i][1] == strides[1] and imgs_strides[i][2] == strides[2]) {
            int n = imgs_strides[i][0]*imgs_strides[i][1]*imgs_strides[i][2];

            T* cell0 = &ret.atUnchecked(nLinesCopied, 0, 0);
            T* dataStart = &sub.atUnchecked(0,0,0);

            std::memcpy(cell0, dataStart, n*sizeof(T));
        } else {

            for (int l = 0; l < imgs_shapes[i][0]; l++) {
                for (int s = 0; s < imgs_shapes[i][1]; s++) {
                    for (int b = 0; b < imgs_shapes[i][2]; b++) {
                        ret.atUnchecked(nLinesCopied+l, s, b) = sub.atUnchecked(l,s,b);
                    }
                }
            }

        }

    }

    return ret;

}

Multidim::Array<float, 3> read_envi_bil_to_float(std::string const& filename);
Multidim::Array<float, 3> read_bil_sequence_to_float(std::vector<std::string> const& filenames);

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
