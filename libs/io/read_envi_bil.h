#ifndef PIKALTOOLS_READ_ENVI_BIL_H
#define PIKALTOOLS_READ_ENVI_BIL_H

#include "./io_globals.h"
#include "./read_trajectory_data.h"

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

// source = https://www.l3harrisgeospatial.com/docs/enviheaderfiles.html
enum BilTypes {
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
    case BilUint8T:
        return std::is_same_v<T, uint8_t>;
    case BilInt16T:
        return std::is_same_v<T, int16_t>;
    case BilInt32T:
        return std::is_same_v<T, int32_t>;
    case BilFloatT:
        return std::is_same_v<T, float>;
    case BilDoubleT:
        return std::is_same_v<T, double>;
    case BilComplexFloatT:
        return std::is_same_v<T, std::complex<float>>;
    case BilComplexDoubleT:
        return std::is_same_v<T, std::complex<double>>;
    case BilUint16T:
        return std::is_same_v<T, uint16_t>;
    case BilUint32T:
        return std::is_same_v<T, uint32_t>;
    case BilInt64T:
        return std::is_same_v<T, int64_t>;
    case BilUint64T:
        return std::is_same_v<T, uint64_t>;
    default:
        break;
    }

    return false;
}

/*!
 * \brief get_bil_sequence_files gives the list of bil files in a folder containing a sequence
 * \param main_folder the folder containing the sequence (each file along with metadata in its own folder)
 * \return the list of files
 */
std::vector<std::string> get_bil_sequence_files(std::string const& main_folder);

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

    if (n_bytes/sizeof (T) != static_cast<size_t>(lines)*samples*bands) {
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
Multidim::Array<T, 3> read_bil_sequence(std::vector<std::string> const& filenames,
                                        std::optional<int> firstLine = std::nullopt,
                                        std::optional<int> lastLine = std::nullopt) {

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

    int s_Line = 0;
    int l_Line = lines;

    if (firstLine.has_value()) {
        int firstLineVal = firstLine.value();

        if (firstLineVal >= 0 and firstLineVal < lines) {
            s_Line = firstLineVal;
        }
    }

    if (lastLine.has_value()) {
        int lastLineVal = lastLine.value();

        if (lastLineVal > s_Line and lastLineVal <= lines) {
            l_Line = lastLineVal;
        }
    }

    int n_loaded_lines = l_Line - s_Line;

    std::array<int, 3> shape = {n_loaded_lines, samples, bands};
    std::array<int, 3> strides = {samples*bands,1,samples};

    Multidim::Array<T, 3> ret(shape, strides);

    int nLinesCopied = 0;
    int nLinesSeen = 0;

    for (int i = 0; i < filenames.size(); i++) {

        if (nLinesSeen > s_Line) {
            //all required data copied
            break;
        }

        int imgNLines = imgs_shapes[i][LineAxis];

        if (nLinesSeen + imgNLines <= s_Line) {
            //not in the region of interest yet
            nLinesSeen += imgNLines;
            continue;
        }

        std::ifstream image(filenames[i]);

        if (image.fail()) {
            return Multidim::Array<T, 3>();
        }

        image.seekg(0, std::ios::end);
        size_t n_bytes = image.tellg();

        if (n_bytes != imgNLines*samples*bands*sizeof (T)) {
            //invalid file size
            return Multidim::Array<T, 3>();
        }

        int sub_sLine = 0;
        int sub_lLine = imgNLines;

        if (nLinesSeen < s_Line and nLinesSeen + imgNLines >= s_Line) {
            sub_sLine = s_Line - nLinesSeen;
        }

        if (nLinesSeen < l_Line and nLinesSeen + imgNLines >= l_Line) {
            sub_lLine = l_Line - nLinesSeen;
        }

        T* buffer = new T[n_bytes/sizeof (T)];

        image.seekg(0);
        image.read(reinterpret_cast<char*>(buffer), n_bytes);

        Multidim::Array<T, 3> sub(buffer, imgs_shapes[i], imgs_strides[i], true);

        if (imgs_strides[i][0] == strides[0] and
                imgs_strides[i][1] == strides[1] and
                imgs_strides[i][2] == strides[2]) {

            int nLines = sub_lLine - sub_sLine;
            int n = nLines*imgs_strides[i][LineAxis]; //possible only if the strides are dense

            T* cell0 = &ret.atUnchecked(nLinesCopied, 0, 0);
            T* dataStart = &sub.atUnchecked(sub_sLine,0,0);

            std::memcpy(cell0, dataStart, n*sizeof(T));

            nLinesCopied += nLines;

        } else {

            for (int l = sub_sLine; l < sub_lLine; l++) {

                for (int s = 0; s < imgs_shapes[i][1]; s++) {
                    for (int b = 0; b < imgs_shapes[i][2]; b++) {
                        ret.atUnchecked(nLinesCopied, s, b) = sub.atUnchecked(l,s,b);
                    }
                }

                nLinesCopied++;
            }

        }

        nLinesSeen += imgs_shapes[i][LineAxis];

    }

    return ret;

}

Multidim::Array<float, 3> read_envi_bil_to_float(std::string const& filename);
Multidim::Array<float, 3> read_bil_sequence_to_float(std::vector<std::string> const& filenames);

/*!
 * \brief read_envi_bil_times read the times infos
 * \param filename the bil filename
 * \return the list of times
 *
 * This function load the times only if a time file exist. It tries to load in priority the .timings file, if not present loads the .times file.
 */
std::vector<double> read_envi_bil_times(std::string const& filename);

/*!
 * \brief get_envi_bil_lines_times get the times for the lines
 * \param filename the bil filename
 * \return the list of times
 *
 * this function tries to load the times from the .timings file, if it exists. if not, it compute the times from the headers and lcf files.
 */
std::vector<double> get_envi_bil_lines_times(std::string const& filename);

using EnviBilLcfLine = RawTrajectoryLine;

/*!
 * \brief read_envi_bil_lcf_data read the trajectory data from the lcf associated with a bil file
 * \param filename the path to the bil file
 * \return the trajectory
 */
std::vector<EnviBilLcfLine> read_envi_bil_lcf_data(std::string const& filename);

#endif // PIKALTOOLS_READ_ENVI_BIL_H
