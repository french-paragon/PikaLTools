#include "read_envi_bil.h"

#include <algorithm>

#include <QString>
#include <QDir>
#include <QFile>

const std::string WHITESPACE = " \n\r\t\f\v";

std::string ltrim(const std::string &s)
{
    size_t start = s.find_first_not_of(WHITESPACE);
    return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string &s)
{
    size_t end = s.find_last_not_of(WHITESPACE);
    return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string &s) {
    return rtrim(ltrim(s));
}

std::vector<std::string> get_bil_sequence_files(std::string const& main_folder) {

    QString folder = QString::fromStdString(main_folder);
    QList<QString> bilFiles;

    QDir mainFolder(folder);

    //List the folders within the main folder
    QStringList subFolders = mainFolder.entryList();
    std::sort(subFolders.begin(), subFolders.end(), [] (QString str1, QString str2) {
        QString n1 = str1.split("-").last();
        QString n2 = str2.split("-").last();

        bool ok;
        int i1 = n1.toInt(&ok);

        if (!ok) {
            return false;
        }

        int i2 = n2.toInt(&ok);

        if (!ok) {
            return true;
        }

        return i1 < i2;
    });

    for (QString subFolder : subFolders) {

        if (subFolder == "." or subFolder == "..") {
            continue;
        }

        QString fullPath = mainFolder.absoluteFilePath(subFolder);

        QFileInfo dirInfos(fullPath);

        if (!dirInfos.isDir()) {
            continue;
        }

        QDir subDir(fullPath);

        QStringList entries = subDir.entryList({"*.bil"});

        if (entries.size() == 1) {
            bilFiles.push_back(subDir.absoluteFilePath(entries[0]));
        }
    }

    std::vector<std::string> ret(bilFiles.size());

    for (int i = 0; i < bilFiles.size(); i++) {
        ret[i] = bilFiles[i].toStdString();
    }

    return ret;
}

std::optional<std::map<std::string, std::string>> readBilHeaderData(std::string const& filename) {

    std::ifstream headerFile(filename + ".hdr");

    if (headerFile.fail()) {
        std::string replaced = filename;

        int s = filename.size();

        if (s < 3) {
            return std::nullopt;
        }
        replaced[s-3] = 'h';
        replaced[s-2] = 'd';
        replaced[s-1] = 'r';

        headerFile.open(replaced);

        if (headerFile.fail()) {
            return std::nullopt;
        }
    }

    std::string lineData;

    bool ok = std::getline(headerFile, lineData).good();

    if (!ok) {
        return std::nullopt;
    }

    std::string trimmed = trim(lineData);

    if (trimmed != "ENVI") {
        return std::nullopt;
    }

    std::map<std::string, std::string> ret;

    while (std::getline(headerFile, lineData)) {
        std::stringstream splitter(lineData);

        std::string var;
        std::getline(splitter, var, '=');

        std::string value;
        std::getline(splitter, value, '=');

        ret[trim(var)] = trim(value);
    }

    return ret;
}


Multidim::Array<float, 3> read_envi_bil_to_float(std::string const& filename) {

    if (envi_bil_img_match_type<uint8_t>(filename)) {
        return read_envi_bil<uint8_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<int8_t>(filename)) {
        return read_envi_bil<int8_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<uint16_t>(filename)) {
        return read_envi_bil<uint16_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<int16_t>(filename)) {
        return read_envi_bil<int16_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<uint32_t>(filename)) {
        return read_envi_bil<uint32_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<int32_t>(filename)) {
        return read_envi_bil<int32_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<uint64_t>(filename)) {
        return read_envi_bil<uint64_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<int64_t>(filename)) {
        return read_envi_bil<int64_t>(filename).cast<float>();
    }

    if (envi_bil_img_match_type<float>(filename)) {
        return read_envi_bil<float>(filename);
    }

    if (envi_bil_img_match_type<double>(filename)) {
        return read_envi_bil<double>(filename).cast<float>();
    }

    return Multidim::Array<float, 3>();
}


Multidim::Array<float, 3> read_bil_sequence_to_float(std::vector<std::string> const& filenames) {

    if (envi_bil_img_match_type<uint8_t>(filenames[0])) {
        return read_bil_sequence<uint8_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<int8_t>(filenames[0])) {
        return read_bil_sequence<int8_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<uint16_t>(filenames[0])) {
        return read_bil_sequence<uint16_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<int16_t>(filenames[0])) {
        return read_bil_sequence<int16_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<uint32_t>(filenames[0])) {
        return read_bil_sequence<uint32_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<int32_t>(filenames[0])) {
        return read_bil_sequence<int32_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<uint64_t>(filenames[0])) {
        return read_bil_sequence<uint64_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<int64_t>(filenames[0])) {
        return read_bil_sequence<int64_t>(filenames).cast<float>();
    }

    if (envi_bil_img_match_type<float>(filenames[0])) {
        return read_bil_sequence<float>(filenames);
    }

    if (envi_bil_img_match_type<double>(filenames[0])) {
        return read_bil_sequence<double>(filenames).cast<float>();
    }

    return Multidim::Array<float, 3>();

}

std::vector<double> read_envi_bil_times(std::string const& filename) {

    auto header = readBilHeaderData(filename);

    if (!header.has_value()) {
        return std::vector<double>();
    }

    std::map<std::string, std::string> headerData = header.value();

    int nLines = 0;

    if (headerData.count("lines") <= 0) {
        return std::vector<double>();
    }

    try {
        nLines = std::stoi(headerData["lines"]);
    } catch(std::invalid_argument const& e) {
        return std::vector<double>();
    }


    std::ifstream timesFile(filename + ".timing");

    if (timesFile.fail()) {

        timesFile.open((filename + ".times"));
        if (timesFile.fail()) {
            return std::vector<double>();
        }
    }

    std::string lineData;

    std::vector<double> ret;
    ret.reserve(nLines);

    while (std::getline(timesFile, lineData)) {

        float tVal;

        try {
            tVal = std::stod(lineData);
        } catch(std::invalid_argument const& e) {
            return std::vector<double>();
        }

        ret.push_back(tVal);
    }

    return ret;
}
std::vector<double> get_envi_bil_lines_times(std::string const& filename) {

    auto header = readBilHeaderData(filename);

    if (!header.has_value()) {
        return std::vector<double>();
    }

    std::map<std::string, std::string>& headerData = header.value();

    int nLines = 0;

    if (headerData.count("lines") <= 0) {
        return std::vector<double>();
    }

    try {
        nLines = std::stoi(headerData["lines"]);
    } catch(std::invalid_argument const& e) {
        return std::vector<double>();
    }


    std::ifstream timesFile(filename + ".timing");

    if (timesFile.fail()) {

        auto lcfData = read_envi_bil_lcf_data(filename);

        if (lcfData.empty()) {
            return std::vector<double>();
        }

        double t0 = lcfData[0].timeStamp;

        if (headerData.count("framerate") <= 0) {
            return std::vector<double>();
        }

        double frameRate;
        try {
         frameRate = stod(headerData["framerate"]);
        } catch(std::invalid_argument const& e) {
            return std::vector<double>();
        }
        double frameTime = 1/frameRate;

        std::vector<double> ret(nLines);

        for (int i = 0; i < nLines; i++) {
            ret[i] = t0 + i*frameTime;
        }

        return ret;
    }

    std::string lineData;

    std::vector<double> ret;
    ret.reserve(nLines);

    while (std::getline(timesFile, lineData)) {

        float tVal;

        try {
            tVal = std::stod(lineData);
        } catch(std::invalid_argument const& e) {
            return std::vector<double>();
        }

        ret.push_back(tVal);
    }

    return ret;
}


std::vector<EnviBilLcfLine> read_envi_bil_lcf_data(std::string const& filename) {

    std::string subname = filename.substr(0, filename.size()-4);

    std::ifstream lcfFile(subname + ".lcf");

    if (lcfFile.fail()) {
        return std::vector<EnviBilLcfLine>();
    }

    std::string lineData;

    std::vector<EnviBilLcfLine> ret;

    int lineCount = 0;

    while (std::getline(lcfFile, lineData)) {

        std::stringstream extractor(lineData);

        ret.emplace_back();

        ret.back().nLine = lineCount;
        lineCount++;

        extractor >> ret.back().timeStamp;
        extractor >> ret.back().roll;
        extractor >> ret.back().pitch;
        extractor >> ret.back().yaw;
        extractor >> ret.back().lon;
        extractor >> ret.back().lat;
        extractor >> ret.back().height;

    }

    return ret;
}
