#include "read_envi_bil.h"

#include <algorithm>

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

std::optional<std::map<std::string, std::string>> readHeaderData(std::string const& filename) {

    std::ifstream headerFile(filename + ".hdr");

    if (headerFile.fail()) {
        return std::nullopt;
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
