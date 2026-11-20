#include "relativeoffsetsestimator.h"

#include <fstream>

namespace PikaLTools {

namespace PushBroomRelativeOffsets {

bool RelativeShiftConverter::cacheRectificationInfos(std::string const& filePath)  {

    std::ofstream outFile(filePath, std::ofstream::out);

    if (!outFile.is_open() or !outFile.good()) {
        return false;
    }

    outFile << "#line_id horizontal_shift";

    int nLines = std::min(_verticalLinesMatch.size(), _horizontalLinesOffsets.size());

    for (int i = 0; i < nLines; i++) {
        outFile << "\n" << _verticalLinesMatch[i] << " " << _horizontalLinesOffsets[i];
        if (!outFile.good()) {
            return false;
        }
    }

    return true;

}

std::optional<RelativeShiftConverter::ShiftInfos> RelativeShiftConverter::loadCachedRectificationInfos(std::string const& filePath) {


    std::ifstream inFile(filePath, std::ofstream::in);

    if (!inFile.is_open() or !inFile.good()) {
        return std::nullopt;
    }

    RelativeShiftConverter::ShiftInfos ret;

    std::string line;

    while (std::getline(inFile, line)) {

        if (line[0] == '#') {
            continue;
        }

        std::stringstream lineSplit(line);
        int lineId;
        float lineShift;

        lineSplit >> lineId;
        lineSplit >> lineShift;

        if (lineSplit.fail()) {
            continue;
        }

        ret.verticalLinesMatch.push_back(lineId);
        ret.horizontalLinesOffsets.push_back(lineShift);
    }

    if (ret.verticalLinesMatch.empty()) {
        return std::nullopt;
    }

    return ret;
}

} // namespace PushBroomRelativeOffsets

} // namespace PikaLTools
