#ifndef READ_AVIRIS4_BIN_H
#define READ_AVIRIS4_BIN_H

#include <optional>
#include <type_traits>
#include "./io_globals.h"

#include <MultidimArrays/MultidimArrays.h>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdint>

namespace aviris4io {

using data_t = uint16_t;

struct Aviris4FrameData {

    std::vector<double> gpsTime;
    Multidim::Array<data_t, 3> frame;

};

/*!
 * \brief getAviris4FramesNLines get the number of lines in an aviris4 frame
 * \param frameFilePath the file path to the frame file.
 * \return the number of lines, or -1 in case of error
 */
int getAviris4FramesNLines(std::string const& frameFilePath);

/*!
 * \brief loadFrame load a single aviris4 frame
 * \param frameFilePath the path to the frame file.
 * \return the frame data with gps time of the individual lines
 */
Multidim::Array<data_t, 3> loadFrame(std::string const& frameFilePath);

/*!
 * \brief loadFrameSlice load a certain number of lines from an aviris4 frame
 * \param frameFilePath the path to the frame file
 * \param startLine the starting line to load from
 * \param nLines the number of lines to load
 * \return the frame data with gps time of the individual lines still included
 */
Multidim::Array<data_t, 3> loadFrameSlice(std::string const& frameFilePath, int startLine, int nLines);

/*!
 * \brief loadFrameTime load the gps time from a single aviris4 frame
 * \param frameFilePath the path to the frame file.
 * \return the times
 *
 * This function load only the times from the frame, not any other data.
 */
std::vector<int64_t> loadFrameTimes(std::string const& frameFilePath);

/*!
 * \brief loadSequenceTimes load all times from a sequence of aviris4 frames
 * \param sequenceFolderPath the folder where all frames are stored
 * \return the times
 *
 * This function load only the times from the frame, not any other data.
 */
std::vector<int64_t> loadSequenceTimes(std::string const& sequenceFolderPath, const std::string &filter);

/*!
 * \brief getFilesInSequence list the files in a sequence
 * \param sequenceFolderPath the folder containing the sequence
 * \param filter the filter (unix wildcard) for selecting the files in the sequence
 * \return the list of files, in chronological order based on the time in the files.
 */
std::vector<std::string> getFilesInSequence(std::string const& sequenceFolderPath, const std::string &filter);


} // namespace aviris4io

#endif // READ_AVIRIS4_BIN_H
