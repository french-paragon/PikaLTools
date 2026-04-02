#include "io/read_envi_bil.h"
#include "io/write_envi_bil.h"

#include "processing/relativeoffsetsestimator.h"

#include <StereoVision/io/image_io.h>
#include <StereoVision/imageProcessing/colorConversions.h>

#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <tclap/CmdLine.h>
#include <QApplication>
#include <QString>
#include <QFileInfo>

#include <iostream>


struct ShiftsInfos {
    std::vector<float> horizontal;
    std::vector<int> vertical;
};

struct RectifiedResult {
    Multidim::Array<float,3> rectified;
    ShiftsInfos infos;
};

template<bool verbose>
RectifiedResult horizontalOnly(Multidim::Array<float,3> const& image,
                                         int hradius,
                                         double dx_pos_lambda,
                                         int nIterations,
                                         double residual_treshold,
                                         int linesAxis,
                                         int samplesAxis,
                                         int bandsAxis) {

    constexpr bool smoothOut = true;

    auto rectificationDataInfos = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomHorizontalShiftBayesian<verbose>(
        image,
        hradius,
        dx_pos_lambda,
        smoothOut,
        nIterations,
        residual_treshold,
        linesAxis,
        samplesAxis,
        bandsAxis);

    auto& rectificationData = rectificationDataInfos.value();

    auto accumulated = PikaLTools::PushBroomRelativeOffsets::computeAccumulatedFromRelativeShifts(rectificationData);

    constexpr bool shiftsAccumulated = true;

    Multidim::Array<float,3> rectified = PikaLTools::PushBroomRelativeOffsets::computeHorizontallyRectifiedImage<shiftsAccumulated>(
        image,
        accumulated.accumulatedShifts,
        linesAxis,
        samplesAxis,
        bandsAxis);

    return {rectified, ShiftsInfos{.horizontal=accumulated.accumulatedShifts, .vertical={}}};

}

template<bool verbose>
RectifiedResult verticalOnly(Multidim::Array<float,3> const& image,
                                       int hwindow,
                                       int vradius,
                                       int linesAxis,
                                       int samplesAxis,
                                       int bandsAxis) {

    int nLines = image.shape()[linesAxis];

    std::vector<float> horizontalShifts(nLines);
    std::fill(horizontalShifts.begin(), horizontalShifts.end(), 0);

    std::vector<int> orderEstimated =
        PikaLTools::PushBroomRelativeOffsets::estimatePushBroomVerticalReorderBayesian<verbose>(
        image,
        horizontalShifts,
        hwindow,
        vradius,
        linesAxis,
        samplesAxis,
        bandsAxis);

    Multidim::Array<float,3> rectified = PikaLTools::PushBroomRelativeOffsets::computeVerticallyReorderedImage(image,
                                                                                 orderEstimated,
                                                                                 linesAxis,
                                                                                 samplesAxis,
                                                                                 bandsAxis);

    return {rectified, ShiftsInfos{.horizontal={}, .vertical=orderEstimated}};
}

template<bool verbose>
RectifiedResult verticalHorizontal(Multidim::Array<float,3> const& image,
                                             int hwindow,
                                             int vradius,
                                             double dx_pos_lambda,
                                             int nIterations,
                                             double residual_treshold,
                                             int linesAxis,
                                             int samplesAxis,
                                             int bandsAxis) {

    constexpr bool smoothOutHorizontal = true;

    auto rectificationDataInfos = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomHorizontalVerticalBayesian<verbose>(
        image,
        hwindow,
        vradius,
        dx_pos_lambda,
        nIterations,
        residual_treshold,
        smoothOutHorizontal,
        linesAxis,
        samplesAxis,
        bandsAxis);


    auto& rectificationData = rectificationDataInfos.value();

    //auto accumulated = PikaLTools::PushBroomRelativeOffsets::computeAccumulatedFromRelativeShifts(rectificationData);

    if (verbose) {

        float minxShift = std::numeric_limits<float>::infinity(); // accumulated.minDelta;
        float maxxShift = -std::numeric_limits<float>::infinity(); // accumulated.maxDelta;

        float minyShift = std::numeric_limits<float>::infinity(); // accumulated.minDelta;
        float maxyShift = -std::numeric_limits<float>::infinity(); // accumulated.maxDelta;

        for (int i = 0; i < rectificationData.dx_shifts.size(); i++) {
            float const& dx = rectificationData.dx_shifts[i];

            minxShift = std::min(dx, minxShift);
            maxxShift = std::max(dx, maxxShift);
        }

        for (int i = 0; i < rectificationData.y_idx.size(); i++) {
            int dy = rectificationData.y_idx[i] - i;

            minyShift = std::min<float>(dy, minyShift);
            maxyShift = std::max<float>(dy, maxyShift);

        }

        std::cout << "\tShifts stats: min x : " << minxShift << " max x : " << maxxShift << " min y : " << int(minyShift) << " max y : " << int(maxyShift) << "\n";
        std::cout << "Start rectifiying image..." << std::endl;
    }

    Multidim::Array<float,3> rectified = PikaLTools::PushBroomRelativeOffsets::computeHorizontallyRectifiedVerticallyReorderedImage(
        image,
        rectificationData.dx_shifts,
        rectificationData.y_idx,
        linesAxis,
        samplesAxis,
        bandsAxis);

    return {rectified, ShiftsInfos{.horizontal=rectificationData.dx_shifts, .vertical=rectificationData.y_idx}};

}

int main(int argc, char** argv) {

    TCLAP::CmdLine cmd("Try and undistort a bil file", '=', "0.0");

    std::vector<std::string> allowedSubtools;
    allowedSubtools.push_back("horizontal");
    allowedSubtools.push_back("vertical");
    allowedSubtools.push_back("both");
    TCLAP::ValuesConstraint<std::string> allowedVals(allowedSubtools);

    TCLAP::UnlabeledValueArg<std::string> subtoolArg("subTool", "Subtool to use", true, "", &allowedVals);

    TCLAP::UnlabeledValueArg<std::string> bilFilePathArg("bildFilePath", "Path where the bil is stored", true, "", "local path to bil file");

    cmd.add(subtoolArg);
    cmd.add(bilFilePathArg);

    TCLAP::ValueArg<int> startLineArg("s","lineStart", "initial line of the section to treat", false, 0, "int");
    TCLAP::ValueArg<int> endLineArg("e","endStart", "final line of the section to treat", false, -1, "int");

    cmd.add(startLineArg);
    cmd.add(endLineArg);

    TCLAP::ValueArg<int> hRadiusArg("","hradius", "horizontal radius for which to consider pixels error model", false, 16, "int");
    TCLAP::ValueArg<int> vradiusArg("","vradius", "vertical radius for which to consider pixels error model", false, 1, "int");

    cmd.add(hRadiusArg);
    cmd.add(vradiusArg);

    TCLAP::ValueArg<std::string> outBilFilePathArg("o", "output-file", "Path where the rectified bil is stored", false, "", "local path to bil file");

    cmd.add(outBilFilePathArg);

    cmd.parse(argc, argv);

    std::string tool(subtoolArg.getValue());

    std::string filename(bilFilePathArg.getValue());

    std::string outfilename(outBilFilePathArg.getValue());

    int startLine = startLineArg.getValue();
    int endLine = endLineArg.getValue();

    int hradius = hRadiusArg.getValue();
    int vradius = vradiusArg.getValue();

    Multidim::Array<float,3> bil = read_bil_sequence_to_float({filename}, startLine, endLine);

    if (bil.empty()) {
        std::cerr << "Could not read bil file: " << filename << std::endl;
        return 1;
    }

    Multidim::Array<float,3> normalized = StereoVision::ImageProcessing::normalizeImageChannels(bil);

    double x_pos_lambda = 0.01;
    double y_pos_lambda = 1;

    double dx_pos_lambda = 1.0;
    double dy_pos_lambda = 0.1;

    double I_lambda = 0.5;

    int nIterations = 500;

    double residual_treshold = 1e-3;

    double damping_factor = 0.1;

    int linesAxis = 0;
    int samplesAxis = 1;
    int bandsAxis = 2;

    constexpr bool verbose = true;

    RectifiedResult rectifiedResults;

    if (verbose) {
        std::cout << "Start estimating distortion..." << std::endl;
    }

    if (tool == "horizontal") {
        rectifiedResults = horizontalOnly<verbose>(normalized,
                                            hradius,
                                            dx_pos_lambda,
                                            nIterations,
                                            residual_treshold,
                                            linesAxis,
                                            samplesAxis,
                                            bandsAxis);

    } else if (tool == "vertical") {
        rectifiedResults = verticalOnly<verbose>(normalized,
                                          hradius,
                                          vradius,
                                          linesAxis,
                                          samplesAxis,
                                          bandsAxis);
    } else if (tool == "both") {
        rectifiedResults = verticalHorizontal<verbose>(normalized,
                                                hradius,
                                                vradius,
                                                dx_pos_lambda,
                                                nIterations,
                                                residual_treshold,
                                                linesAxis,
                                                samplesAxis,
                                                bandsAxis);
    }

    Multidim::Array<float,3>& rectified = rectifiedResults.rectified;

    if (rectified.empty()) {

        std::cerr << "Failed to rectify image!" << std::endl;
        return 1;
    }

    if (verbose) {
        std::cout << "Image rectified!" << std::endl;
    }

    float blackLevel = 0;
    float whiteLevel = 1;


    QApplication app(argc, argv);


    HyperspectralSimplePseudocolorDisplayAdapter<float> distortedViewAdapter(&normalized,
                                                                             blackLevel,
                                                                             whiteLevel,
                                                                             samplesAxis,
                                                                             linesAxis,
                                                                             bandsAxis,
                                                                             {0,1,2});

    QImageDisplay::ImageWindow distortedViewWindow;
    distortedViewWindow.setImage(&distortedViewAdapter);

    distortedViewWindow.setWindowTitle("Original image");
    distortedViewWindow.show();


    HyperspectralSimplePseudocolorDisplayAdapter<float> rectifiedViewAdapter(&rectified,
                                                                             blackLevel,
                                                                             whiteLevel,
                                                                             samplesAxis,
                                                                             linesAxis,
                                                                             bandsAxis,
                                                                             {0,1,2});

    QImageDisplay::ImageWindow rectifiedViewWindow;
    rectifiedViewWindow.setImage(&rectifiedViewAdapter);

    rectifiedViewWindow.setWindowTitle("Rectified image");
    rectifiedViewWindow.show();


    int code = app.exec();

    if (!outfilename.empty()) {

        write_envi_bil(rectified, outfilename, {}, linesAxis, samplesAxis, bandsAxis);

        std::string shifts_outfilename = outfilename + ".shifts";

        std::fstream shiftsFile(shifts_outfilename, std::fstream::out);

        shiftsFile << "#dx, y_idx\n";

        int linesToWrite = 0;

        if (!rectifiedResults.infos.horizontal.empty()) {
            linesToWrite = rectifiedResults.infos.horizontal.size();
        } else if (!rectifiedResults.infos.vertical.empty()) {
            linesToWrite = rectifiedResults.infos.vertical.size();
        }

        for (int i = 0; i < linesToWrite; i++) {
            float dx = 0;
            float y_idx = i;

            if (!rectifiedResults.infos.horizontal.empty()) {
                dx = rectifiedResults.infos.horizontal[i];
            }

            if (!rectifiedResults.infos.vertical.empty()) {
                y_idx = rectifiedResults.infos.vertical[i];
            }

            shiftsFile << dx << "," << y_idx << "\n";
        }

    }

    return code;
}
