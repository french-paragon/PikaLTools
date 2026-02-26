#include "libs/io/read_envi_bil.h"

#include "libs/processing/relativeoffsetsestimator.h"

#include <StereoVision/io/image_io.h>
#include <StereoVision/imageProcessing/colorConversions.h>

#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <tclap/CmdLine.h>
#include <QApplication>

#include <iostream>

int main(int argc, char** argv) {

    TCLAP::CmdLine cmd("Try and undistort a bil file", '=', "0.0");

    TCLAP::UnlabeledValueArg<std::string> bilFilePathArg("bildFilePath", "Path where the bil is stored", true, "", "local path to bil file");

    cmd.add(bilFilePathArg);

    TCLAP::ValueArg<int> startLineArg("s","lineStart", "initial line of the section to treat", false, 0, "int");
    TCLAP::ValueArg<int> endLineArg("e","endStart", "final line of the section to treat", false, -1, "int");

    cmd.add(startLineArg);
    cmd.add(endLineArg);

    TCLAP::ValueArg<int> hRadiusArg("","hradius", "horizontal radius for which to consider pixels error model", false, 16, "int");
    TCLAP::ValueArg<int> vradiusArg("","vradius", "vertical radius for which to consider pixels error model", false, 1, "int");

    cmd.add(hRadiusArg);
    cmd.add(vradiusArg);

    cmd.parse(argc, argv);

    std::string filename(bilFilePathArg.getValue());

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

    double dx_pos_lambda = 0.1;
    double dy_pos_lambda = 0.1;

    double I_lambda = 0.5;

    int nIterations = 500;

    double residual_treshold = 1e-3;

    double damping_factor = 0.1;

    int linesAxis = 0;
    int samplesAxis = 1;
    int bandsAxis = 2;

    constexpr bool verbose = true;

    std::cout << "Start estimating distortion..." << std::endl;

    auto rectificationDataInfos = PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>(
        normalized,
        hradius,
        vradius,
        x_pos_lambda,
        y_pos_lambda,
        dx_pos_lambda,
        dy_pos_lambda,
        I_lambda,
        nIterations,
        residual_treshold,
        damping_factor,
        linesAxis,
        samplesAxis,
        bandsAxis);

    /*auto rectificationDataInfos = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomHorizontalShiftBayesian<verbose>(
        normalized,
        hradius,
        dx_pos_lambda,
        nIterations,
        residual_treshold,
        linesAxis,
        samplesAxis,
        bandsAxis);*/

    auto& rectificationData = rectificationDataInfos.value();

    //auto accumulated = PikaLTools::PushBroomRelativeOffsets::computeAccumulatedFromRelativeShifts(rectificationData);

    float minxShift = std::numeric_limits<float>::infinity(); // accumulated.minDelta;
    float maxxShift = -std::numeric_limits<float>::infinity(); // accumulated.maxDelta;

    float minyShift = std::numeric_limits<float>::infinity();
    float maxyShift = -std::numeric_limits<float>::infinity();

    for (int i = 0; i < rectificationData.size(); i++) {
        PikaLTools::PushBroomRelativeOffsets::LineShiftInfos const& shiftsInfos = rectificationData[i];

        minxShift = std::min(shiftsInfos.dx, minxShift);
        maxxShift = std::max(shiftsInfos.dx, maxxShift);

        minyShift = std::min(shiftsInfos.y-i, minyShift);
        maxyShift = std::max(shiftsInfos.y-i, maxyShift);
    }

    std::cout << "\tShifts stats: min x : " << minxShift << " max x : " << maxxShift << " min y : " << minyShift << " max y : " << maxyShift << "\n";

    std::cout << "Start rectifiying image..." << std::endl;

    Multidim::Array<float,3> rectified = PikaLTools::PushBroomRelativeOffsets::computeHorizontallyRectifiedVerticallyReorderedImage(
        normalized,
        rectificationData,
        linesAxis,
        samplesAxis,
        bandsAxis);

    /*constexpr bool shiftsAccumulated = true;

    Multidim::Array<float,3> rectified = PikaLTools::PushBroomRelativeOffsets::computeHorizontallyRectifiedImage<shiftsAccumulated>(
        normalized,
        accumulated.accumulatedShifts,
        linesAxis,
        samplesAxis,
        bandsAxis);*/

    std::cout << "Image rectified!" << std::endl;

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


    return app.exec();
}
