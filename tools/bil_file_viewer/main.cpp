#include "libs/io/read_envi_bil.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>

#include <StereoVision/utils/types_manipulations.h>

#include "gui/nextandpreviousframeeventfilter.h"
#include "gui/hyperspectralslicedisplayadapter.h"
#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"

#include "processing/darkframestools.h"

#include <tclap/CmdLine.h>

struct ArgStruct {
    int argc;
    char** argv;

    TCLAP::UnlabeledValueArg<std::string>& bilFilePathArg;
    TCLAP::UnlabeledValueArg<std::string>& blackFramePathArg;

    TCLAP::ValueArg<int>& blackArg;
    TCLAP::ValueArg<int>& whiteArg;
};

template<typename T>
int displayBilImage(std::string const& filename, ArgStruct& args) {

    QTextStream out(stdout);

    if (!envi_bil_img_match_type<T>(filename)) {
        out << "displayBilImage called with mismatched type" << Qt::endl;
        return 1;
    }


    auto header = readHeaderData(filename);

    if (!header.has_value()) {
        out << "Missing header file!" << Qt::endl;
        return 1;
    }

    Multidim::Array<T,3> spectral_data = read_envi_bil<T>(filename);

    if (spectral_data.empty()) {
        out << "Image is empty, or could not read image data!" << Qt::endl;
        return 1;
    }

    if (args.blackFramePathArg.isSet()) {
        std::string darkframe = args.blackFramePathArg.getValue();

        if (!envi_bil_img_match_type<T>(darkframe)) {
            out << "dark frame has mismatched type" << Qt::endl;
        } else {
            Multidim::Array<T,3> dark_frame_data = read_envi_bil<T>(darkframe);
            Multidim::Array<T,2> meanDrakFrame = PikaLTools::averageDarkFrame(dark_frame_data);
            spectral_data = PikaLTools::subtractDarkFrame(spectral_data, meanDrakFrame);
        }
    }

    std::map<std::string, std::string> headerData = header.value();

    QApplication app(args.argc, args.argv);

    T blackLevel = args.blackArg.getValue();
    T whiteLevel = args.whiteArg.getValue();

    if ( whiteLevel <= 0) {
        try {
            whiteLevel = std::stoi(headerData["ceiling"]);
        }
        catch(std::invalid_argument const& e) {
            out << "Invalid ceiling value in header file!" << Qt::endl;
            return 1;
        }
    }

    if ( whiteLevel <= 0) {
        whiteLevel = spectral_data.valueUnchecked(0,0,0);

        for (int i = 0; i < spectral_data.shape()[0]; i++) {
            for (int j = 0; j < spectral_data.shape()[1]; j++) {
                for (int k = 0; k < spectral_data.shape()[2]; k++) {
                    T val = spectral_data.valueUnchecked(i,j,k);

                    if (val > whiteLevel) {
                        whiteLevel = val;
                    }
                }
            }
        }
    }

    QVector<float> waveLengths;

    if (headerData.count("wavelength") > 0) {
        QString elements = QString::fromStdString(headerData["wavelength"]).mid(1,headerData["wavelength"].size()-2);
        QStringList splittedElements = elements.split(",");
        waveLengths.reserve(splittedElements.size());

        for (QString const& element : splittedElements) {
            bool ok;
            float value = element.toFloat(&ok);

            if (ok) {
                waveLengths.push_back(value);
            }
        }
    }

    constexpr int ScanLinesAxis = 0;
    constexpr int SamplesAxis = 1;
    constexpr int SpectralAxis = 2;

    int line_viewer_xAxis = SamplesAxis;
    int line_viewer_yAxis = SpectralAxis;
    int line_viewer_channelAxis = ScanLinesAxis;

    int image_viewer_xAxis = SamplesAxis;
    int image_viewer_yAxis = ScanLinesAxis;
    int image_viewer_channelAxis = SpectralAxis;

    int selectedChannel = 0;

    HyperspectralSliceDisplayAdapter<T> lineViewAdapter(&spectral_data, blackLevel, whiteLevel, line_viewer_xAxis, line_viewer_yAxis, line_viewer_channelAxis, selectedChannel);

    QImageDisplay::ImageWindow lineViewWindow;
    lineViewWindow.setImage(&lineViewAdapter);

    NextAndPreviousFrameEventFilter nextAndPreviousFilter;
    QObject::connect(&nextAndPreviousFilter, &NextAndPreviousFrameEventFilter::nextFrameRequested, [&lineViewAdapter, &lineViewWindow] () {
        lineViewAdapter.nextChannel();
        lineViewWindow.setWindowTitle(QString("BIL slice #%1 view").arg(lineViewAdapter.getChannel()));
    });
    QObject::connect(&nextAndPreviousFilter, &NextAndPreviousFrameEventFilter::previousFrameRequested, [&lineViewAdapter, &lineViewWindow] () {
        lineViewAdapter.previousChannel();
        lineViewWindow.setWindowTitle(QString("BIL slice #%1 view").arg(lineViewAdapter.getChannel()));
    });

    lineViewWindow.installEventFilter(&nextAndPreviousFilter);

    lineViewWindow.setWindowTitle(QString("BIL slice #%1 view").arg(lineViewAdapter.getChannel()));
    lineViewWindow.show();

    std::array<int, 3> colorChannels = {spectral_data.shape()[SpectralAxis]*3/4,
                                        spectral_data.shape()[SpectralAxis]/2,
                                        spectral_data.shape()[SpectralAxis]/4};

    if (waveLengths.size() == spectral_data.shape()[SpectralAxis]) {

        std::array<float, 3> referenceWl = {630, 532, 465};
        std::array<float, 3> currentWl = {waveLengths[colorChannels[0]], waveLengths[colorChannels[1]], waveLengths[colorChannels[2]]};


        for (int i = 0; i < waveLengths.size(); i++) {
            for (int c = 0; c < 3; c++) {

                float candDelta = std::fabs(waveLengths[i] - referenceWl[c]);
                float currentDelta = std::fabs(currentWl[c] - referenceWl[c]);

                if (candDelta < currentDelta) {
                    currentWl[c] = waveLengths[i];
                    colorChannels[c] = i;
                }
            }
        }

    }

    HyperspectralSimplePseudocolorDisplayAdapter<T> imageViewAdapter(&spectral_data,
                                                                     blackLevel,
                                                                     whiteLevel,
                                                                     image_viewer_xAxis,
                                                                     image_viewer_yAxis,
                                                                     image_viewer_channelAxis,
                                                                     colorChannels);

    imageViewAdapter.configureOriginalChannelDisplay(
                QString("Band %1").arg(colorChannels[0]),
            QString("Band %1").arg(colorChannels[1]),
            QString("Band %1").arg(colorChannels[2]));

    QImageDisplay::ImageWindow imageViewWindow;
    imageViewWindow.setImage(&imageViewAdapter);

    imageViewWindow.setWindowTitle("Pseudo color image");
    imageViewWindow.show();


    return app.exec();
}

int main(int argc, char** argv) {

    QTextStream out(stdout);

    TCLAP::CmdLine cmd("View a bill file", '=', "0.0");

    TCLAP::UnlabeledValueArg<std::string> bilFilePathArg("bildFilePath", "Path where the bil is stored", true, "", "local path to bil file");
    TCLAP::UnlabeledValueArg<std::string> blackFramePathArg("blackFramePath", "path to the black frame to use", false, "", "path to black frame bil");

    cmd.add(bilFilePathArg);
    cmd.add(blackFramePathArg);

    TCLAP::ValueArg<int> blackArg("b","blackLevel", "black level to use for display", false, 0, "int");
    TCLAP::ValueArg<int> whiteArg("w","whiteLevel", "white level to use for display", false, 0, "int");

    cmd.add(blackArg);
    cmd.add(whiteArg);

    cmd.parse(argc, argv);

    ArgStruct argStruct{argc, argv, bilFilePathArg, blackFramePathArg, blackArg, whiteArg};

    std::string filename(bilFilePathArg.getValue());

    if (envi_bil_img_match_type<uint8_t>(filename)) {
        return displayBilImage<uint8_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<int8_t>(filename)) {
        return displayBilImage<int8_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<uint16_t>(filename)) {
        return displayBilImage<uint16_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<int16_t>(filename)) {
        return displayBilImage<int16_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<uint32_t>(filename)) {
        return displayBilImage<uint32_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<int32_t>(filename)) {
        return displayBilImage<int32_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<uint64_t>(filename)) {
        return displayBilImage<uint64_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<int64_t>(filename)) {
        return displayBilImage<int64_t>(filename, argStruct);
    }

    if (envi_bil_img_match_type<float>(filename)) {
        return displayBilImage<float>(filename, argStruct);
    }

    if (envi_bil_img_match_type<double>(filename)) {
        return displayBilImage<double>(filename, argStruct);
    }

    out << "Unsupported image type!" << Qt::endl;
    return 1;
}
