#include "libs/io/read_envi_bil.h"

#include <qImageDisplayWidget/imagewindow.h>
#include <qImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>

#include <LibStevi/utils/types_manipulations.h>

#include "gui/nextandpreviousframeeventfilter.h"
#include "gui/hyperspectralslicedisplayadapter.h"
#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"


template<typename T>
int displayBilImage(std::string const& filename, int argc, char** argv) {

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

    std::map<std::string, std::string> headerData = header.value();

    QApplication app(argc, argv);

    T blackLevel = 0;
    T whiteLevel;

    if (headerData.count("ceiling") <= 0) {
        out << "Missing ceiling data in header!" << Qt::endl;
        return 1;
    }

    try {
        whiteLevel = std::stoi(headerData["ceiling"]);
    }
    catch(std::invalid_argument const& e) {
        out << "Invalid ceiling value in header file!" << Qt::endl;
        return 1;
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

    std::array<int, 3> colorChannels = {spectral_data.shape()[SpectralAxis]/4,
                                        spectral_data.shape()[SpectralAxis]/2,
                                        spectral_data.shape()[SpectralAxis]-spectral_data.shape()[SpectralAxis]/4};

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
    QImageDisplay::ImageWindow imageViewWindow;
    imageViewWindow.setImage(&imageViewAdapter);

    imageViewWindow.setWindowTitle("Pseudo color image");
    imageViewWindow.show();


    return app.exec();
}

int main(int argc, char** argv) {

    QTextStream out(stdout);

    if (argc != 2) {
        out << "Missing input image argument" << Qt::endl;
        return 1;
    }

    std::string filename(argv[1]);

    if (envi_bil_img_match_type<uint8_t>(filename)) {
        return displayBilImage<uint8_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<int8_t>(filename)) {
        return displayBilImage<int8_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<uint16_t>(filename)) {
        return displayBilImage<uint16_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<int16_t>(filename)) {
        return displayBilImage<int16_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<uint32_t>(filename)) {
        return displayBilImage<uint32_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<int32_t>(filename)) {
        return displayBilImage<int32_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<uint64_t>(filename)) {
        return displayBilImage<uint64_t>(filename, argc, argv);
    }

    if (envi_bil_img_match_type<int64_t>(filename)) {
        return displayBilImage<int64_t>(filename, argc, argv);
    }

    out << "Unsupported image type!" << Qt::endl;
    return 1;
}
