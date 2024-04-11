#include "libs/io/read_aviris4_bin.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>

#include <StereoVision/utils/types_manipulations.h>

#include "gui/nextandpreviousframeeventfilter.h"
#include "gui/hyperspectralslicedisplayadapter.h"
#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"

#include <cmath>


int displayAviris4Image(std::string const& filename, int argc, char** argv) {

    QTextStream out(stdout);

    aviris4io::Aviris4FrameData spectral_data = aviris4io::loadFrame(filename);

    if (spectral_data.frame.empty()) {
        out << "Image is empty, or could not read image data!" << Qt::endl;
        return 1;
    }

    QApplication app(argc, argv);

    aviris4io::data_t blackLevel = 0;
    aviris4io::data_t whiteLevel = 0xffff;

    constexpr float min = 380;
    constexpr float max = 2490;
    constexpr float delta = max - min;

    int nBands = spectral_data.frame.shape()[2];

    QVector<float> waveLengths(nBands);

    //compute the bands wavelengths
    for (int i = 0; i < nBands; i++) {
        waveLengths[i] = min + delta*i/(nBands-1);
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

    HyperspectralSliceDisplayAdapter<aviris4io::data_t> lineViewAdapter(&spectral_data.frame,
                                                               blackLevel,
                                                               whiteLevel,
                                                               line_viewer_xAxis,
                                                               line_viewer_yAxis,
                                                               line_viewer_channelAxis, selectedChannel);

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

    std::array<int, 3> colorChannels = {spectral_data.frame.shape()[SpectralAxis]*3/4,
                                        spectral_data.frame.shape()[SpectralAxis]/2,
                                        spectral_data.frame.shape()[SpectralAxis]/4};

    if (waveLengths.size() == spectral_data.frame.shape()[SpectralAxis]) {

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

    HyperspectralSimplePseudocolorDisplayAdapter<aviris4io::data_t> imageViewAdapter(&spectral_data.frame,
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

    return displayAviris4Image(filename, argc, argv);
}
