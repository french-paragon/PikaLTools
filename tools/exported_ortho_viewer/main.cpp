#include <StereoVision/io/image_io.h>
#include <StereoVision/utils/types_manipulations.h>

#include <QApplication>

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/gui/arraydisplayadapter.h>


#include <QTextStream>
#include <QString>
#include <QMap>
#include <QVector>

template<typename T>
int displayImage(std::string const& imageName, QMap<QString, QString> const& options, QApplication & application, QTextStream & outStream) {

    constexpr int nDim = 3;

    T blackLevel = StereoVision::TypesManipulations::defaultBlackLevel<T>();
    T whiteLevel = StereoVision::TypesManipulations::defaultWhiteLevel<T>();

    if (options.contains("--blacklevel")) {
        bool ok;
        if (std::is_floating_point_v<T>) {
            double val = options["--blacklevel"].toDouble(&ok);
            if (ok) {
                blackLevel = val;
            }
        } else {
            long val = options["--blacklevel"].toLong(&ok);
            if (ok) {
                blackLevel = val;
            }
        }
    }

    if (options.contains("--whitelevel")) {
        bool ok;
        if (std::is_floating_point_v<T>) {
            double val = options["--whitelevel"].toDouble(&ok);
            if (ok) {
                whiteLevel = val;
            }
        } else {
            long val = options["--whitelevel"].toLong(&ok);
            if (ok) {
                whiteLevel = val;
            }
        }
    }

    outStream << "Read image: " << QString(imageName.c_str()) << Qt::endl;

    Multidim::Array<T, nDim> datacube = StereoVision::IO::readStevimg<T, nDim>(imageName);

    if (datacube.empty()) {
        outStream << "impossible to read image: " << QString(imageName.c_str()) << Qt::endl;
        return 1;
    } else {
        outStream << "Read image: " << QString(imageName.c_str()) << Qt::endl;
        outStream << "Image shape: " << datacube.shape()[0] << "x" << datacube.shape()[1] << "x" <<  datacube.shape()[2] << Qt::endl;
    }

    std::array<int,3> selectedChannels = {datacube.shape()[2]-1, datacube.shape()[2]/2, 0};

    if (options.contains("--red_channel")) {
        selectedChannels[0] = options.value("--red_channel").toInt();
    }
    if (options.contains("--green_channel")) {
        selectedChannels[1] = options.value("--green_channel").toInt();
    }
    if (options.contains("--blue_channel")) {
        selectedChannels[2] = options.value("--blue_channel").toInt();
    }

    Multidim::Array<T, nDim> img(datacube.shape()[0], datacube.shape()[1], 3);

    std::array<T,3> maxCol = {0,0,0};

    for (int i = 0; i < datacube.shape()[0]; i++) {
        for (int j = 0; j < datacube.shape()[1]; j++) {

            for (int c = 0; c < 3; c++) {
                T val = datacube.atUnchecked(i,j,selectedChannels[c]);
                img.atUnchecked(i,j,c) = val;

                if (val > maxCol[c]) {
                    maxCol[c] = val;
                }
            }

        }
    }

    outStream << "max colors: R = " << maxCol[0] << " G = " << maxCol[1] << " B = " << maxCol[2] << Qt::endl;

    StereoVision::Gui::ArrayDisplayAdapter<T> cimgAdapter(&img, blackLevel, whiteLevel);

    QImageDisplay::ImageWindow imgWindow;

    imgWindow.setImage(&cimgAdapter);

    if (options.contains("--title")) {
        imgWindow.setWindowTitle(options.value("--title"));
    } else {
        imgWindow.setWindowTitle("Orthophoto viewer");
    }

    imgWindow.show();
    return application.exec();

}

int main(int argc, char** argv) {

    QApplication app(argc, argv);

    QVector<QString> arguments;
    QMap<QString, QString> options;

    for (int i = 1; i < argc; i++) {
        QString input(argv[i]);

        if (input.startsWith("-")) {
            QStringList split = input.split("=");
            if (split.size() == 1) {
                options.insert(split[0], "");
            } else {
                options.insert(split[0], split[1]);
            }

        } else {
            arguments.push_back(input);
        }
    }

    QTextStream out(stdout);

    if (arguments.size() < 1) { //no input image
        out << "No input image provided" << Qt::endl;
        return 1;
    }

    std::string inFileName = arguments[0].toStdString();

    out << "In file name:" << inFileName.c_str() << Qt::endl;

    //8 bit integer
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<int8_t, 3>(inFileName)) {
        return displayImage<int8_t>(inFileName, options, app, out);
    }

    if (StereoVision::IO::stevImgFileMatchTypeAndDim<uint8_t, 3>(inFileName)) {
        return displayImage<uint8_t>(inFileName, options, app, out);
    }

    //16 bit integer
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<int16_t, 3>(inFileName)) {
        return displayImage<int16_t>(inFileName, options, app, out);
    }

    if (StereoVision::IO::stevImgFileMatchTypeAndDim<uint16_t, 3>(inFileName)) {
        return displayImage<uint16_t>(inFileName, options, app, out);
    }

    //32 bit integer
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<int32_t, 3>(inFileName)) {
        return displayImage<int32_t>(inFileName, options, app, out);
    }

    if (StereoVision::IO::stevImgFileMatchTypeAndDim<uint32_t, 3>(inFileName)) {
        return displayImage<uint32_t>(inFileName, options, app, out);
    }

    //64 bit integer
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<int64_t, 3>(inFileName)) {
        return displayImage<int64_t>(inFileName, options, app, out);
    }

    if (StereoVision::IO::stevImgFileMatchTypeAndDim<uint64_t, 3>(inFileName)) {
        return displayImage<uint64_t>(inFileName, options, app, out);
    }

    //floating points
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<float, 3>(inFileName)) {
        return displayImage<float>(inFileName, options, app, out);
    }

    //double precision floating points
    if (StereoVision::IO::stevImgFileMatchTypeAndDim<double, 3>(inFileName)) {
        return displayImage<double>(inFileName, options, app, out);
    }

    out << "Input image does not match expected type or shape for a displayable image!" << Qt::endl;
    out << "The image must have 3 dimensions and be a numerical array!" << Qt::endl;
    return 1;
}
