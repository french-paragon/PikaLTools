#include "libs/io/read_envi_bil.h"
#include "libs/geo/coordinate_conversions.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>
#include <QVector>

#include <StereoVision/utils/types_manipulations.h>

#include <StereoVision/gui/arraydisplayadapter.h>
#include "gui/hyperspectralsimplepseudocolordisplayadapter.h"

#include <StereoVision/geometry/core.h>

#include <MultidimArrays/MultidimArrays.h>

using LinePose = StereoVision::Geometry::AffineTransform<double>;

/*!
 * \brief getScalinesPoseValues get the pose of a camera for each scanlines in a hypercube.
 * \param files the bil files which makes up the hyperspectral cube
 * \return a vector with the pose given as transformation from the camera frame to a local cartesian frame.
 */
QVector<LinePose> getScalinesPoseValues(QVector<QString> const& files, bool flipZ = true) {

    QVector<LinePose> accumulated;

    double meanLat = 0;
    double meanLon = 0;

    for (QString const& file : files) {

        auto header = readHeaderData(file.toStdString());

        if (!header.has_value()) {
            return QVector<LinePose>();
        }

        std::map<std::string, std::string> headerData = header.value();

        float samplingFreq;
        float nLines;

        if (headerData.count("lines") <= 0 or headerData.count("framerate") <= 0) {
            return QVector<LinePose>();
        }

        try {
            samplingFreq = std::stof(headerData["framerate"]);
            nLines = std::stof(headerData["lines"]);
        }
        catch(std::invalid_argument const& e) {
            return QVector<LinePose>();
        }

        double timePerFrame = 1/samplingFreq;

        std::vector<EnviBilLcfLine> lines = read_envi_bil_lcf_data(file.toStdString());

        accumulated.reserve(accumulated.size() + lines.size());

        double currentLineTime = lines[0].timeStamp;

        for (int i = 0; i < lines.size()-1; i++) {
            EnviBilLcfLine const& line = lines[i];
            EnviBilLcfLine const& nline = lines[i+1];

            while (line.timeStamp <= currentLineTime and nline.timeStamp >= currentLineTime) {

                double timeRatio = (currentLineTime-line.timeStamp)/(nline.timeStamp-line.timeStamp);

                double lat= (1-timeRatio)*line.lat + timeRatio*nline.lat;
                double lon = (1-timeRatio)*line.lon + timeRatio*nline.lon;
                double alt = (1-timeRatio)*line.height + timeRatio*nline.height;

                auto interpolateAngle = [] (double coeff, double a1, double a2) {
                    if (std::fabs(a1 - a2) > M_PI) {
                        double interp = (1-coeff)*(a1) + coeff*(a2);
                        if (interp < 0) {
                            return interp + M_PI;
                        } else {
                            return interp - M_PI;
                        }
                    } else {
                        return (1-coeff)*(a1) + coeff*(a2);
                    }
                };

                double roll= interpolateAngle(timeRatio, line.roll, nline.roll);
                double pitch = interpolateAngle(timeRatio, line.pitch, nline.pitch);
                double yaw = interpolateAngle(timeRatio, line.yaw, nline.yaw);

                meanLat += lat;
                meanLon += lon;

                CartesianCoord<double> ecefPosition = convertLatLonToECEF<double>(lat, lon, alt);

                Eigen::Vector3d position(ecefPosition.x, ecefPosition.y, ecefPosition.z);

                Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

                Eigen::Matrix<double, 3,3> Attitude = (yawAngle*pitchAngle*rollAngle).matrix();

                if (flipZ) {

                    Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
                    flip(1,1)=-1;
                    flip(2,2)=-1;

                    Attitude = Attitude*flip;
                }

                accumulated.push_back(LinePose(Attitude, position));

                currentLineTime += timePerFrame;

                if (accumulated.size() == nLines) {
                    break;
                }
            }

            if (accumulated.size() == nLines) {
                break;
            }
        }
    }

    meanLat /= accumulated.size();
    meanLon /= accumulated.size();

    StereoVision::Geometry::AffineTransform<double> ecef2local = getLocalFrameAtPos<double>(meanLat, meanLon);

    for (LinePose & pose : accumulated) {

        pose.t = ecef2local*pose.t;

        //pose
        //pose = LinePose(pose.R.transpose(), -pose.R.transpose()*pose.t);
    }

    return accumulated;
}

template<typename T>
struct PatchBand {
    Multidim::Array<T,3> patchBand;
    Multidim::Array<float,2> nSamples;
};

/*!
 * \brief extractPatchBand extract an approximate image band from the raw hypercube
 * \param hypercube the raw hypercube
 * \param positionData the position of each scaline
 * \param line the index of the line around which the patch needs to be extracted
 * \param fieldOfView the field of view of the camera
 * \param radius the number of pixels the patch needs to be extended from on both side of the line
 * \param maxLineDelta the maximum number of lines to consider on both side of the scaline
 * \param groundDist an estimate of the ground distance for the image.
 * \return a patchband struct containing the extracted patch, as well as the number of samples for each pixels.
 */
template<typename T>
PatchBand<T> extractPatchBand(Multidim::Array<T,3> const& hypercube,
                              QVector<LinePose> const& positionData,
                              int line,
                              float fieldOfView,
                              int radius,
                              int maxLineDelta,
                              float groundDist) {

    constexpr Multidim::AccessCheck Nc = Multidim::AccessCheck::Nocheck;

    if (line < 0 or line >= positionData.size()) {
        return PatchBand<T>();
    }

    if (positionData.size() != hypercube.shape()[LineAxis])  {
        return PatchBand<T>();
    }

    int nPixs = hypercube.shape()[SamplesAxis];
    int nBands = hypercube.shape()[BandsAxis];

    PatchBand<T> ret;
    ret.patchBand = Multidim::Array<T,3>(2*radius+1,nPixs,nBands);
    ret.nSamples = Multidim::Array<float,2>(2*radius+1,nPixs);

    for (int l = 0; l < 2*radius+1; l++) {
        for (int s = 0; s < nPixs; s++) {
            for (int b = 0; b < nBands; b++) {
                ret.patchBand.template at<Nc>(l,s,b) = 0;
            }

            ret.nSamples.template at<Nc>(l,s) = 0;
        }
    }

    float fov_rad = fieldOfView/180*M_PI;
    double normalizedSensorSize = 2*std::tan(fov_rad/2);
    double groundPixSize = groundDist* normalizedSensorSize/nPixs;
    double fLenPix = static_cast<double>(nPixs)/normalizedSensorSize;

    LinePose referenceLinePose = positionData[line]; //cam2local
    LinePose& refLine2local = referenceLinePose;
    LinePose local2refLine = LinePose(refLine2local.R.transpose(), -refLine2local.R.transpose()*refLine2local.t);

    Eigen::Vector3d direction(0,1,0);
    direction = referenceLinePose*direction;

    auto reprojectPixel = [nPixs, groundDist, groundPixSize, fLenPix]
            (int pixelId, LinePose const& currentLine2refLine) -> Eigen::Vector2d {

        Eigen::Vector3d currentViewDirectionInCurrentRef(pixelId-static_cast<double>(nPixs)/2.,0,fLenPix);

        Eigen::Vector3d currentLineOptioncalCenter = currentLine2refLine.t;

        Eigen::Vector3d rayDirection = currentLine2refLine.R*currentViewDirectionInCurrentRef;

        double t = (groundDist - currentLineOptioncalCenter.z())/rayDirection.z();

        Eigen::Vector3d planePos = currentLineOptioncalCenter+t*rayDirection;

        return Eigen::Vector2d(planePos.x(), planePos.y())/groundPixSize;

    };

    for (int dl = -maxLineDelta; dl <= maxLineDelta; dl++) {
        int cl = line + dl;

        if (cl < 0 or cl >= positionData.size()) {
            continue;
        }

        LinePose currentLine2refLine = local2refLine*positionData[cl];

        for (int pix = 0; pix < nPixs; pix++) {
            Eigen::Vector2d pos = reprojectPixel(pix, currentLine2refLine);

            int x0 = std::floor(pos.x() + static_cast<double>(nPixs)/2.);
            int x1 = std::ceil(pos.x() + static_cast<double>(nPixs)/2.);

            int y0 = std::floor(pos.y());
            int y1 = std::ceil(pos.y());

            double dx = pos.x() + static_cast<double>(nPixs)/2. - x0;
            double dy = pos.y() - y0;

            for (int x : {x0, x1}) {

                if (x < 0 or x >= nPixs) {
                    continue;
                }

                for (int y : {y0, y1}) {

                    if (std::abs(y) > radius) {
                        continue;
                    }

                    int xIdx = x;
                    int yIdx = y + radius;

                    double coeff = 1;

                    if (x == x0) {
                        coeff *= (1-dx);
                    } else {
                        coeff *= dx;
                    }

                    if (y == y0) {
                        coeff *= (1-dy);
                    } else {
                        coeff *= dy;
                    }

                    std::array<int, 3> patch_idx;

                    patch_idx[LineAxis] = yIdx;
                    patch_idx[SamplesAxis] = xIdx;

                    std::array<int, 3> hypercube_idx;

                    hypercube_idx[LineAxis] = cl;
                    hypercube_idx[SamplesAxis] = pix;

                    for (int b = 0; b < nBands; b++) {
                        patch_idx[BandsAxis] = b;
                        hypercube_idx[BandsAxis] = b;

                        ret.patchBand.template at<Nc>(patch_idx) += coeff*hypercube.valueUnchecked(hypercube_idx);
                    }


                    ret.nSamples.template at<Nc>(yIdx,xIdx) += coeff;

                }
            }
        }
    }

    for (int l = 0; l < 2*radius+1; l++) {
        for (int s = 0; s < nPixs; s++) {

            double coeff = ret.nSamples.template at<Nc>(l,s);

            if (coeff <= 0) {
                continue;
            }

            for (int b = 0; b < nBands; b++) {
                ret.patchBand.template at<Nc>(l,s,b) /= coeff;

                if (std::isinf(ret.patchBand.template at<Nc>(l,s,b)) or
                        std::isnan(ret.patchBand.template at<Nc>(l,s,b))) {
                    ret.patchBand.template at<Nc>(l,s,b) = 0;
                }
            }
        }
    }

    return ret;

}

template<typename T>
int showSelectedBand(QVector<QString> inputs, QMap<QString, QString> options, int argc, char** argv) {

    QTextStream out(stdout);

    int line = 0;
    int radius = 3;
    int maxLineDelta = 6;
    float fieldOfView = 35;
    float groundDist = 200;

    bool ok;

    if (options.contains("--line")) {
        line = options["--line"].toInt(&ok);

        if (!ok) {
            out << "Invalid --line option!" << Qt::endl;
            return 1;
        }
    }

    if (options.contains("--radius")) {
        radius = options["--radius"].toInt(&ok);

        if (!ok) {
            out << "Invalid --radius option!" << Qt::endl;
            return 1;
        }
    }

    if (options.contains("--maxLineDelta")) {
        maxLineDelta = options["--maxLineDelta"].toInt(&ok);

        if (!ok) {
            out << "Invalid --maxLineDelta option!" << Qt::endl;
            return 1;
        }
    } else {
        maxLineDelta = 2*radius;
    }

    auto header = readHeaderData(inputs[0].toStdString());

    if (!header.has_value()) {
        out << "Missing header file!" << Qt::endl;
        return 1;
    }

    std::map<std::string, std::string> headerData = header.value();

    if (options.contains("--fieldOfView")) {
        fieldOfView = options["--fieldOfView"].toFloat(&ok);

        if (!ok) {
            out << "Invalid --fieldOfView option!" << Qt::endl;
            return 1;
        }
    } else {

        if (headerData.count("field of view") <= 0) {
            out << "Missing field of view data in header!" << Qt::endl;
            return 1;
        }

        try {
            fieldOfView = std::stof(headerData["field of view"]);
        }
        catch(std::invalid_argument const& e) {
            out << "Invalid field of view data in header!" << Qt::endl;
            return 1;
        }
    }

    if (options.contains("--groundDist")) {
        groundDist = options["--groundDist"].toFloat(&ok);

        if (!ok) {
            out << "Invalid --groundDist option!" << Qt::endl;
            return 1;
        }
    }

    std::vector<std::string> std_inputs(inputs.size());

    for (int i = 0; i < inputs.size(); i++) {
        std_inputs[i] = inputs[i].toStdString();
    }

    Multidim::Array<T,3> hypercube = read_bil_sequence<T>(std_inputs);
    QVector<LinePose> linePoses = getScalinesPoseValues(inputs);

    if (hypercube.empty() or linePoses.empty()) {
        out << "Could not read input data!" << Qt::endl;
        return 1;
    }

    PatchBand<T> patchBand = extractPatchBand(hypercube, linePoses, line, fieldOfView, radius, maxLineDelta, groundDist);

    if (patchBand.patchBand.empty() or patchBand.nSamples.empty()) {
        out << "Could not compute patch band!" << Qt::endl;
        return 1;
    }

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
        return false;
    }

    QVector<float> waveLengths;

    if (headerData.count("wavelength") > 0) {

        QString elements = QString::fromStdString(headerData["wavelength"]);

        elements = elements.mid(1,headerData["wavelength"].size()-2);

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

    QApplication app(argc, argv);

    std::array<int, 3> colorChannels = {hypercube.shape()[BandsAxis]/4,
                                        hypercube.shape()[BandsAxis]/2,
                                        hypercube.shape()[BandsAxis]-hypercube.shape()[BandsAxis]/4};

    if (waveLengths.size() == hypercube.shape()[BandsAxis]) {

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

    int image_viewer_xAxis = SamplesAxis;
    int image_viewer_yAxis = LineAxis;
    int image_viewer_channelAxis = BandsAxis;

    HyperspectralSimplePseudocolorDisplayAdapter<T> imageViewAdapter(&(patchBand.patchBand),
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

    float minSamples = 0;
    float maxSamples = 1;

    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float> weightMapViewAdapter(&(patchBand.nSamples),
                                                                                minSamples,
                                                                                maxSamples,
                                                                                image_viewer_xAxis,
                                                                                image_viewer_yAxis
                                                                                );

    weightMapViewAdapter.configureOriginalChannelDisplay("#Samples");

    std::function<QColor(float)> colorMap = [] (float val) {

        QColor val0(24,68,129);
        QColor val05(1,132,71);
        QColor val1(253,236,132);

        if (val < 0) {
            return val0;
        }

        if (val < 0.5) {
            float w0 = 1 - val*2;
            float w05 = val*2;

            float r = static_cast<float>(val0.red())*w0 + val05.red()*w05;
            float g = static_cast<float>(val0.green())*w0 + val05.green()*w05;
            float b = static_cast<float>(val0.blue())*w0 + val05.blue()*w05;

            return QColor(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
        }

        if (val < 1.0) {
            float w05 = 2 - val*2;
            float w1 = val*2 - 1;

            float r = static_cast<float>(val05.red())*w05 + val1.red()*w1;
            float g = static_cast<float>(val05.green())*w05 + val1.green()*w1;
            float b = static_cast<float>(val05.blue())*w05 + val1.blue()*w1;

            return QColor(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
        }

        return val1;

    };

    weightMapViewAdapter.setColorMap(colorMap);

    QImageDisplay::ImageWindow nSamplesViewWindow;
    nSamplesViewWindow.setImage(&weightMapViewAdapter);

    nSamplesViewWindow.setWindowTitle("N samples per pixels");
    nSamplesViewWindow.show();

    return app.exec();

}

int main(int argc, char** argv) {

    QTextStream out(stdout);

    if (argc <= 0) {
        out << "Missing input arguments!" << Qt::endl;
        return 1;
    }

    QVector<QString> inputs;
    QMap<QString, QString> options;

    for (int a = 1; a < argc; a++) {

        QString val(argv[a]);

        if (val.startsWith("-")) {

            QString nextVal(argv[a+1]);
            options.insert(val, nextVal);
            a++;

        } else {
            inputs.push_back(val);
        }
    }

    std::string filename(argv[1]);

    if (envi_bil_img_match_type<uint8_t>(filename)) {
        return showSelectedBand<uint8_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<int8_t>(filename)) {
        return showSelectedBand<int8_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<uint16_t>(filename)) {
        return showSelectedBand<uint16_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<int16_t>(filename)) {
        return showSelectedBand<int16_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<uint32_t>(filename)) {
        return showSelectedBand<uint32_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<int32_t>(filename)) {
        return showSelectedBand<int32_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<uint64_t>(filename)) {
        return showSelectedBand<uint64_t>(inputs, options, argc, argv);
    }

    if (envi_bil_img_match_type<int64_t>(filename)) {
        return showSelectedBand<int64_t>(inputs, options, argc, argv);
    }

    out << "Unsupported image type!" << Qt::endl;
    return 1;

}
