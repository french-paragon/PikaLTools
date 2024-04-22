#include "libs/io/read_aviris4_bin.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>
#include <QFileInfo>
#include <QFile>
#include <QDir>

#include <StereoVision/utils/types_manipulations.h>

#include <cmath>

int main(int argc, char** argv) {

    QTextStream out(stdout);
    QTextStream err(stderr);

    if (argc != 3) {
        err << "Missing input image and output folder argument" << Qt::endl;
        return 1;
    }

    std::string filename(argv[1]);

    QDir outDir(argv[2]);

    if (!outDir.exists()) {
        bool ok = outDir.mkpath(".");

        if (!ok) {
            err << "Impossible to create output directory" << Qt::endl;
            return 1;
        }
    }

    Multidim::Array<aviris4io::data_t, 3> frameData = aviris4io::loadFrame(filename);
    std::vector<int64_t> times = aviris4io::loadFrameTimes(filename);

    if (frameData.empty()) {
        err << "empty frame data" << Qt::endl;
        return 1;
    }

    if (times.empty()) {
        err << "empty time data" << Qt::endl;
        return 1;
    }

    QString bilFileName = "lines.bil";
    QString hdrFileName = bilFileName + ".hdr";
    QString timingsFileName = bilFileName + ".timings";

    QFile bilFile(outDir.filePath(bilFileName));

    if (!bilFile.open(QFile::WriteOnly)) {
        err << "Impossible to open bil file" << Qt::endl;
        return 1;
    }

    aviris4io::data_t* dataPtr = &frameData.at(0,0,0);
    void* voidPtr = static_cast<void*>(dataPtr);

    bilFile.write(static_cast<char*>(voidPtr), frameData.flatLenght()*sizeof (aviris4io::data_t));

    bilFile.close();

    QFile hdrFile(outDir.filePath(hdrFileName));

    if (!hdrFile.open(QFile::WriteOnly | QFile::Text)) {
        err << "Impossible to open header file" << Qt::endl;
        return 1;
    }

    int framerate = 215; //TODO: compute from times

    QTextStream hdrFileStream(&hdrFile);
    hdrFileStream << "ENVI" << "\n";
    hdrFileStream << "interleave = bil" << "\n";
    hdrFileStream << "data type = 12" << "\n";
    hdrFileStream << "lines = " << frameData.shape()[0] << "\n";
    hdrFileStream << "samples = " << frameData.shape()[1] << "\n";
    hdrFileStream << "bands = " << frameData.shape()[2] << "\n";
    hdrFileStream << "ceiling = " << 0xffff << "\n";
    hdrFileStream << "field of view = " << 39.5 << "\n";
    hdrFileStream << "imager serial number = " << "AVIRIS-4" << "\n";
    hdrFileStream << "framerate = " << framerate << "\n";
    hdrFileStream << "wavelength units = nanometers" << "\n";
    hdrFileStream << "wavelength = {";

    int nBands = frameData.shape()[2];

    constexpr float wlmin = 380;
    constexpr float wlmax = 2490;
    constexpr float wldelta = wlmax - wlmin;

    //compute the bands wavelengths
    for (int i = 0; i < nBands; i++) {
        if (i > 0) {
            hdrFileStream << ",";
        }
        hdrFileStream << wlmin + wldelta*i/(nBands-1);
    }

    hdrFileStream << "}\n";

    hdrFileStream.flush();
    hdrFile.close();

    QFile timingsFile(outDir.filePath(timingsFileName));

    if (!timingsFile.open(QFile::WriteOnly | QFile::Text)) {
        err << "Impossible to open timings file" << Qt::endl;
        return 1;
    }

    QTextStream timingsFileStream(&timingsFile);

    for (int64_t time : times) {
        timingsFileStream << time << "\n";
    }

    timingsFileStream.flush();
    timingsFile.close();

    return 0;
}
