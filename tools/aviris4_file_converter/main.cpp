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

#include <tclap/CmdLine.h>

int main(int argc, char** argv) {

    QTextStream out(stdout);
    QTextStream err(stderr);

    std::string filename;
    QDir outDir;

    int lineNumber;
    int maxBilLen;

    try {
        TCLAP::CmdLine cmd("Project a bin file from aviris4 to a sequence of bil files", '=', "0.0");

        TCLAP::UnlabeledValueArg<std::string> binPathArg("binPath", "path to the bin file to export", true, "", "path to aviris4 bin data");
        TCLAP::UnlabeledValueArg<std::string> bilFolderPathArg("bilFolderPath", "Path where the bil sequence will be stored", true, "", "local path to folder");

        cmd.add(binPathArg);
        cmd.add(bilFolderPathArg);

        TCLAP::ValueArg<int> lineArg("l","line", "number of the line in the flight", false, 1, "int");
        TCLAP::ValueArg<int> heightArg("b","blockHeight", "Maximal size of a bil file (<= 0 means no limit)", false, 2000, "int");

        cmd.add(lineArg);
        cmd.add(heightArg);

        cmd.parse(argc, argv);

        filename = binPathArg.getValue();

        outDir = QDir(QString::fromStdString(bilFolderPathArg.getValue()));

        lineNumber = lineArg.getValue();
        maxBilLen = heightArg.getValue();

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        return 1;
    }

    if (!outDir.exists()) {
        bool ok = outDir.mkpath(".");

        if (!ok) {
            err << "Impossible to create output directory" << Qt::endl;
            return 1;
        }
    }

    std::vector<int64_t> times = aviris4io::loadFrameTimes(filename);

    if (times.empty()) {
        err << "empty time data" << Qt::endl;
        return 1;
    }

    int nLines = aviris4io::getAviris4FramesNLines(filename);

    if (maxBilLen <= 0) {
        maxBilLen = nLines;
    }

    int nBlocks = nLines/maxBilLen;

    if (nLines > maxBilLen*nBlocks) {
        nBlocks += 1;
    }

    for (int i = 0; i < nBlocks; i++) {

        out << "Processing block " << (i+1) << "/" << nBlocks << Qt::endl;

        int startLine = i*maxBilLen;
        int blockNLines = std::min(maxBilLen, nLines - startLine);

        QString frameFolderName = QString("%1_%2").arg(lineNumber, 4, 10, QChar('0')).arg(i+1, 4, 10, QChar('0'));

        QDir blockDir(outDir.filePath(frameFolderName));

        bool ok = blockDir.mkpath(".");

        if (!ok) {
            err << "Impossible to create folder for block " << frameFolderName << Qt::endl;
            return 1;
        }

        QString bilFileName = "frame.bil";
        QString hdrFileName = bilFileName + ".hdr";
        QString timingsFileName = bilFileName + ".timing";

        QFile bilFile(blockDir.filePath(bilFileName));

        if (!bilFile.open(QFile::WriteOnly)) {
            err << "Impossible to open bil file for block " << frameFolderName << Qt::endl;
            return 1;
        }

        int channels = 0;
        int samples = 0;
        int linesWritten = 0;

        //read the file line by line
        for (int i = 0; i < blockNLines; i++) {

            Multidim::Array<aviris4io::data_t, 3> frameData = aviris4io::loadFrameSlice(filename, startLine+i, 1);

            if (frameData.empty()) {
                continue;
            }

            channels = frameData.shape()[2];
            samples = frameData.shape()[1];

            aviris4io::data_t* dataPtr = &frameData.at(0,0,0);
            void* voidPtr = static_cast<void*>(dataPtr);

            bilFile.write(static_cast<char*>(voidPtr), frameData.flatLenght()*sizeof (aviris4io::data_t));

            linesWritten++;
        }

        bilFile.close();

        QFile hdrFile(blockDir.filePath(hdrFileName));

        if (!hdrFile.open(QFile::WriteOnly | QFile::Text)) {
            err << "Impossible to open header file for block " << frameFolderName << Qt::endl;
            return 1;
        }

        int framerate = 215; //TODO: compute from times

        QTextStream hdrFileStream(&hdrFile);
        hdrFileStream << "ENVI" << "\n";
        hdrFileStream << "interleave = bil" << "\n";
        hdrFileStream << "data type = 12" << "\n";
        hdrFileStream << "lines = " << linesWritten << "\n";
        hdrFileStream << "samples = " << samples << "\n";
        hdrFileStream << "bands = " << channels << "\n";
        hdrFileStream << "ceiling = " << 0xffff << "\n";
        hdrFileStream << "field of view = " << 39.5 << "\n";
        hdrFileStream << "imager serial number = " << "AVIRIS-4" << "\n";
        hdrFileStream << "framerate = " << framerate << "\n";
        hdrFileStream << "wavelength units = nanometers" << "\n";
        hdrFileStream << "wavelength = {";

        constexpr float wlmin = 380;
        constexpr float wlmax = 2490;
        constexpr float wldelta = wlmax - wlmin;

        //compute the bands wavelengths
        for (int i = 0; i < channels; i++) {
            if (i > 0) {
                hdrFileStream << ",";
            }
            hdrFileStream << wlmin + wldelta*i/(channels-1);
        }

        hdrFileStream << "}\n";

        hdrFileStream.flush();
        hdrFile.close();

        QFile timingsFile(blockDir.filePath(timingsFileName));

        if (!timingsFile.open(QFile::WriteOnly | QFile::Text)) {
            err << "Impossible to open timings file for block " << frameFolderName << Qt::endl;
            return 1;
        }

        QTextStream timingsFileStream(&timingsFile);

        for (int i = startLine; i < startLine + blockNLines; i++) {
            timingsFileStream << times[i] << "\n";
        }

        timingsFileStream.flush();
        timingsFile.close();

    }

    return 0;
}
