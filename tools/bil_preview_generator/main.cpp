#include "libs/io/read_envi_bil.h"

#include <iostream>

#include <QString>
#include <QFileInfo>
#include <QDir>

#include <StereoVision/io/image_io.h>

int main(int argc, char** argv) {

    constexpr int red = 118;
    constexpr int green = 71;
    constexpr int blue = 38;

    constexpr int bLevel = 0;
    constexpr int wLevel = 4095;

    if (argc != 2) {
        std::cerr << "Wrong number of arguments" << std::endl;
    }

    std::string main_folder = argv[1];

    std::vector<std::string> files = get_bil_sequence_files(main_folder);

    for (std::string const& filename : files) {

        std::cout << "Processing image " << filename << std::endl;

        Multidim::Array<float,3> spectral_data = read_envi_bil_to_float(filename);

        if (spectral_data.empty()) {
            std::cout << "\tImage " << filename << " is empty, or could not read image data!" << std::endl;
            continue;
        }

        std::array<int,3> coloredShape = spectral_data.shape();
        coloredShape[BandsAxis] = 3;

        Multidim::Array<float,3> preview(coloredShape);

        for (int i = 0; i < coloredShape[LineAxis]; i++) {

            for (int j = 0; j < coloredShape[SamplesAxis]; j++) {

                std::array<int,3> idxPreview;
                std::array<int,3> idxSpectral;

                idxPreview[LineAxis] = i;
                idxSpectral[LineAxis] = i;

                idxPreview[SamplesAxis] = j;
                idxSpectral[SamplesAxis] = j;

                std::array<int,3> rgbIdxs = {red, green, blue};

                for (int c = 0; c < 3; c++) {

                    idxPreview[BandsAxis] = c;
                    idxSpectral[BandsAxis] = rgbIdxs[c];

                    float spectral_value = spectral_data.valueUnchecked(idxSpectral);

                    spectral_value -= bLevel;
                    spectral_value /= wLevel;
                    spectral_value = (spectral_value < 0) ? 0 : spectral_value;
                    spectral_value = (spectral_value > 1) ? 1 : spectral_value;
                    spectral_value *= 255;

                    preview.atUnchecked(idxPreview) = spectral_value;

                }


            }
        }

        QFileInfo fileInfo(QString::fromStdString(filename));

        QString saveName = fileInfo.dir().filePath(fileInfo.baseName() + "_preview.jpg");

        std::string saveFileName = saveName.toStdString();

        StereoVision::IO::writeImage<uint8_t>(saveFileName, preview);

    }

    return 0;
}
