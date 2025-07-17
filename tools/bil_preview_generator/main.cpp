#include "libs/io/read_envi_bil.h"

#include <iostream>

#include <QString>
#include <QFileInfo>
#include <QDir>

#include <StereoVision/io/image_io.h>

#include <tclap/CmdLine.h>

int main(int argc, char** argv) {

    int red = 118;
    int green = 71;
    int blue = 38;

    int bLevel = 35000;
    int wLevel = 50000;

    bool normalize = false;
    bool fullLine = false;

    std::string main_folder;

    try {

        TCLAP::CmdLine cmd("Export a preview for a sequence of bil directory in a folder", '=', "0.0");

        TCLAP::UnlabeledValueArg<std::string> bilFolderPathArg("bilFolderPath", "Path where the bil sequence will be stored", true, "", "local path to folder");

        cmd.add(bilFolderPathArg);

        TCLAP::ValueArg<int> redArg("r","red", "Channel for red data", false, 118, "int");
        TCLAP::ValueArg<int> greenArg("g","green", "Channel for red data", false, 71, "int");
        TCLAP::ValueArg<int> blueArg("b","blue", "Channel for red data", false, 38, "int");

        TCLAP::ValueArg<int> blackArg("k","black", "black level", false, 35000, "int");
        TCLAP::ValueArg<int> whiteArg("w","white", "white level", false, 50000, "int");

        TCLAP::SwitchArg normalizeArg("n","normalize", "normalize color channels", false);
        TCLAP::SwitchArg fullLineArg("l","fullline", "export a preview of the full line", false);

        cmd.add(redArg);
        cmd.add(greenArg);
        cmd.add(blueArg);

        cmd.add(blackArg);
        cmd.add(whiteArg);

        cmd.add(normalizeArg);
        cmd.add(fullLineArg);

        cmd.parse(argc, argv);

        main_folder = bilFolderPathArg.getValue();

        red = redArg.getValue();
        green = greenArg.getValue();
        blue = blueArg.getValue();

        bLevel = blackArg.getValue();
        wLevel = whiteArg.getValue();

        normalize = normalizeArg.getValue();
        fullLine = fullLineArg.getValue();

    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        return 1;
    }

    std::vector<std::string> files = get_bil_sequence_files(main_folder);

    Multidim::Array<float,3> fullLinePreview;

    if (fullLine) {

        std::vector<int> channels = {red, green, blue};
        std::optional<int> startLine = std::nullopt;
        std::optional<int> endLine = std::nullopt;

        fullLinePreview = read_bil_sequence_to_float(files, startLine, endLine, channels);

        auto fullShape = fullLinePreview.shape();

        std::vector<float> red_values;
        std::vector<float> green_values;
        std::vector<float> blue_values;

        std::array<std::vector<float>*, 3> valuesArray = {&red_values, &green_values, &blue_values};

        if (normalize) {
            red_values.reserve(fullLinePreview.flatLenght());
            green_values.reserve(fullLinePreview.flatLenght());
            blue_values.reserve(fullLinePreview.flatLenght());
        }


        for (int i = 0; i < fullShape[LineAxis]; i++) {

            for (int j = 0; j < fullShape[SamplesAxis]; j++) {

                for (int c = 0; c < 3; c++) {

                    std::array<int,3> idxPreview;

                    idxPreview[LineAxis] = i;
                    idxPreview[SamplesAxis] = j;
                    idxPreview[BandsAxis] = c;

                    float spectral_value = fullLinePreview.valueUnchecked(idxPreview);

                    if (normalize) {
                        valuesArray[c]->push_back(spectral_value);
                    } else {
                        spectral_value -= bLevel;
                        spectral_value /= wLevel;
                        spectral_value = (spectral_value < 0) ? 0 : spectral_value;
                        spectral_value = (spectral_value > 1) ? 1 : spectral_value;
                        spectral_value *= 255;

                        fullLinePreview.atUnchecked(idxPreview) = spectral_value;
                    }
                }

            }
        }

        if (normalize) {

            int blackLevelQuant = 0.05*red_values.size();
            int whiteLevelQuant = 0.95*red_values.size();

            for (int c = 0; c < 3; c++) {
                std::nth_element(valuesArray[c]->begin(),
                                 valuesArray[c]->begin()+blackLevelQuant,
                                 valuesArray[c]->end());

                float bLevel = valuesArray[c]->at(blackLevelQuant);

                std::nth_element(valuesArray[c]->begin()+blackLevelQuant,
                                 valuesArray[c]->begin()+whiteLevelQuant,
                                 valuesArray[c]->end());

                float wLevel = valuesArray[c]->at(whiteLevelQuant);


                for (int i = 0; i < fullShape[LineAxis]; i++) {

                    for (int j = 0; j < fullShape[SamplesAxis]; j++) {

                        std::array<int,3> idxPreview;
                        idxPreview[LineAxis] = i;
                        idxPreview[SamplesAxis] = j;
                        idxPreview[BandsAxis] = c;

                        float spectral_value = fullLinePreview.valueUnchecked(idxPreview);

                        spectral_value -= bLevel;
                        spectral_value /= (wLevel - bLevel);
                        spectral_value = (spectral_value < 0) ? 0 : spectral_value;
                        spectral_value = (spectral_value > 1) ? 1 : spectral_value;
                        spectral_value *= 255;

                        fullLinePreview.atUnchecked(idxPreview) = spectral_value;

                    }
                }
            }

        }

        QDir dir(QString::fromStdString(main_folder));

        QString saveName = dir.filePath("preview.jpg");

        std::string saveFileName = saveName.toStdString();

        StereoVision::IO::writeImage<uint8_t>(saveFileName, fullLinePreview);

        return 0;
    }

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

        std::vector<float> red_values;
        std::vector<float> green_values;
        std::vector<float> blue_values;

        std::array<std::vector<float>*, 3> valuesArray = {&red_values, &green_values, &blue_values};

        if (normalize) {
            red_values.reserve(preview.flatLenght());
            green_values.reserve(preview.flatLenght());
            blue_values.reserve(preview.flatLenght());
        }

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

                    if (!normalize) {
                        spectral_value -= bLevel;
                        spectral_value /= (wLevel - bLevel);
                        spectral_value = (spectral_value < 0) ? 0 : spectral_value;
                        spectral_value = (spectral_value > 1) ? 1 : spectral_value;
                        spectral_value *= 255;
                    }

                    preview.atUnchecked(idxPreview) = spectral_value;

                    if (normalize) {
                        valuesArray[c]->push_back(spectral_value);
                    }

                }


            }
        }

        if (normalize) {

            int blackLevelQuant = 0.05*red_values.size();
            int whiteLevelQuant = 0.95*red_values.size();

            std::array<float,3> blackLevels;
            std::array<float,3> whiteLevels;

            for (int c = 0; c < 3; c++) {
                std::nth_element(valuesArray[c]->begin(),
                                 valuesArray[c]->begin()+blackLevelQuant,
                                 valuesArray[c]->end());

                blackLevels[c] = valuesArray[c]->at(blackLevelQuant);

                std::nth_element(valuesArray[c]->begin()+blackLevelQuant,
                                 valuesArray[c]->begin()+whiteLevelQuant,
                                 valuesArray[c]->end());

                whiteLevels[c] = valuesArray[c]->at(whiteLevelQuant);


                for (int i = 0; i < coloredShape[LineAxis]; i++) {

                    for (int j = 0; j < coloredShape[SamplesAxis]; j++) {

                        std::array<int,3> idxPreview;
                        idxPreview[LineAxis] = i;
                        idxPreview[SamplesAxis] = j;
                        idxPreview[BandsAxis] = c;

                        float spectral_value = preview.valueUnchecked(idxPreview);

                        spectral_value -= blackLevels[c];
                        spectral_value /= (whiteLevels[c] - blackLevels[c]);
                        spectral_value = (spectral_value < 0) ? 0 : spectral_value;
                        spectral_value = (spectral_value > 1) ? 1 : spectral_value;
                        spectral_value *= 255;

                        preview.atUnchecked(idxPreview) = spectral_value;

                    }
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
