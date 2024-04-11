#include "libs/io/read_envi_bil.h"

#include <QCommandLineParser>
#include <QDir>

#include <iostream>

int main(int argc, char** argv) {

    QCommandLineParser parser;

    QString datasetName = "datasetFolder";
    parser.addPositionalArgument(datasetName, "Folder where the dataset is stored");

    QStringList args;
    for (int i = 0; i < argc; i++) {
        args << argv[i];
    }

    parser.process(args);

    QString datasetPath = parser.positionalArguments()[0];

    QDir datasetDir(datasetPath);

    if (!datasetDir.exists()) {
        std::cerr << "Non existing dir " << datasetPath.toStdString() << std::endl;
        return 1;
    }

    std::vector<std::string> bilFilesLists = get_bil_sequence_files(datasetDir.absolutePath().toStdString());

    int nFiles = bilFilesLists.size();

    int nLines = 0;

    bool missing_headers = false;
    std::string missingHeaderFile;

    for (std::string const& bilFilePath : bilFilesLists) {
        auto headerOpt = readHeaderData(bilFilePath);

        if (!headerOpt.has_value()) {
            missing_headers = true;
            break;
        }

        std::map<std::string,std::string>& header = headerOpt.value();

        if (header.count("lines") <= 0) {
            missing_headers = true;
            missingHeaderFile = bilFilePath;
            break;
        }

        int nLinesBilFile = stoi(header["lines"]);
        nLines += nLinesBilFile;

    }

    if (missing_headers) {
        std::cerr << "Missing header for image " << missingHeaderFile << std::endl;
        return 1;
    }

    std::cout << "Dataset stats for: " << datasetDir.absolutePath().toStdString() << std::endl;
    std::cout << "\t" << "Number of files: " << nFiles << std::endl;
    std::cout << "\t" << "Number of lines: " << nLines << std::endl;

    return 0;

}
