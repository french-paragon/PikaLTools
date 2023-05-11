#include <QTextStream>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>

#include <algorithm>

int main(int argc, char** argv) {

    QTextStream out(stdout);

    QVector<QString> args;
    QVector<QString> opts;

    for (int i = 1; i < argc; i++) {

        QString arg(argv[i]);

        if (arg.startsWith("-")) {
            opts.push_back(arg);
        } else {
            args.push_back(arg);
        }
    }

    if (args.size() != 1) {
        out << "Incorrect number of arguments provided!" << Qt::endl;
        return 1;
    }

    bool addHeader = opts.contains("--addHeader");

    QDir baseDir(args[0]);
    QDirIterator baseDirIterator(args[0]);

    QVector<QString> manual_folders;

    while(baseDirIterator.hasNext()) {
        QString entry = baseDirIterator.next();

        QFileInfo entryInfo(entry);

        if (entryInfo.baseName().startsWith("manual")) {
            manual_folders.push_back(entry);
        }
    }

    if (manual_folders.isEmpty()) {
        out << "No input data!" << Qt::endl;
        return 1;
    }

    auto strComparer = [] (QString path1, QString path2) {

        QStringList splitted1 = path1.split("-");
        QStringList splitted2 = path2.split("-");

        bool ok1;
        bool ok2;

        int conv1 = splitted1.last().toInt(&ok1);
        int conv2 = splitted2.last().toInt(&ok2);

        if (!ok1 or !ok2) {
            return false;
        }

        return conv1 < conv2;
    };

    std::sort(manual_folders.begin(), manual_folders.end(), strComparer);

    QVector<QString> lcf_files;

    for (QString entry : manual_folders) {

        QDir subDir(entry);
        QDirIterator subDirIterator(entry);

        QString file;

        while(subDirIterator.hasNext()) {
            QString entry = subDirIterator.next();

            QFileInfo entryInfo(entry);

            if (entryInfo.baseName().startsWith("manual") and entry.endsWith(".lcf")) {
                lcf_files.push_back(entry);
            }
        }

    }

    if (lcf_files.isEmpty()) {
        out << "No input data!" << Qt::endl;
        return 1;
    }

    QFile outFile(baseDir.absoluteFilePath("merged_lcf.csv"));
    outFile.open(QIODevice::WriteOnly | QIODevice::Text );
    QTextStream outFileStream(&outFile);


    if (addHeader) {
        outFileStream << "timestamp\t" << "roll\t" << "pitch\t" << "yaw\t" << "longitude\t" << "lattitude\t" << "altitude";

        for (int i = 1; i <= 4; i++) {
            outFileStream << "\tStatus" << i;
        }

        outFileStream << Qt::endl;
    }

    for (QString lcf_file_path: lcf_files) {

        QFile lcf_file(lcf_file_path);

        lcf_file.open(QIODevice::ReadOnly | QIODevice::Text );

        if (!lcf_file.isOpen()) {
            out << "Could no open file: " << lcf_file_path << Qt::endl;
        }

        QString line;

        QTextStream in(&lcf_file);
        while (!in.atEnd()) {
           QString line = in.readLine();

           if (!line.isEmpty()) {
               outFileStream << line << Qt::endl;
           }
        }

        lcf_file.close();
    }

    outFile.close();

    return 0;

}
