#include "read_trajectory_data.h"

#include <QFile>
#include <QString>
#include <QRegularExpression>

std::vector<RawTrajectoryLine> read_trajectory_data(std::string const& filename,
                                                    TrajectoryColumnsInfos const& trajectoryDef,
                                                    const std::vector<std::string> &separators,
                                                    const std::string &commentSign) {

    QFile trajFile(filename.c_str());

    if (trajFile.open(QIODevice::ReadOnly)) {
        return std::vector<RawTrajectoryLine>();
    }

    QString lineData;

    std::vector<RawTrajectoryLine> ret;

    int lineCount = 0;

    QString sepPattern = "";

    for (int i = 0; i < separators.size(); i++) {
        sepPattern += QRegularExpression::escape(QString::fromStdString(separators[i]));
        if (i != separators.size()-1) {
            sepPattern += "|";
        }
    }

    QRegularExpression sep(sepPattern);

    //compute the minimum numbers of items in a line
    int minLineElements = trajectoryDef.timeCol;

    if (trajectoryDef.rollCol > minLineElements) {
        minLineElements = trajectoryDef.rollCol;
    }
    if (trajectoryDef.pitchCol > minLineElements) {
        minLineElements = trajectoryDef.pitchCol;
    }
    if (trajectoryDef.yawCol > minLineElements) {
        minLineElements = trajectoryDef.yawCol;
    }

    if (trajectoryDef.latCol > minLineElements) {
        minLineElements = trajectoryDef.latCol;
    }
    if (trajectoryDef.lonCol > minLineElements) {
        minLineElements = trajectoryDef.lonCol;
    }
    if (trajectoryDef.heightCol > minLineElements) {
        minLineElements = trajectoryDef.heightCol;
    }

    minLineElements += 1;

    QString commentPattern = QString::fromStdString(commentSign);

    while (!trajFile.atEnd()) {

        QByteArray line = trajFile.readLine();
        QString str = QString::fromLocal8Bit(line);

        if (str.startsWith(commentPattern)) {
            continue;
        }

        QStringList splitted = str.split(sep, Qt::SkipEmptyParts);

        if (splitted.size() < minLineElements) {
            continue;
        }

        RawTrajectoryLine entry;

        entry.nLine = lineCount;

        bool ok = true;

        entry.timeStamp = splitted[trajectoryDef.timeCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.roll = splitted[trajectoryDef.rollCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.pitch = splitted[trajectoryDef.pitchCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.yaw = splitted[trajectoryDef.yawCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.lon = splitted[trajectoryDef.lonCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.lat = splitted[trajectoryDef.latCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.height = splitted[trajectoryDef.heightCol].toDouble(&ok);

        if (!ok) {
            continue;
        }

        ret.push_back(entry);

        lineCount++;

    }

    return ret;
}
