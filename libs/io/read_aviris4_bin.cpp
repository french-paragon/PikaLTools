#include "read_aviris4_bin.h"

#include <QDir>
#include <QFileInfo>

constexpr int aviris4img_channels = 327; //this does not include the band with time tags
constexpr int aviris4img_resolution = 1280;
constexpr int aviris4img_headerlinelen = aviris4img_resolution*sizeof (aviris4io::data_t);
constexpr int aviris4img_linelen = aviris4img_resolution*(aviris4img_channels+1)*sizeof (aviris4io::data_t);
constexpr int aviris4img_linedatalen = aviris4img_resolution*aviris4img_channels*sizeof (aviris4io::data_t);

constexpr int sysTimeOffset = 0;
constexpr int statusFlagOffset = 80;
constexpr int statusFlagExpected = 0xBABE;
constexpr int utcTowOffset = 103;
constexpr int sysTimePPSOffset = 164;

aviris4io::Aviris4FrameData aviris4io::loadFrame(std::string const& frameFilePath) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        std::vector<double>();
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        std::vector<double>();
    }

    int nLines = fileSize/aviris4img_linelen;

    std::array<int,3> shape{nLines, aviris4img_resolution, aviris4img_channels};
    std::array<int,3> strides{aviris4img_resolution*aviris4img_channels, 1, aviris4img_resolution};

    aviris4io::Aviris4FrameData ret {std::vector<double>(nLines), Multidim::Array<data_t, 3>(shape, strides)};

    for (int i = 0; i < nLines; i++) {

        file.seek(i*aviris4img_linelen);
        QByteArray headerData = file.read(aviris4img_headerlinelen); //first 4 bytes are the time

        if (headerData.size() < aviris4img_headerlinelen) {
            std::fill_n(&ret.frame.at(i,0,0), aviris4img_linedatalen, 0);
            ret.gpsTime[i] = -1;
            continue; //skip faulty line
        }

        uint32_t lineInternalTime;
        std::memcpy(&lineInternalTime, &(headerData.data()[sysTimeOffset]), 4);
        lineInternalTime = lineInternalTime ^ (1 << 15);
        lineInternalTime = lineInternalTime ^ (1 << 31);

        uint32_t flag = 0;
        std::memcpy(&flag, &(headerData.data()[statusFlagOffset]), 4);

        if ((flag ^ 0xBABE) != 0) {
            std::cout << "Formatting error in the data at line " << i << std::endl;
        }

        uint32_t gpsValidityTime;
        std::memcpy(&gpsValidityTime, &(headerData.data()[utcTowOffset]), 4);

        uint32_t ppsInternalTime;
        std::memcpy(&ppsInternalTime, &(headerData.data()[sysTimePPSOffset]), 4);

        int32_t timeDelta = lineInternalTime - ppsInternalTime;

        ret.gpsTime[i] = static_cast<double>(gpsValidityTime + timeDelta)/10000;

        data_t* dataptr = &ret.frame.at(i,0,0);
        void* voidptr = static_cast<void*>(dataptr);
        char* charptr = static_cast<char*>(voidptr);

        qint64 written = file.read(charptr, aviris4img_linedatalen);

        if (written != aviris4img_linedatalen) {
            std::fill_n(&ret.frame.at(i,0,0), aviris4img_linedatalen, 0);
            ret.gpsTime[i] = -1;
            continue; //skip faulty line
        }

    }

    return ret;

}

std::vector<double> aviris4io::loadFrameTimes(std::string const& frameFilePath) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        std::vector<double>();
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        std::vector<double>();
    }

    int nLines = fileSize/aviris4img_linelen;

    std::vector<double> ret(nLines);

    for (int i = 0; i < nLines; i++) {

        file.seek(i*aviris4img_linelen);
        QByteArray headerData = file.read(aviris4img_headerlinelen); //first 4 bytes are the time

        if (headerData.size() < aviris4img_headerlinelen) {
            continue;
        }

        uint32_t lineInternalTime;
        std::memcpy(&lineInternalTime, &(headerData.data()[sysTimeOffset]), 4);
        lineInternalTime = lineInternalTime ^ (1 << 15);
        lineInternalTime = lineInternalTime ^ (1 << 31);

        uint32_t flag = 0;
        std::memcpy(&flag, &(headerData.data()[statusFlagOffset]), 4);

        if ((flag ^ 0xBABE) != 0) {
            std::cout << "Formatting error in the data" << std::endl;
        }

        uint32_t gpsValidityTime;
        std::memcpy(&gpsValidityTime, &(headerData.data()[utcTowOffset]), 4);

        uint32_t ppsInternalTime;
        std::memcpy(&ppsInternalTime, &(headerData.data()[sysTimePPSOffset]), 4);

        int32_t timeDelta = lineInternalTime - ppsInternalTime;

        ret[i] = static_cast<double>(gpsValidityTime + timeDelta)/10000;

    }

    return ret;


}

std::vector<double> aviris4io::loadSequenceTimes(std::string const& sequenceFolderPath, std::string const& filter) {

    std::vector<std::string> files = getFilesInSequence(sequenceFolderPath, filter);
    std::vector<double> times;

    for (std::string const& path : files) {
        std::vector<double> ftimes = loadFrameTimes(path);

        times.insert(times.end(), ftimes.begin(), ftimes.end());
    }

    return times;

}

std::vector<std::string> aviris4io::getFilesInSequence(std::string const& sequenceFolderPath, const std::string &filter) {


    QDir directory(QString::fromStdString(sequenceFolderPath));

    if (!directory.exists()) {
        return std::vector<std::string>();
    }

    QStringList files = directory.entryList(QStringList{QString::fromStdString(filter)}, QDir::Files);

    struct fileWithTime {
        int fileId;
        uint32_t time;
    };

    QVector<fileWithTime> fileList;
    fileList.reserve(files.size());

    for (int i = 0; i < files.size(); i++) {

        QFile file(files[i]);

        if (!file.open(QIODevice::ReadOnly)) {
            continue;
        }

        QByteArray timeData = file.read(4); //first 4 bytes are the time

        if (timeData.size() < 4) {
            continue;
        }

        uint32_t value1 = uint32_t(timeData[0]) | (uint32_t(timeData[1]) << 8);
        value1 = value1 ^ (1 << 15);
        uint32_t value2 = uint32_t(timeData[2]) | (uint32_t(timeData[3]) << 8);
        value2 = value2 ^ (1 << 15);
        uint32_t count = (value2 << 16) | value1;

        fileList.push_back({i, count});

    }

    std::sort(fileList.begin(), fileList.end(), [] (fileWithTime const& f1, fileWithTime const& f2) {
       return  f1.time < f2.time;
    });

    std::vector<std::string> ret(fileList.size());

    for (int i = 0; i < fileList.size(); i++) {
        ret[i] = files[fileList[i].fileId].toStdString();
    }

    return ret;
}
