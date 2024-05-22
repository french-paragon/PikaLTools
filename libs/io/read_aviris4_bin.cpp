#include "read_aviris4_bin.h"

#include <QDir>
#include <QFileInfo>

constexpr int64_t aviris4img_channels = 327; //this does not include the band with time tags
constexpr int64_t aviris4img_resolution = 1280;
constexpr int64_t aviris4img_headerlinelen = aviris4img_resolution*sizeof (aviris4io::data_t);
constexpr int64_t aviris4img_linelen = aviris4img_resolution*(aviris4img_channels+1)*sizeof (aviris4io::data_t);
constexpr int64_t aviris4img_linedatalen = aviris4img_resolution*aviris4img_channels*sizeof (aviris4io::data_t);

constexpr int64_t sysTimeOffset = 0;
constexpr int64_t statusFlagOffset = 80;
constexpr int64_t statusFlagExpected = 0xBABE;
constexpr int64_t utcTowOffset = 116;
constexpr int64_t sysTimePPSOffset = 164;

int aviris4io::getAviris4FramesNLines(std::string const& frameFilePath) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        return -1;
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        return -1;
    }

    return fileSize/aviris4img_linelen;
}

Multidim::Array<aviris4io::data_t, 3> aviris4io::loadFrame(std::string const& frameFilePath) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        return Multidim::Array<aviris4io::data_t, 3>();
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        return Multidim::Array<aviris4io::data_t, 3>();
    }

    int nLines = fileSize/aviris4img_linelen;

    std::array<int,3> shape{nLines, aviris4img_resolution, aviris4img_channels};
    std::array<int,3> strides{aviris4img_resolution*aviris4img_channels, 1, aviris4img_resolution};

    Multidim::Array<data_t, 3> ret(shape, strides);

    for (int i = 0; i < nLines; i++) {

        file.seek(i*aviris4img_linelen + aviris4img_headerlinelen);

        data_t* dataptr = &ret.at(i,0,0);
        void* voidptr = static_cast<void*>(dataptr);
        char* charptr = static_cast<char*>(voidptr);

        qint64 written = file.read(charptr, aviris4img_linedatalen);

        if (written != aviris4img_linedatalen) {
            std::fill_n(&ret.at(i,0,0), aviris4img_linedatalen, 0);
            continue; //skip faulty line
        }

    }

    return ret;

}

Multidim::Array<aviris4io::data_t, 3> aviris4io::loadFrameSlice(std::string const& frameFilePath, int startLine, int nLines) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        return Multidim::Array<aviris4io::data_t, 3>();
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        return Multidim::Array<aviris4io::data_t, 3>();
    }

    int fileNLines = fileSize/aviris4img_linelen;

    int nLoadedLines = std::min(fileNLines-startLine, nLines);

    std::array<int,3> shape{nLoadedLines, aviris4img_resolution, aviris4img_channels};
    std::array<int,3> strides{aviris4img_resolution*aviris4img_channels, 1, aviris4img_resolution};

    Multidim::Array<data_t, 3> ret(shape, strides);

    for (int i = 0; i < nLoadedLines; i++) {

        file.seek((startLine+i)*aviris4img_linelen + aviris4img_headerlinelen);

        data_t* dataptr = &ret.at(i,0,0);
        void* voidptr = static_cast<void*>(dataptr);
        char* charptr = static_cast<char*>(voidptr);

        qint64 written = file.read(charptr, aviris4img_linedatalen);

        if (written != aviris4img_linedatalen) {
            std::fill_n(&ret.at(i,0,0), aviris4img_linedatalen, 0);
            continue; //skip faulty line
        }

    }

    return ret;


}

std::vector<int64_t> aviris4io::loadFrameTimes(std::string const& frameFilePath) {

    QFile file(QString::fromStdString(frameFilePath));

    if (!file.open(QIODevice::ReadOnly)) {
        return std::vector<int64_t>();
    }

    qint64 fileSize = file.size();

    if (fileSize % aviris4img_linelen != 0) { //unexpected size
        return std::vector<int64_t>();
    }

    int nLines = fileSize/aviris4img_linelen;

    struct lineTimingInfos {
        int64_t internalTime;
        int64_t gpsTimeLastPPS;
        int64_t internalTimeLastPPS;
        bool isBabe;
    };

    std::vector<lineTimingInfos> infos(nLines);
    std::vector<int64_t> ret(nLines);

    for (int i = 0; i < nLines; i++) {

        file.seek(i*aviris4img_linelen);
        QByteArray headerData = file.read(aviris4img_headerlinelen); //first 4 bytes are the time

        if (headerData.size() < aviris4img_headerlinelen) {
            continue;
        }

        uint8_t bytesBuffer[4];

        std::memcpy(bytesBuffer, &(headerData.data()[sysTimeOffset]), 4);
        uint32_t lineInternalTime = bytesBuffer[0] | bytesBuffer[1] << 8 | bytesBuffer[2] << 16 | bytesBuffer[3] << 24;

        std::memcpy(bytesBuffer, &(headerData.data()[statusFlagOffset]), 4);

        uint32_t flag = bytesBuffer[0] | bytesBuffer[1] << 8;
        //BABE if PPS changes

        bool isBabe = false;

        if ((flag ^ 0xBABE) == 0) {
            isBabe = true;
        }

        std::memcpy(bytesBuffer, &(headerData.data()[utcTowOffset]), 4);

        //Big endian
        uint32_t gpsValidityTime = bytesBuffer[3] | bytesBuffer[2] << 8 | bytesBuffer[1] << 16 | bytesBuffer[0] << 24;

        std::memcpy(bytesBuffer, &(headerData.data()[sysTimePPSOffset]), 4);
        uint32_t ppsInternalTime = bytesBuffer[2] | bytesBuffer[3] << 8 | bytesBuffer[0] << 16 | bytesBuffer[1] << 24;

        infos[i] = {lineInternalTime, gpsValidityTime, ppsInternalTime, isBabe};

    }

    //fill in missing values
    std::vector<size_t> babeIdxs;
    for (int i = 0; i < infos.size(); i++) {
        if (infos[i].isBabe) {
            babeIdxs.push_back(i);
        }
    }

    if (babeIdxs.empty()) {
        return std::vector<int64_t>();
    }

    int previousBabeIdx = babeIdxs[0];
    int nextBabeIdx = babeIdxs[0];
    int currentBabeIdxPos = 0;

    for (int i = 0; i < infos.size(); i++) {
        int delta_prev = std::abs(i - previousBabeIdx);
        int delta_next = std::abs(i - nextBabeIdx);

        if (delta_prev < delta_next) {
            infos[i].gpsTimeLastPPS = infos[previousBabeIdx].gpsTimeLastPPS;
            infos[i].internalTimeLastPPS = infos[previousBabeIdx].internalTimeLastPPS;
        } else {
            infos[i].gpsTimeLastPPS = infos[nextBabeIdx].gpsTimeLastPPS;
            infos[i].internalTimeLastPPS = infos[nextBabeIdx].internalTimeLastPPS;
        }

        if (i == nextBabeIdx) {
            previousBabeIdx = nextBabeIdx;
            currentBabeIdxPos++;
            if (currentBabeIdxPos >= babeIdxs.size()) {
                currentBabeIdxPos = babeIdxs.size()-1;
            }
            nextBabeIdx = babeIdxs[currentBabeIdxPos];
        }
    }

    //at that point infos has been filled such that a gps time reference and internal time for the corresponding pps is set
    for (int i = 0; i < infos.size(); i++) {
        int64_t delta_t = infos[i].internalTime - infos[i].internalTimeLastPPS;

        ret[i] = infos[i].gpsTimeLastPPS*10 + delta_t;
    }

    return ret;

}

std::vector<int64_t> aviris4io::loadSequenceTimes(std::string const& sequenceFolderPath, std::string const& filter) {

    std::vector<std::string> files = getFilesInSequence(sequenceFolderPath, filter);
    std::vector<int64_t> times;

    for (std::string const& path : files) {
        std::vector<int64_t> ftimes = loadFrameTimes(path);

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
