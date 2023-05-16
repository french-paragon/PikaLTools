#include "bilacquisitiondata.h"

#include "io/read_envi_bil.h"

#include "geo/coordinate_conversions.h"

#include <QJsonArray>

namespace PikaLTools {

BilSequenceAcquisitionData::BilSequenceAcquisitionData(StereoVisionApp::Project *parent) :
    StereoVisionApp::DataBlock(parent),
    _lcfDataLoaded(false)
{
    connect(this, &BilSequenceAcquisitionData::bilSequenceChanged, this, [this] () {
        _lcfDataLoaded = false;
    });
}
BilSequenceAcquisitionData::BilAcquisitionData::BilAcquisitionData(QString path) :
    _bil_file_path(path)
{

}

void BilSequenceAcquisitionData::BilAcquisitionData::setBilFilePath(QString filePath) {

    if (filePath != _bil_file_path) {
        _bil_file_path = filePath;
    }

}

QString BilSequenceAcquisitionData::BilAcquisitionData::bilFilePath() const {
    return _bil_file_path;
}
QString BilSequenceAcquisitionData::BilAcquisitionData::headerFilePath() const {
    return _bil_file_path + ".hdr";
}
QString BilSequenceAcquisitionData::BilAcquisitionData::lcfFilePath() const {
    return _bil_file_path.mid(0, _bil_file_path.size()-4)+".lcf";
}

QMap<QString, QString> BilSequenceAcquisitionData::BilAcquisitionData::headerData() const {
    std::optional<std::map<std::string, std::string>> optData = readHeaderData(_bil_file_path.toStdString());

    if (!optData.has_value()) {
        return QMap<QString, QString>();
    }

    std::map<std::string, std::string> val = optData.value();
    QMap<QString, QString> ret;

    for (auto const& [key, val] : val) {
        ret.insert(QString::fromStdString(key), QString::fromStdString(val));
    }

    return ret;
}

void BilSequenceAcquisitionData::setBilSequence(QList<QString> const& bilFiles) {

    bool same = false;

    if (_bilSequence.size() == bilFiles.size()) {

        same = true;

        for (int i = 0; i < _bilSequence.size(); i++) {
            if (_bilSequence[i].bilFilePath() != bilFiles[i]) {
                same = false;
                break;
            }
        }
    }

    if (same) {
        return;
    }

    _bilSequence.clear();
    _bilSequence.reserve(bilFiles.size());

    for (int i = 0; i < bilFiles.size(); i++) {
        _bilSequence.push_back(BilAcquisitionData(bilFiles[i]));
    }

}

QList<QString> BilSequenceAcquisitionData::getBilFiles() const {

    QList<QString> ret;
    ret.reserve(_bilSequence.size());

    for (int i = 0; i < _bilSequence.size(); i++) {
        ret.push_back(_bilSequence[i].bilFilePath());
    }
    return ret;
}

QList<BilSequenceAcquisitionData::BilAcquisitionData> BilSequenceAcquisitionData::getBilInfos() const {
    return _bilSequence;
}


bool BilSequenceAcquisitionData::loadLcfData(const QList<QString> &files) const {

    _localTrajectory.clear();

    double meanLat = 0;
    double meanLon = 0;

    double minAlt = -1;

    for (QString const& file : files) {
        std::vector<EnviBilLcfLine> lines = read_envi_bil_lcf_data(file.toStdString());

        _localTrajectory.reserve(_localTrajectory.size() + lines.size());

        for (EnviBilLcfLine const& line : lines) {

            double lat= line.lat;
            double lon = line.lon;
            double alt = line.height;

            meanLat += lat;
            meanLon += lon;

            if (alt < minAlt or minAlt < 0) {
                minAlt = alt;
            }

            CartesianCoord<double> pos = convertLatLonToECEF<double>(lat, lon, alt, WGS84_Ellipsoid);

            Eigen::Vector3f t(pos.x, pos.y, pos.z);

            Eigen::Matrix3f rot = StereoVision::Geometry::eulerDegXYZToRotation<float>(line.roll, line.pitch, line.yaw);
            Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(rot);

            _localTrajectory.push_back(StereoVision::Geometry::ShapePreservingTransform<float>(r,t,1));
        }
    }

    meanLat /= _localTrajectory.size();
    meanLon /= _localTrajectory.size();

    StereoVision::Geometry::AffineTransform<double> ecef2localAlt0 = getLocalFrameAtPos<double>(meanLat, meanLon, WGS84_Ellipsoid);
    StereoVision::Geometry::AffineTransform<float> localAlt0ToLocal(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0,0,-minAlt));

    _ecef2local = localAlt0ToLocal*StereoVision::Geometry::AffineTransform<float>(ecef2localAlt0.R.cast<float>(), ecef2localAlt0.t.cast<float>());

    for (StereoVision::Geometry::ShapePreservingTransform<float> & pos : _localTrajectory) {

        Eigen::Matrix3f R = _ecef2local.R*StereoVision::Geometry::rodriguezFormula(pos.r);
        pos.r = StereoVision::Geometry::inverseRodriguezFormula(R);
        pos.t = _ecef2local*pos.t;
    }

    _lcfDataLoaded = true;
    return true;
}

void BilSequenceAcquisitionData::clearOptimized() {
    return;
}
bool BilSequenceAcquisitionData::hasOptimizedParameters() const {
    return false;
}

QJsonObject BilSequenceAcquisitionData::encodeJson() const {

    QJsonObject obj;

    QJsonArray bilFiles;

    for (int i = 0; i < _bilSequence.size(); i++) {
        bilFiles.push_back(_bilSequence[i].bilFilePath());
    }
    obj.insert("bilFiles", bilFiles);

    return obj;
}

void BilSequenceAcquisitionData::configureFromJson(QJsonObject const& data) {

    QList<QString> bilFiles;

    if (data.contains("bilFiles")) {
        QJsonValue val = data.value("bilFiles");

        if (val.isArray()) {
            QJsonArray arr = val.toArray();
            bilFiles.reserve(arr.size());

            for (QJsonValue val : arr) {
                bilFiles.push_back(val.toString());
            }
        }
    }

    setBilSequence(bilFiles);
}

void BilSequenceAcquisitionData::extendDataModel() {
    //TODO: add more options to extend the data model
}


BilSequenceAcquisitionDataFactory::BilSequenceAcquisitionDataFactory(QObject* parent) :
    StereoVisionApp::DataBlockFactory(parent)
{

}

QString BilSequenceAcquisitionDataFactory::TypeDescrName() const {
    return tr("Hyperspectral bil sequence");
}
StereoVisionApp::DataBlockFactory::FactorizableFlags BilSequenceAcquisitionDataFactory::factorizable() const {
    return RootDataBlock;
}
StereoVisionApp::DataBlock* BilSequenceAcquisitionDataFactory::factorizeDataBlock(StereoVisionApp::Project* parent) const {
    return new BilSequenceAcquisitionData(parent);
}

QString BilSequenceAcquisitionDataFactory::itemClassName() const {
    return BilSequenceAcquisitionData::staticMetaObject.className();
}

} // namespace PikaLTools
