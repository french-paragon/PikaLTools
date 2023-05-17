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

Multidim::Array<float, 3> BilSequenceAcquisitionData::getFloatBilData(int startLine, int lastLine) const {

    if (sequenceMatchType<int8_t>()) {
        return getBilData<int8_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<uint8_t>()) {
        return getBilData<uint8_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<int16_t>()) {
        return getBilData<int16_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<uint16_t>()) {
        return getBilData<uint16_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<int32_t>()) {
        return getBilData<int32_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<uint32_t>()) {
        return getBilData<uint32_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<int64_t>()) {
        return getBilData<int64_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<uint64_t>()) {
        return getBilData<uint64_t>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<float>()) {
        return getBilData<float>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<float>()) {
        return getBilData<float>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<double>()) {
        return getBilData<double>(startLine, lastLine).cast<float>();
    }

    if (sequenceMatchType<double>()) {
        return getBilData<double>(startLine, lastLine).cast<float>();
    }

    return Multidim::Array<float, 3>();

}
int BilSequenceAcquisitionData::nLinesInSequence() const {

    QList<BilAcquisitionData> infos = getBilInfos();

    int total = 0;

    for (BilAcquisitionData const& data : infos) {
        QString nLines = data.headerData().value("lines", "");

        bool ok;
        int n = nLines.toInt(&ok);

        if (!ok) {
            return -1;
        }

        total += n;

    }

    return total;
}

qint64 BilSequenceAcquisitionData::addBilSequenceLandmark(const QPointF &coordinates, bool uncertain, qreal sigma_pos) {
    return addBilSequenceLandmark(coordinates, -1, uncertain, sigma_pos);
}
qint64 BilSequenceAcquisitionData::addBilSequenceLandmark(const QPointF &coordinates, qint64 attacheLandmarkId, bool uncertain, qreal sigma_pos) {
    BilSequenceLandmark* iml = new BilSequenceLandmark(this);

    iml->stopTrackingChanges(true);

    StereoVisionApp::floatParameter x(coordinates.x(), pFloatType(sigma_pos));
    StereoVisionApp::floatParameter y(coordinates.y(), pFloatType(sigma_pos));

    if (!uncertain) {
        x.clearUncertainty();
        y.clearUncertainty();
    }

    iml->setX(x);
    iml->setY(y);

    insertSubItem(iml);

    if (iml->internalId() >= 0) {
        iml->setAttachedLandmark(attacheLandmarkId);
        Q_EMIT pointAdded(iml->internalId());

        iml->stopTrackingChanges(false);
        return iml->internalId();
    } else {
        iml->clear();
        iml->deleteLater();
    }

    return -1;
}
BilSequenceLandmark* BilSequenceAcquisitionData::getBilSequenceLandmark(qint64 id) const {
    return qobject_cast<BilSequenceLandmark*>(getById(id));
}
QVector<BilSequenceLandmark*> BilSequenceAcquisitionData::getBilSequenceLandmarkByLandmarkId(qint64 id) const {

    QVector<qint64> lmIds = listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());

    QVector<BilSequenceLandmark*> ret;

    for (qint64 im_id : lmIds) {
        BilSequenceLandmark* imlm = getBilSequenceLandmark(im_id);

        if (imlm != nullptr) {
            if (imlm->attachedLandmarkid() == id) {
                ret.push_back(imlm);
            }
        }
    }

    return ret;
}
void BilSequenceAcquisitionData::clearBilSequenceLandmark(qint64 id) {

    BilSequenceLandmark* iml = getBilSequenceLandmark(id);

    if (iml != nullptr) {
        clearSubItem(id, iml->metaObject()->className());
        iml = getBilSequenceLandmark(id);
        if (iml == nullptr) {
            Q_EMIT pointRemoved(id);
        }
    }
}
qint64 BilSequenceAcquisitionData::getBilSequenceLandmarkAt(QPointF const& coord, float tol) {

    qint64 r = -1;
    float d = std::abs(tol);

    for(qint64 id : listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className())) {
        BilSequenceLandmark* ilm = getBilSequenceLandmark(id);

        if (ilm != nullptr) {

            QPointF delta = coord - ilm->bilSequenceCoordinates();

            float dm = std::max(std::abs(delta.x()), std::abs(delta.y()));
            if (dm < d) {
                r = id;
                d = dm;
            }
        }
    }

    return r;
}
QVector<qint64> BilSequenceAcquisitionData::getAttachedLandmarksIds() const {

    if (!isInProject()) {
        return {};
    }

    QVector<qint64> imlmids = listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());
    QSet<qint64> r;
    r.reserve(imlmids.size());

    for (qint64 id : imlmids) {
        BilSequenceLandmark* imlm = getBilSequenceLandmark(id);

        if (imlm == nullptr) {
            continue;
        }

        qint64 lmid = imlm->attachedLandmarkid();

        StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(getProject()->getById(lmid));

        if (lm != nullptr) {
            r.insert(lmid);
        }
    }

    return r.values().toVector();
}

int BilSequenceAcquisitionData::countPointsRefered(QSet<qint64> const& excluded) const {

    if (!isInProject()) {
        return 0;
    }

    QSet<qint64> referedPtId;
    for (qint64 id : listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className())) {
        BilSequenceLandmark* iml = qobject_cast<BilSequenceLandmark*>(getById(id));

        if (iml == nullptr) {
            continue;
        }

        qint64 lm_id = iml->attachedLandmarkid();
        StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(getProject()->getById(lm_id));

        if (lm != nullptr) {
            referedPtId.insert(id);
        }
    }

    for (qint64 id : excluded) {
        referedPtId.remove(id);
    }

    return referedPtId.count();
}
int BilSequenceAcquisitionData::countPointsRefered(QVector<qint64> const& excluded) const {

    QSet<qint64> s(excluded.begin(), excluded.end());
    return countPointsRefered(s);

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

BilSequenceLandmark::BilSequenceLandmark(BilSequenceAcquisitionData* parent) :
    StereoVisionApp::DataBlock(parent)
{
    connect(this, &BilSequenceLandmark::xCoordChanged, this, &BilSequenceLandmark::coordsChanged);
    connect(this, &BilSequenceLandmark::yCoordChanged, this, &BilSequenceLandmark::coordsChanged);
}

qint64 BilSequenceLandmark::attachedLandmarkid() const {
    return _attachedLandmarkId;
}
void BilSequenceLandmark::setAttachedLandmark(qint64 id) {

    if (id < 0 and _attachedLandmarkId >= 0) {
        removeRefered({_attachedLandmarkId});
        _attachedLandmarkId = -1;
        emit attachedLandmarkidChanged(-1);
        return;
    } else if (id < 0) {
        return;
    }

    if (id == _attachedLandmarkId) {
        return;
    }

    qint64 t = id;

    if (t != _attachedLandmarkId) {
        if (_attachedLandmarkId >= 0) removeRefered({_attachedLandmarkId});
        _attachedLandmarkId = t;
        if (_attachedLandmarkId >= 0) addRefered({_attachedLandmarkId});
        emit attachedLandmarkidChanged(_attachedLandmarkId);
        return;
    }
}
QString BilSequenceLandmark::attachedLandmarkName() const {
    StereoVisionApp::Project* p = getProject();

    if (p == nullptr) {
        return "";
    }

    StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(p->getById(attachedLandmarkid()));

    if (lm != nullptr) {
        return lm->objectName();
    }

    return "";
}

StereoVisionApp::Landmark* BilSequenceLandmark::attachedLandmark() const {
    StereoVisionApp::Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(p->getById(attachedLandmarkid()));

    return lm;
}

StereoVisionApp::floatParameter BilSequenceLandmark::x() const {
    return _x;
}
void BilSequenceLandmark::setX(const StereoVisionApp::floatParameter &x) {
    if (!x.isApproximatlyEqual(_x, 1e-4)) {
        _x = x;
        emit xCoordChanged(x);
        isChanged();
    }
}
void BilSequenceLandmark::setX(float x) {
    if (!_x.isApproximatlyEqual(x, 1e-4) or !_x.isSet()) {
        _x.setIsSet(x);
        emit xCoordChanged(_x);
        isChanged();
    }
}

StereoVisionApp::floatParameter BilSequenceLandmark::y() const {
    return _y;
}
void BilSequenceLandmark::setY(const StereoVisionApp::floatParameter &y) {
    if (!y.isApproximatlyEqual(_y, 1e-4)) {
        _y = y;
        Q_EMIT yCoordChanged(y);
        isChanged();
    }
}
void BilSequenceLandmark::setY(float y) {
    if (!_y.isApproximatlyEqual(y, 1e-4) or !_y.isSet()) {
        _y.setIsSet(y);
        Q_EMIT yCoordChanged(_y);
        isChanged();
    }
}

void BilSequenceLandmark::setBilSequenceCoordinates(QPointF const& point) {
    setX(point.x());
    setY(point.y());
}
QPointF BilSequenceLandmark::bilSequenceCoordinates() const {
    return QPointF(_x.value(), _y.value());
}

QJsonObject BilSequenceLandmark::encodeJson() const{
    QJsonObject obj;

    obj.insert("attachedLandmarkId", attachedLandmarkid());

    obj.insert("x", StereoVisionApp::floatParameter::toJson(x()));
    obj.insert("y", StereoVisionApp::floatParameter::toJson(y()));

    return obj;
}
void BilSequenceLandmark::configureFromJson(QJsonObject const& data) {
    if (data.contains("x")) {
        _x = StereoVisionApp::floatParameter::fromJson(data.value("x").toObject());
    }
    if (data.contains("y")) {
        _y = StereoVisionApp::floatParameter::fromJson(data.value("y").toObject());
    }

    if (data.contains("attachedLandmarkId")) {
        _attachedLandmarkId = data.value("attachedLandmarkId").toInt(-1);
    }
}

void BilSequenceLandmark::referedCleared(QVector<qint64> const& referedId) {
    DataBlock::referedCleared(referedId);

    if (referedId.front() == _attachedLandmarkId) {
        _attachedLandmarkId = -1;
        emit attachedLandmarkidChanged(-1);
    }
}

} // namespace PikaLTools
