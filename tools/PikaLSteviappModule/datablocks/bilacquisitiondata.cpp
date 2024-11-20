#include "bilacquisitiondata.h"

#include "io/read_envi_bil.h"
#include <steviapp/datablocks/trajectory.h>
#include <steviapp/datablocks/itemdatamodel.h>

#include <StereoVision/geometry/rotations.h>

#include "geo/coordinate_conversions.h"

#include <QJsonArray>

#include <QFile>

#include <proj.h>

namespace PikaLTools {

BilSequenceAcquisitionData::BilSequenceAcquisitionData(StereoVisionApp::Project *parent) :
    StereoVisionApp::RigidBody(parent),
    _ecefTrajectoryCached(false),
    _sensorIndex(-1),
    _timeScale(1),
    _timeDelta(0),
    _o_f_pix(),
    _o_c_x(),
    _o_a0(),
    _o_a1(),
    _o_a2(),
    _o_a3(),
    _o_a4(),
    _o_a5(),
    _o_b0(),
    _o_b1(),
    _o_b2(),
    _o_b3(),
    _o_b4(),
    _o_b5()

{
    connect(this, &BilSequenceAcquisitionData::bilSequenceChanged, this, [this] () {
        _ecefTrajectoryCached = false;
    });

    extendDataModel();
}
BilSequenceAcquisitionData::BilAcquisitionData::BilAcquisitionData(QString path) :
    _bil_file_path(path),
    _nLines(std::nullopt),
    _nLcfLines(std::nullopt)
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
    std::optional<std::map<std::string, std::string>> optData = readBilHeaderData(_bil_file_path.toStdString());

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

std::vector<EnviBilLcfLine> BilSequenceAcquisitionData::BilAcquisitionData::loadLcfData() const {
    std::vector<EnviBilLcfLine> ret = read_envi_bil_lcf_data(lcfFilePath().toStdString());
    _nLcfLines = ret.size();
    return ret;
}

int BilSequenceAcquisitionData::BilAcquisitionData::getLcfNLines() const {
    if (!_nLcfLines.has_value()) {
        QFile lcf(lcfFilePath());
        _nLcfLines = loadLcfData().size();
    }

    return _nLcfLines.value();
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

    int nLines = nLinesInSequence();

    _loadedTimes.resize(std::max(nLines,0));
    _linesTimes.resize(std::max(nLines,0));

    std::fill(_loadedTimes.begin(), _loadedTimes.end(), false);

}

QList<QString> BilSequenceAcquisitionData::getBilFiles() const {

    QList<QString> ret;
    ret.reserve(_bilSequence.size());

    for (int i = 0; i < _bilSequence.size(); i++) {
        ret.push_back(_bilSequence[i].bilFilePath());
    }
    return ret;
}


bool BilSequenceAcquisitionData::loadLcfData() const {

    _ecefTrajectory.clear();
    _ecefTimes.clear();

    PJ_CONTEXT* ctx = proj_context_create();

    const char* wgs84_latlonalt = "EPSG:4326";
    const char* wgs84_ecef = "EPSG:4978";
    PJ* converter = proj_create_crs_to_crs(ctx, wgs84_latlonalt, wgs84_ecef, nullptr);

    if (converter == 0) { //in case of error
        return false;
    }

    bool ecef2LocalEstimated = false;
    double lat0;
    double lon0;

    StereoVision::Geometry::AffineTransform<double> ecef2localAlt0;

    for (BilAcquisitionData const& bil : _bilSequence) {
        std::vector<EnviBilLcfLine> lines = bil.loadLcfData();

        _ecefTrajectory.reserve(_ecefTrajectory.size() + lines.size());
        _ecefTimes.reserve(_ecefTimes.size() + lines.size());

        for (EnviBilLcfLine line : lines) {

            if (!ecef2LocalEstimated) {
                lat0 = line.lat;
                lon0 = line.lon;

                ecef2localAlt0 = getLocalFrameAtPos<double>(lat0, lon0);

                ecef2LocalEstimated = true;
            }

            double vx = line.lat;
            double vy = line.lon;
            double vz = line.height;

            Eigen::Vector3f pos;

            proj_trans_generic(converter, PJ_FWD, &vx, 0, 1, &vy, 0, 1, &vz, 0, 1, nullptr, 0, 1);

            Eigen::Vector3f t(vx, vy, vz);

            StereoVision::Geometry::AffineTransform<double> ecef2ned = getLocalFrameAtPosNED<double>(line.lat, line.lon, line.height);

            line.pitch = line.pitch;
            line.roll = -line.roll;

            Eigen::Matrix3f yawRMat;
            yawRMat << cos(line.yaw), -sin(line.yaw), 0,
                       sin(line.yaw), cos(line.yaw), 0,
                       0, 0, 1;

            Eigen::Matrix3f pitchRMat;
            pitchRMat << cos(line.pitch), 0, sin(line.pitch),
                         0, 1, 0,
                         -sin(line.pitch), 0, cos(line.pitch);

            Eigen::Matrix3f rollRMat;
            rollRMat << 1, 0, 0,
                        0, cos(line.roll), -sin(line.roll),
                        0, sin(line.roll), cos(line.roll);

            Eigen::Matrix3f RIMU2NED = yawRMat*pitchRMat*rollRMat;

            Eigen::Matrix3f cam2imu = Eigen::Matrix3f::Identity();
            cam2imu << 0, 1, 0,
                       -1, 0, 0,
                       0, 0, 1;

            _ecefTrajectory.push_back(StereoVision::Geometry::AffineTransform<float>(ecef2ned.R.transpose().cast<float>()*RIMU2NED*cam2imu,t));
            _ecefTimes.push_back(line.timeStamp);
        }
    }

    if (_ecefTrajectory.empty()) {
        return false;
    }

    proj_destroy(converter);
    proj_context_destroy(ctx);

    _ecefTrajectoryCached = true;
    return true;
}

bool BilSequenceAcquisitionData::isInfosOnly() const {
    return _bilSequence.isEmpty();
}

const BilSequenceAcquisitionData::SequenceInfos &BilSequenceAcquisitionData::sequenceInfos() const
{
    return _sequenceInfos;
}

void BilSequenceAcquisitionData::setSequenceInfos(const SequenceInfos &newSequenceInfos)
{
    _sequenceInfos = newSequenceInfos;
}

double BilSequenceAcquisitionData::getTimeFromPixCoord(double yPos) const {

    if (isInfosOnly()) {
        return _sequenceInfos.initial_time + yPos*_sequenceInfos.time_per_line;
    }

    int floor = std::floor(yPos);
    int ceil = floor + 1;

    int nLines = 0;

    if (_loadedTimes[floor] and _loadedTimes[ceil]) {
        goto compute_subtime;
    }

    for (int i = 0; i < _bilSequence.size(); i++) {

        int bilnLines = _bilSequence[i].getNLines();

        QString bilFilePath = _bilSequence[i].bilFilePath();

        if (!_loadedTimes[nLines]) {

            std::vector<double> bilTimes = get_envi_bil_lines_times(bilFilePath.toStdString());

            if (bilTimes.size() != bilnLines) { // error loading times
                return 0;
            }

            for (int i = 0; i < bilnLines; i++) {
                _loadedTimes[nLines+i] = true;
                _linesTimes[nLines+i] = _timeScale*bilTimes[i] + _timeDelta;
            }
        }

        nLines += bilnLines;

        if (nLines > ceil) {
            break;
        }
    }

    compute_subtime:

    double pTime = _linesTimes[floor];
    double nTime = _linesTimes[ceil];

    double wp = ceil - yPos;
    double wn = yPos - floor;

    return wp*pTime + wn*nTime;
}

std::vector<std::array<double, 3>> BilSequenceAcquisitionData::getSensorViewDirections(bool optimized) {

    int nSamples = std::ceil(getBilWidth());

    double optical_center;
    double f_len_pix;

    double a0 = 0;
    double a1 = 0;
    double a2 = 0;
    double a3 = 0;
    double a4 = 0;
    double a5 = 0;

    double b0 = 0;
    double b1 = 0;
    double b2 = 0;
    double b3 = 0;
    double b4 = 0;
    double b5 = 0;

    if (optimized) {
        optical_center = optimizedOpticalCenterX().value();
        f_len_pix = optimizedFLen().value();

        a0 = optimizedA0().value();
        a1 = optimizedA1().value();
        a2 = optimizedA2().value();
        a3 = optimizedA3().value();
        a4 = optimizedA4().value();
        a5 = optimizedA5().value();

        b0 = optimizedB0().value();
        b1 = optimizedB1().value();
        b2 = optimizedB2().value();
        b3 = optimizedB3().value();
        b4 = optimizedB4().value();
        b5 = optimizedB5().value();

    } else {
        optical_center = getBilWidth()/2;
        f_len_pix = getFocalLen();
    }

    std::vector<std::array<double, 3>> viewDirectionsSensor(nSamples);

    for (int i = 0; i < nSamples; i++) {

        double s = static_cast<double>(i)/nSamples;
        double s2 = s*s;
        double s3 = s2*s;
        double s4 = s3*s;
        double s5 = s4*s;

        double du = a0 + a1*s + a2*s2 + a3*s3 + a4*s4 + a5*s5;
        double dv = b0 + b1*s + b2*s2 + b3*s3 + b4*s4 + b5*s5;

        viewDirectionsSensor[i] = std::array<double, 3>{0 + dv, i - optical_center + du, f_len_pix};
    }

    return viewDirectionsSensor;

}

double BilSequenceAcquisitionData::getFocalLen() const {

    if (isInfosOnly()) {
        return _sequenceInfos.fLen;
    }

    std::optional<std::map<std::string, std::string>> headerData = readBilHeaderData(getBilFiles()[0].toStdString());

    if (!headerData.has_value()) {
        return -1;
    }

    if (headerData.value().count("field of view") < 1 or headerData.value().count("samples") < 1) {
        return -1;
    }

    //TODO: add a cache
    double fov = std::stod(headerData.value()["field of view"]);
    double sensorWidth = std::stod(headerData.value()["samples"]);
    double fov_rad = fov*M_PI/180.;
    double fLen = sensorWidth*std::tan(M_PI_2-fov_rad/2.)/2.;

    return fLen;

}
double BilSequenceAcquisitionData::getBilWidth() const {

    if (isInfosOnly()) {
        return _sequenceInfos.lineWidth;
    }

    std::optional<std::map<std::string, std::string>> headerData = readBilHeaderData(getBilFiles()[0].toStdString());

    if (!headerData.has_value()) {
        return -1;
    }

    if (headerData.value().count("samples") < 1) {
        return -1;
    }

    //TODO: add a cache
    double sensorWidth = std::stod(headerData.value()["samples"]);

    return sensorWidth;
}


int BilSequenceAcquisitionData::sensorIndex() const {
    return _sensorIndex;
}
void BilSequenceAcquisitionData::setSensorIndex(int pSensorIndex) {
    if (pSensorIndex != _sensorIndex) {
        _sensorIndex = pSensorIndex;
        Q_EMIT sensorIndexChanged(_sensorIndex);
    }
}

double BilSequenceAcquisitionData::timeScale() const {
    return _timeScale;
}
void BilSequenceAcquisitionData::setTimeScale(double timeScale) {
    if (_timeScale != timeScale) {
        _timeScale = timeScale;
        Q_EMIT timeScaleChanged();
    }
}

double BilSequenceAcquisitionData::timeDelta() const {
    return _timeDelta;
}
void BilSequenceAcquisitionData::setTimeDelta(double timeDelta) {
    if (_timeDelta != timeDelta) {
        _timeDelta = timeDelta;
        Q_EMIT timeDeltaChanged();
    }
}

QString BilSequenceAcquisitionData::timeScaleStr() const {
    return QString("%1").arg(_timeScale);
}
void BilSequenceAcquisitionData::setTimeScaleStr(QString timeScale) {
    bool ok;
    double scale = timeScale.toDouble(&ok);
    if (!ok) {
        return;
    }
    setTimeScale(scale);
}

QString BilSequenceAcquisitionData::timeDeltaStr() const {
    return QString("%1").arg(_timeDelta);
}
void BilSequenceAcquisitionData::setTimeDeltaStr(QString timeDelta) {
    bool ok;
    double delta = timeDelta.toDouble(&ok);
    if (!ok) {
        return;
    }
    setTimeDelta(delta);

}


StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedFLen() const {
    return _o_f_pix;
}
void BilSequenceAcquisitionData::setOptimizedFLen(StereoVisionApp::floatParameter const& o_f_pix) {

    if (!_o_f_pix.isApproximatlyEqual(o_f_pix, 1e-4)) {
        _o_f_pix = o_f_pix;
        Q_EMIT optimizedFLenChanged(_o_f_pix);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedOpticalCenterX() const {
    return _o_c_x;
}
void BilSequenceAcquisitionData::setOptimizedOpticalCenterX(StereoVisionApp::floatParameter const& o_c_x) {

    if (!_o_c_x.isApproximatlyEqual(o_c_x, 1e-4)) {
        _o_c_x = o_c_x;
        Q_EMIT optimizedOpticalCenterXChanged(_o_c_x);
        isChanged();
    }
}

StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA0() const {
    return _o_a0;
}
void BilSequenceAcquisitionData::setOptimizedA0(StereoVisionApp::floatParameter const& o_a0) {
    if (!_o_a0.isApproximatlyEqual(o_a0, 1e-8)) {
        _o_a0 = o_a0;
        Q_EMIT optimizedA0Changed(_o_a0);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA1() const {
    return _o_a1;
}
void BilSequenceAcquisitionData::setOptimizedA1(StereoVisionApp::floatParameter const& o_a1) {
    if (!_o_a1.isApproximatlyEqual(o_a1, 1e-8)) {
        _o_a1 = o_a1;
        Q_EMIT optimizedA1Changed(_o_a1);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA2() const {
    return _o_a2;
}
void BilSequenceAcquisitionData::setOptimizedA2(StereoVisionApp::floatParameter const& o_a2) {
    if (!_o_a2.isApproximatlyEqual(o_a2, 1e-8)) {
        _o_a2 = o_a2;
        Q_EMIT optimizedA2Changed(_o_a2);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA3() const {
    return _o_a3;
}
void BilSequenceAcquisitionData::setOptimizedA3(StereoVisionApp::floatParameter const& o_a3) {
    if (!_o_a3.isApproximatlyEqual(o_a3, 1e-8)) {
        _o_a3 = o_a3;
        Q_EMIT optimizedA3Changed(_o_a3);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA4() const {
    return _o_a4;
}
void BilSequenceAcquisitionData::setOptimizedA4(StereoVisionApp::floatParameter const& o_a4) {
    if (!_o_a4.isApproximatlyEqual(o_a4, 1e-8)) {
        _o_a4 = o_a4;
        Q_EMIT optimizedA4Changed(_o_a4);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedA5() const {
    return _o_a5;
}
void BilSequenceAcquisitionData::setOptimizedA5(StereoVisionApp::floatParameter const& o_a5) {
    if (!_o_a5.isApproximatlyEqual(o_a5, 1e-8)) {
        _o_a5 = o_a5;
        Q_EMIT optimizedA5Changed(_o_a5);
        isChanged();
    }
}

StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB0() const {
    return _o_b0;
}
void BilSequenceAcquisitionData::setOptimizedB0(StereoVisionApp::floatParameter const& o_b0) {
    if (!_o_b0.isApproximatlyEqual(o_b0, 1e-8)) {
        _o_b0 = o_b0;
        Q_EMIT optimizedB0Changed(_o_b0);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB1() const {
    return _o_b1;
}
void BilSequenceAcquisitionData::setOptimizedB1(StereoVisionApp::floatParameter const& o_b1) {
    if (!_o_b1.isApproximatlyEqual(o_b1, 1e-8)) {
        _o_b1 = o_b1;
        Q_EMIT optimizedB1Changed(_o_b1);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB2() const {
    return _o_b2;
}
void BilSequenceAcquisitionData::setOptimizedB2(StereoVisionApp::floatParameter const& o_b2) {
    if (!_o_b2.isApproximatlyEqual(o_b2, 1e-8)) {
        _o_b2 = o_b2;
        Q_EMIT optimizedB2Changed(_o_b2);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB3() const {
    return _o_b3;
}
void BilSequenceAcquisitionData::setOptimizedB3(StereoVisionApp::floatParameter const& o_b3) {
    if (!_o_b3.isApproximatlyEqual(o_b3, 1e-8)) {
        _o_b3 = o_b3;
        Q_EMIT optimizedB3Changed(_o_b3);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB4() const {
    return _o_b4;
}
void BilSequenceAcquisitionData::setOptimizedB4(StereoVisionApp::floatParameter const& o_b4) {
    if (!_o_b4.isApproximatlyEqual(o_b4, 1e-8)) {
        _o_b4 = o_b4;
        Q_EMIT optimizedB4Changed(_o_b4);
        isChanged();
    }
}
StereoVisionApp::floatParameter BilSequenceAcquisitionData::optimizedB5() const {
    return _o_b5;
}
void BilSequenceAcquisitionData::setOptimizedB5(StereoVisionApp::floatParameter const& o_b5) {
    if (!_o_b5.isApproximatlyEqual(o_b5, 1e-8)) {
        _o_b5 = o_b5;
        Q_EMIT optimizedB5Changed(_o_b5);
        isChanged();
    }
}

void BilSequenceAcquisitionData::clearOptimized() {
    return;
}
bool BilSequenceAcquisitionData::hasOptimizedParameters() const {
    return false;
}

qint64 BilSequenceAcquisitionData::assignedTrajectory() const {
    return _assignedTrajectory;
}

StereoVisionApp::Trajectory* BilSequenceAcquisitionData::getAssignedTrajectory() const {
    return getProject()->getDataBlock<StereoVisionApp::Trajectory>(_assignedTrajectory);
}

QString BilSequenceAcquisitionData::getAssignedTrajectoryName() const {
    StereoVisionApp::Trajectory* traj = getAssignedTrajectory();

    if (traj == nullptr) {
        return tr("No trajectory");
    }

    return traj->objectName();
}

void BilSequenceAcquisitionData::assignTrajectory(qint64 trajId) {

    if (trajId == _assignedTrajectory) {
        return;
    }

    if (_assignedTrajectory >= 0) removeRefered({_assignedTrajectory});
    _assignedTrajectory = trajId;
    if (_assignedTrajectory >= 0) addRefered({_assignedTrajectory});
    emit assignedTrajectoryChanged();
    return;

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

bool BilSequenceAcquisitionData::geoReferenceSupportActive() const {
    return true;
}
Eigen::Array<float,3, Eigen::Dynamic> BilSequenceAcquisitionData::getLocalPointsEcef() const {

    bool ok = true;

    if (!_ecefTrajectoryCached) {
        ok = loadLcfData();
    }

    Eigen::Array<float,3, Eigen::Dynamic> ret;

    if (!ok) {
        ret.resize(3,0);
        return ret;
    }

    ret.resize(3,1);
    ret.col(0) = _ecefTrajectory[0].t;

    return ret;

}
QString BilSequenceAcquisitionData::getCoordinateReferenceSystemDescr(int CRSRole) const {
    Q_UNUSED(CRSRole)
    return "EPSG:4978";
}

QJsonObject BilSequenceAcquisitionData::encodeJson() const {

    QJsonObject obj = RigidBody::encodeJson();

    QJsonArray bilFiles;

    for (int i = 0; i < _bilSequence.size(); i++) {
        bilFiles.push_back(_bilSequence[i].bilFilePath());
    }
    obj.insert("bilFiles", bilFiles);

    QJsonArray arr;

    auto ids = listTypedSubDataBlocks(BilSequenceLandmark::staticMetaObject.className());
    for(qint64 id : qAsConst(ids)) {
        arr.push_back(getBilSequenceLandmark(id)->toJson());
    }

    obj.insert("Landmarks", arr);

    QJsonObject seqInfos;

    seqInfos.insert("fLen", _sequenceInfos.fLen);
    seqInfos.insert("initialTime", _sequenceInfos.initial_time);
    seqInfos.insert("lineWidth", _sequenceInfos.lineWidth);
    seqInfos.insert("nLines", _sequenceInfos.nLines);
    seqInfos.insert("timePerLine", _sequenceInfos.time_per_line);

    obj.insert("SeqInfos", seqInfos);

    if (_sensorIndex != -1) {
        obj.insert("sensorIdx", _sensorIndex);
    }

    if (_assignedTrajectory >= 0) {
        obj.insert("assignedTrajectoryId", _assignedTrajectory);
    }

    obj.insert("timeScale", _timeScale);
    obj.insert("timeDelta", _timeDelta);


    obj.insert("of", StereoVisionApp::floatParameter::toJson(optimizedFLen()));
    obj.insert("ocx", StereoVisionApp::floatParameter::toJson(optimizedOpticalCenterX()));

    obj.insert("oa0", StereoVisionApp::floatParameter::toJson(optimizedA0()));
    obj.insert("oa1", StereoVisionApp::floatParameter::toJson(optimizedA1()));
    obj.insert("oa2", StereoVisionApp::floatParameter::toJson(optimizedA2()));
    obj.insert("oa3", StereoVisionApp::floatParameter::toJson(optimizedA3()));
    obj.insert("oa4", StereoVisionApp::floatParameter::toJson(optimizedA4()));
    obj.insert("oa5", StereoVisionApp::floatParameter::toJson(optimizedA5()));

    obj.insert("ob0", StereoVisionApp::floatParameter::toJson(optimizedB0()));
    obj.insert("ob1", StereoVisionApp::floatParameter::toJson(optimizedB1()));
    obj.insert("ob2", StereoVisionApp::floatParameter::toJson(optimizedB2()));
    obj.insert("ob3", StereoVisionApp::floatParameter::toJson(optimizedB3()));
    obj.insert("ob4", StereoVisionApp::floatParameter::toJson(optimizedB4()));
    obj.insert("ob5", StereoVisionApp::floatParameter::toJson(optimizedB5()));

    return obj;
}

void BilSequenceAcquisitionData::configureFromJson(QJsonObject const& data) {

    RigidBody::configureFromJson(data);

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

    if (data.contains("Landmarks")) {
        QJsonArray arr = data.value("Landmarks").toArray();

        for (QJsonValue const& v : arr) {
            QJsonObject o = v.toObject();

            BilSequenceLandmark* lm = new BilSequenceLandmark(this);
            lm->setFromJson(o);

            if (lm->internalId() >= 0) {
                insertSubItem(lm);
            }
        }
    }

    if (data.contains("SeqInfos")) {
        QJsonObject seqInfos = data.value("SeqInfos").toObject();

        _sequenceInfos.fLen = seqInfos.value("fLen").toDouble();
        _sequenceInfos.initial_time = seqInfos.value("initialTime").toDouble();
        _sequenceInfos.lineWidth = seqInfos.value("lineWidth").toInt();
        _sequenceInfos.nLines = seqInfos.value("nLines").toInt();
        _sequenceInfos.time_per_line = seqInfos.value("timePerLine").toDouble();
    }

    if (data.contains("sensorIdx")) {
        _sensorIndex = data.value("sensorIdx").toInt(-1);
    } else {
        _sensorIndex = -1;
    }

    if(data.contains("assignedTrajectoryId")) {
        _assignedTrajectory = data.value("assignedTrajectoryId").toInt(-1);
    } else {
        _assignedTrajectory = -1;
    }

    if (data.contains("timeScale")) {
        _timeScale = data.value("timeScale").toDouble(1);
    } else {
        _timeScale = 1;
    }

    if (data.contains("timeDelta")) {
        _timeDelta = data.value("timeDelta").toDouble(0);
    } else {
        _timeDelta = 0;
    }


    if (data.contains("of")) {
        _o_f_pix = StereoVisionApp::floatParameter::fromJson(data.value("of").toObject());
    }

    if (data.contains("ocx")) {
        _o_c_x = StereoVisionApp::floatParameter::fromJson(data.value("ocx").toObject());
    }



    if (data.contains("oa0")) {
        _o_a0 = StereoVisionApp::floatParameter::fromJson(data.value("oa0").toObject());
    }

    if (data.contains("oa1")) {
        _o_a1 = StereoVisionApp::floatParameter::fromJson(data.value("oa1").toObject());
    }

    if (data.contains("oa2")) {
        _o_a2 = StereoVisionApp::floatParameter::fromJson(data.value("oa2").toObject());
    }

    if (data.contains("oa3")) {
        _o_a3 = StereoVisionApp::floatParameter::fromJson(data.value("oa3").toObject());
    }

    if (data.contains("oa4")) {
        _o_a4 = StereoVisionApp::floatParameter::fromJson(data.value("oa4").toObject());
    }

    if (data.contains("oa5")) {
        _o_a5 = StereoVisionApp::floatParameter::fromJson(data.value("oa5").toObject());
    }



    if (data.contains("ob0")) {
        _o_b0 = StereoVisionApp::floatParameter::fromJson(data.value("ob0").toObject());
    }

    if (data.contains("ob1")) {
        _o_b1 = StereoVisionApp::floatParameter::fromJson(data.value("ob1").toObject());
    }

    if (data.contains("ob2")) {
        _o_b2 = StereoVisionApp::floatParameter::fromJson(data.value("ob2").toObject());
    }

    if (data.contains("ob3")) {
        _o_b3 = StereoVisionApp::floatParameter::fromJson(data.value("ob3").toObject());
    }

    if (data.contains("ob4")) {
        _o_b4 = StereoVisionApp::floatParameter::fromJson(data.value("ob4").toObject());
    }

    if (data.contains("ob5")) {
        _o_b5 = StereoVisionApp::floatParameter::fromJson(data.value("ob5").toObject());
    }
}

void BilSequenceAcquisitionData::extendDataModel() {
    //TODO: add more options to extend the data model


    StereoVisionApp::ItemDataModel::Category* tjg = _dataModel->addCategory(tr("Trajectory properties"));

    tjg->addCatProperty<QString,
            BilSequenceAcquisitionData,
            false,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal> (
                tr("Trajectory"),
                &BilSequenceAcquisitionData::getAssignedTrajectoryName,
                nullptr,
                &BilSequenceAcquisitionData::assignedTrajectoryChanged
                );

    StereoVisionApp::ItemDataModel::Category* tg = _dataModel->addCategory(tr("Timing properties"));

    tg->addCatProperty<QString,
            BilSequenceAcquisitionData,
            false,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("Time scale"),
                &BilSequenceAcquisitionData::timeScaleStr,
                &BilSequenceAcquisitionData::setTimeScaleStr,
                &BilSequenceAcquisitionData::timeScaleChanged
                );
    tg->addCatProperty<QString,
            BilSequenceAcquisitionData,
            false,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("Time delta"),
                &BilSequenceAcquisitionData::timeDeltaStr,
                &BilSequenceAcquisitionData::setTimeDeltaStr,
                &BilSequenceAcquisitionData::timeDeltaChanged
                );

    StereoVisionApp::ItemDataModel::Category* g = _dataModel->addCategory(tr("Lever arm properties"));

    //Position
    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("X"),
             &StereoVisionApp::RigidBody::xCoord,
             &StereoVisionApp::RigidBody::setXCoord,
             &StereoVisionApp::RigidBody::xCoordChanged);

    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Y"),
             &StereoVisionApp::RigidBody::yCoord,
             &StereoVisionApp::RigidBody::setYCoord,
             &StereoVisionApp::RigidBody::yCoordChanged);

    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Z"),
             &StereoVisionApp::RigidBody::yCoord,
             &StereoVisionApp::RigidBody::setYCoord,
             &StereoVisionApp::RigidBody::yCoordChanged);

    //Rotation
    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("RAxis X"),
             &StereoVisionApp::RigidBody::xRot,
             &StereoVisionApp::RigidBody::setXRot,
             &StereoVisionApp::RigidBody::xRotChanged);

    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("RAxis Y"),
             &StereoVisionApp::RigidBody::yRot,
             &StereoVisionApp::RigidBody::setYRot,
             &StereoVisionApp::RigidBody::yRotChanged);

    g->addCatProperty<StereoVisionApp::floatParameter,
            StereoVisionApp::RigidBody,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Raxis Z"),
             &StereoVisionApp::RigidBody::zRot,
             &StereoVisionApp::RigidBody::setZRot,
             &StereoVisionApp::RigidBody::zRotChanged);



    StereoVisionApp::ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

    optCat->addCatProperty<bool, DataBlock, false, StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Enabled"),
             &DataBlock::isEnabled,
             &DataBlock::setEnabled,
             &DataBlock::isEnabledChanged);

    optCat->addCatProperty<bool, DataBlock, false, StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Fixed"),
             &DataBlock::isFixed,
             &DataBlock::setFixed,
             &DataBlock::isFixedChanged);

    optCat->addCatProperty<int, BilSequenceAcquisitionData, false, StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("SensorIdx"),
             &BilSequenceAcquisitionData::sensorIndex,
             &BilSequenceAcquisitionData::setSensorIndex,
             &BilSequenceAcquisitionData::sensorIndexChanged);

    StereoVisionApp::ItemDataModel::Category* og = _dataModel->addCategory(tr("Optimized lever arm"));

    //Position
    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("X pos"),
             &StereoVisionApp::RigidBody::optXCoord,
             &StereoVisionApp::RigidBody::setOptXCoord,
             &StereoVisionApp::RigidBody::optPosChanged);

    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("Y pos"),
             &StereoVisionApp::RigidBody::optYCoord,
             &StereoVisionApp::RigidBody::setOptYCoord,
             &StereoVisionApp::RigidBody::optPosChanged);

    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("Z pos"),
             &StereoVisionApp::RigidBody::optZCoord,
             &StereoVisionApp::RigidBody::setOptZCoord,
             &StereoVisionApp::RigidBody::optPosChanged);

    //Rotation
    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("X Raxis"),
             &StereoVisionApp::RigidBody::optXRot,
             &StereoVisionApp::RigidBody::setOptXRot,
             &StereoVisionApp::RigidBody::optRotChanged);

    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("Y Raxis"),
             &StereoVisionApp::RigidBody::optYRot,
             &StereoVisionApp::RigidBody::setOptYRot,
             &StereoVisionApp::RigidBody::optRotChanged);

    og->addCatProperty<float, StereoVisionApp::RigidBody, true, StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal>
            (tr("Z Raxis"),
             &StereoVisionApp::RigidBody::optZRot,
             &StereoVisionApp::RigidBody::setOptZRot,
             &StereoVisionApp::RigidBody::optRotChanged);

    StereoVisionApp::ItemDataModel::Category* oip = _dataModel->addCategory(tr("Optimized intrisic parameters"));

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Focal length [px]"),
             &BilSequenceAcquisitionData::optimizedFLen,
             nullptr,
             &BilSequenceAcquisitionData::optimizedFLenChanged);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("Optical center X"),
             &BilSequenceAcquisitionData::optimizedOpticalCenterX,
             nullptr,
             &BilSequenceAcquisitionData::optimizedOpticalCenterXChanged);



    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a0"),
             &BilSequenceAcquisitionData::optimizedA0,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA0Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a1"),
             &BilSequenceAcquisitionData::optimizedA1,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA1Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a2"),
             &BilSequenceAcquisitionData::optimizedA2,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA2Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a3"),
             &BilSequenceAcquisitionData::optimizedA3,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA3Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a4"),
             &BilSequenceAcquisitionData::optimizedA4,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA4Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("a5"),
             &BilSequenceAcquisitionData::optimizedA5,
             nullptr,
             &BilSequenceAcquisitionData::optimizedA5Changed);



    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b0"),
             &BilSequenceAcquisitionData::optimizedB0,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB0Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b1"),
             &BilSequenceAcquisitionData::optimizedB1,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB1Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b2"),
             &BilSequenceAcquisitionData::optimizedB2,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB2Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b3"),
             &BilSequenceAcquisitionData::optimizedB3,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB3Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b4"),
             &BilSequenceAcquisitionData::optimizedB4,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB4Changed);

    oip->addCatProperty<StereoVisionApp::floatParameter,
            BilSequenceAcquisitionData,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>
            (tr("b5"),
             &BilSequenceAcquisitionData::optimizedB5,
             nullptr,
             &BilSequenceAcquisitionData::optimizedB5Changed);
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

std::optional<Eigen::Matrix<double,3,2>> BilSequenceLandmark::getRayInfos(bool optimizationSpace) {

    if (!_x.isSet() or !_y.isSet()) {
        return std::nullopt;
    }

    StereoVision::Geometry::AffineTransform<double> ecef2optim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    if (optimizationSpace) {
        StereoVisionApp::Project* proj = getProject();

        if (proj != nullptr) {
            ecef2optim = proj->ecef2local().cast<double>();
        }
    }

    BilSequenceAcquisitionData* seqData = qobject_cast<BilSequenceAcquisitionData*>(parent());

    if (seqData == nullptr) {
        return std::nullopt;
    }

    StereoVision::Geometry::RigidBodyTransform<double> leverArm(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));

    StereoVisionApp::floatParameterGroup<3> oPos = seqData->optPos();
    StereoVisionApp::floatParameterGroup<3> oRot = seqData->optRot();

    if (oPos.isSet() and oRot.isSet()) {
        leverArm.r = (Eigen::Vector3d(oRot.value(0),oRot.value(1),oRot.value(2)));
        leverArm.t = (Eigen::Vector3d(oPos.value(0),oPos.value(1),oPos.value(2)));
    } else {
        StereoVisionApp::floatParameter xpos = seqData->xCoord();
        StereoVisionApp::floatParameter ypos = seqData->yCoord();
        StereoVisionApp::floatParameter zpos = seqData->zCoord();

        StereoVisionApp::floatParameter xrot = seqData->xRot();
        StereoVisionApp::floatParameter yrot = seqData->yRot();
        StereoVisionApp::floatParameter zrot = seqData->zRot();

        if (xpos.isSet() and ypos.isSet() and zpos.isSet() and
                xrot.isSet() and yrot.isSet() and zrot.isSet()) {

            leverArm.r = (Eigen::Vector3d(xrot.value(),yrot.value(),zrot.value()));
            leverArm.t = (Eigen::Vector3d(xpos.value(),ypos.value(),zpos.value()));

        }
    }

    StereoVisionApp::Trajectory* traj = seqData->getAssignedTrajectory();

    if (traj == nullptr) {
        return std::nullopt;
    }

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> posSeq =
            traj->loadTrajectorySequence(); //sequence is platform 2 ecef

    if (!posSeq.isValid()) {
        return std::nullopt;
    }

    double time = seqData->getTimeFromPixCoord(_y.value());
    double flen = seqData->getFocalLen();

    if (flen < 0) {
        return std::nullopt;
    }

    double midPoint = seqData->getBilWidth()/2;

    auto interpolablePose = posSeq.value().getValueAtTime(time);

    StereoVision::Geometry::RigidBodyTransform<double> pose = interpolablePose.weigthLower*interpolablePose.valLower + interpolablePose.weigthUpper*interpolablePose.valUpper;

    Eigen::Matrix<double,3,2> ret;
    ret.block<3,1>(0,0) = ecef2optim*(StereoVision::Geometry::angleAxisRotate(pose.r,leverArm.t) + pose.t);

    Eigen::Vector3d dir(0,(_x.value() - midPoint)/flen,1);
    ret.block<3,1>(0,1) = ecef2optim.R*StereoVision::Geometry::angleAxisRotate(pose.r,StereoVision::Geometry::angleAxisRotate(leverArm.r,dir));

    return ret;
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
