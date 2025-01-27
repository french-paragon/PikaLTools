#include "comparisontrajectory.h"

#include <proj.h>

#include "geo/coordinate_conversions.h"

#include <QFile>

namespace PikaLTools {

ComparisonTrajectory::ComparisonTrajectory(StereoVisionApp::Project *parent) :
    StereoVisionApp::DataBlock(parent),
    _ecefTrajectoryCached(false),
    _crsDescr("EPSG:4326")
{

}

QString ComparisonTrajectory::getDataSource() const {
    return _dataSource;
}
void ComparisonTrajectory::setDataSource(QString const& source) {
    if (source != _dataSource) {
        _dataSource = source;
        _ecefTrajectoryCached = false;
        Q_EMIT dataSourceChanged();
    }
}

bool ComparisonTrajectory::geoReferenceSupportActive() const {
    return !_crsDescr.isEmpty();
}
Eigen::Array<float,3, Eigen::Dynamic> ComparisonTrajectory::getLocalPointsEcef() const {

    bool ok = true;

    if (!_ecefTrajectoryCached) {
        ok = loadCsvData();
    }

    Eigen::Array<float,3, Eigen::Dynamic> ret;

    if (!ok) {
        ret.resize(3,0);
        return ret;
    }

    ret.resize(3,1);
    ret.col(0) = _ecefTrajectory[0];

    return ret;
}
QString ComparisonTrajectory::getCoordinateReferenceSystemDescr(int role) const {
    Q_UNUSED(role);
    return "EPSG:4978";
    //return _crsDescr; //TODO: create a role for raw CRS
}

QJsonObject ComparisonTrajectory::encodeJson() const {
    QJsonObject obj;
    obj.insert("datasource", _dataSource);

    return obj;
}

void ComparisonTrajectory::configureFromJson(QJsonObject const& data) {

    if (data.contains("datasource")) {
        _dataSource = data.value("datasource").toString();
    }
}

bool ComparisonTrajectory::loadCsvData() const {

    _ecefTrajectory.clear();
    _ecefTimings.clear();

    PJ_CONTEXT* ctx = proj_context_create();

    std::string tmp = _crsDescr.toStdString();
    const char* input_crs = tmp.c_str();
    const char* wgs84_ecef = "EPSG:4978";
    PJ* converter = proj_create_crs_to_crs(ctx, input_crs, wgs84_ecef, nullptr);

    if (converter == 0) { //in case of error
        return false;
    }

    QFile inputFile(_dataSource);

    if (!inputFile.open(QFile::ReadOnly)) {
        return false;
    }

    QTextStream in(&inputFile);

    while (!in.atEnd()) {

        QString line = in.readLine();

        if (line.startsWith("$")) {
            continue;
        }

        QStringList splitted = line.split("\t", Qt::SkipEmptyParts);

        if (splitted.size() < 5) {
            continue;
        }

        QString timeStr = splitted[0];
        QString latStr = splitted[2];
        QString lonStr = splitted[3];
        QString altStr = splitted[4];

        bool ok;
        double time = timeStr.toDouble(&ok);

        if (!ok) {
            continue;
        }

        double lat = latStr.toDouble(&ok);

        if (!ok) {
            continue;
        }

        double lon = lonStr.toDouble(&ok);

        if (!ok) {
            continue;
        }

        double alt = altStr.toDouble(&ok);

        if (!ok) {
            continue;
        }

        double vx = lat;
        double vy = lon;
        double vz = alt;

        proj_trans_generic(converter, PJ_FWD, &vx, 0, 1, &vy, 0, 1, &vz, 0, 1, nullptr, 0, 1);

        Eigen::Vector3f t(vx, vy, vz);

        _ecefTrajectory.push_back(t);
        _ecefTimings.push_back(time);
    }

    if (_ecefTrajectory.empty()) {
        return false;
    }

    proj_destroy(converter);
    proj_context_destroy(ctx);

    _ecefTrajectoryCached = true;
    return true;

}

ComparisonTrajectoryFactory::ComparisonTrajectoryFactory(QObject* parent) :
    StereoVisionApp::DataBlockFactory(parent)
{

}

QString ComparisonTrajectoryFactory::TypeDescrName() const {
    return tr("Comparison Trajectory");
}
StereoVisionApp::DataBlockFactory::FactorizableFlags ComparisonTrajectoryFactory::factorizable() const {
    return RootDataBlock;
}

StereoVisionApp::DataBlock* ComparisonTrajectoryFactory::factorizeDataBlock(StereoVisionApp::Project *parent) const {
    return new ComparisonTrajectory(parent);
}

QString ComparisonTrajectoryFactory::itemClassName() const {
    return ComparisonTrajectory::staticMetaObject.className();
}

}
