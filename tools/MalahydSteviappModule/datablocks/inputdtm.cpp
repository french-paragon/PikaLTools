#include "inputdtm.h"

#include <steviapp/datablocks/itemdatamodel.h>

#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/dataexception.h>

namespace PikaLTools {

InputDtm::InputDtm(StereoVisionApp::Project *parent) :
    StereoVisionApp::DataBlock(parent),
    _minHeight(0),
    _maxHeight(std::nullopt)
{
    extendDataModel();
}

QString InputDtm::getDataSource() const {
    return _dataSource;
}
void InputDtm::setDataSource(QString const& source) {
    if (source != _dataSource) {
        _dataSource = source;
        _dtmDataCache = std::nullopt;
        Q_EMIT dataSourceChanged();
    }
}

QString InputDtm::getCrsOverride() const {
    return _crs_override;
}
void InputDtm::setCrsOverride(QString const& crsOverride) {
    if (crsOverride != _crs_override) {
        _crs_override = crsOverride;
        if (_dtmDataCache.has_value()) {
            _dtmDataCache.value().crsInfos = _crs_override.toStdString();
        }
        Q_EMIT crsOverrideChanged();
    }
}

QVariant InputDtm::minHeight() const {
    if (_minHeight.has_value()) {
        return _minHeight.value();
    }
    return QVariant();
}
void InputDtm::setMinHeight(QVariant min) {

    bool ok;
    float val = min.toFloat(&ok);

    if (!ok) {

        if (!_minHeight.has_value()) {
            return;
        }

        _minHeight = std::nullopt;
        Q_EMIT minHeightChanged();
    }

    if (_minHeight.has_value()) {
        if (_minHeight.value() != val) {
            _minHeight = val;
            Q_EMIT minHeightChanged();
        }
    } else {
        _minHeight = val;
        Q_EMIT minHeightChanged();
    }
}

QVariant InputDtm::maxHeight() const {
    if (_maxHeight.has_value()) {
        return _maxHeight.value();
    }
    return QVariant();
}
void InputDtm::setMaxHeight(QVariant max) {

    bool ok;
    float val = max.toFloat(&ok);

    if (!ok) {

        if (!_maxHeight.has_value()) {
            return;
        }

        _maxHeight = std::nullopt;
        Q_EMIT maxHeightChanged();
    }

    if (_maxHeight.has_value()) {
        if (_maxHeight.value() != val) {
            _maxHeight = val;
            Q_EMIT maxHeightChanged();
        }
    } else {
        _maxHeight = val;
        Q_EMIT maxHeightChanged();
    }
}

std::optional<StereoVisionApp::Geo::GeoRasterData<float, 2>> InputDtm::dtmData() const {
    if (_dtmDataCache.has_value()) {
        if (!_crs_override.isEmpty()) {
            _dtmDataCache.value().crsInfos = _crs_override.toStdString();
        }
        return _dtmDataCache;
    }

    _dtmDataCache = readGeoRasterData<float,2>(getDataSource().toStdString());
    if (!_crs_override.isEmpty()) {
        _dtmDataCache.value().crsInfos = _crs_override.toStdString();
    }
    return _dtmDataCache;
}

qint64 InputDtm::addDtmLandmark(const QPointF &coordinates,
                                bool uncertain,
                                qreal sigma_pos) {

    return addDtmLandmark(coordinates, -1, uncertain, sigma_pos);

}
qint64 InputDtm::addDtmLandmark(const QPointF &coordinates,
                                qint64 attacheLandmarkId,
                                bool uncertain,
                                qreal sigma_pos) {

    DtmLandmark* dtml = new DtmLandmark(this);

    dtml->stopTrackingChanges(true);

    StereoVisionApp::floatParameter x(coordinates.x(), pFloatType(sigma_pos));
    StereoVisionApp::floatParameter y(coordinates.y(), pFloatType(sigma_pos));

    if (!uncertain) {
        x.clearUncertainty();
        y.clearUncertainty();
    }

    dtml->setX(x);
    dtml->setY(y);

    insertSubItem(dtml);

    if (dtml->internalId() >= 0) {
        dtml->setAttachedLandmark(attacheLandmarkId);
        emit pointAdded(dtml->internalId());

        dtml->stopTrackingChanges(false);

        return dtml->internalId();
    } else {
        dtml->clear();
        dtml->deleteLater();
    }

    return -1;
}

DtmLandmark* InputDtm::getDtmLandmark(qint64 id) const {
    return qobject_cast<DtmLandmark*>(getById(id));
}

DtmLandmark* InputDtm::getDtmLandmarkByLandmarkId(qint64 id) const {
    QVector<qint64> lmIds = listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());

    for (qint64 im_id : lmIds) {
        DtmLandmark* imlm = getDtmLandmark(im_id);

        if (imlm != nullptr) {
            if (imlm->attachedLandmarkid() == id) {
                return imlm;
            }
        }
    }

    return nullptr;
}

void InputDtm::clearDtmLandmark(qint64 id) {

    DtmLandmark* dtml = getDtmLandmark(id);

    if (dtml != nullptr) {
        clearSubItem(id, dtml->metaObject()->className());
        dtml = getDtmLandmark(id);
        if (dtml == nullptr) {
            Q_EMIT pointRemoved(id);
        }
    }
}

qint64 InputDtm::getDtmLandmarkAt(QPointF const& coord, float tol) {

    qint64 r = -1;
    float d = std::abs(tol);

    for(qint64 id : listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className())) {
        DtmLandmark* ilm = getDtmLandmark(id);

        if (ilm != nullptr) {

            QPointF delta = coord - ilm->imageCoordinates();

            float dm = std::max(std::abs(delta.x()), std::abs(delta.y()));
            if (dm < d) {
                r = id;
                d = dm;
            }
        }
    }

    return r;
}

QVector<qint64> InputDtm::getAttachedLandmarksIds() const {

    if (!isInProject()) {
        return {};
    }

    QVector<qint64> imlmids = listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());
    QVector<qint64> r;
    r.reserve(imlmids.size());

    for (qint64 id : imlmids) {
        DtmLandmark* imlm = getDtmLandmark(id);

        if (imlm == nullptr) {
            continue;
        }

        qint64 lmid = imlm->attachedLandmarkid();

        StereoVisionApp::Landmark* lm =
                qobject_cast<StereoVisionApp::Landmark*>(getProject()->getById(lmid));

        if (lm != nullptr) {
            r.push_back(lmid);
        }
    }

    return r;

}

Eigen::Array2Xf InputDtm::getDtmLandmarksCoordinates(QVector<qint64> ids) const {

    Eigen::Array2Xf coords;
    coords.resize(2, ids.size());

    for (int i = 0; i < ids.size(); i++) {

        DtmLandmark* lm = getDtmLandmarkByLandmarkId(ids[i]);

        if (lm == nullptr) {
            throw StereoVisionApp::DataException("Asked for coordinate of an image landmark not in the image !", this);
        }

        coords(0,i) = lm->x().value();
        coords(1,i) = lm->y().value();
    }

    return coords;
}

QJsonObject InputDtm::encodeJson() const {
    QJsonObject obj;
    obj.insert("datasource", _dataSource);
    obj.insert("crsoverride", _crs_override);

    if (_minHeight.has_value()) {
        obj.insert("minHeight", _minHeight.value());
    }

    if (_maxHeight.has_value()) {
        obj.insert("maxHeight", _maxHeight.value());
    }

    QJsonArray arr;

    auto ids = listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());
    for(qint64 id : qAsConst(ids)) {
        arr.push_back(getDtmLandmark(id)->toJson());
    }

    obj.insert("Landmarks", arr);

    return obj;

    return obj;
}

void InputDtm::configureFromJson(QJsonObject const& data) {

    if (data.contains("datasource")) {
        _dataSource = data.value("datasource").toString();
    }

    if (data.contains("crsoverride")) {
        _crs_override = data.value("crsoverride").toString();
    } else {
        _crs_override = "";
    }

    if (data.contains("minHeight")) {
        _minHeight = data.value("minHeight").toDouble();
    }

    if (data.contains("maxHeight")) {
        _maxHeight = data.value("maxHeight").toDouble();
    }

    if (data.contains("Landmarks")) {
        QJsonArray arr = data.value("Landmarks").toArray();

        for (QJsonValue const& v : arr) {
            QJsonObject o = v.toObject();

            DtmLandmark* dtml = new DtmLandmark(this);
            dtml->setFromJson(o);

            if (dtml->internalId() >= 0) {
                insertSubItem(dtml);
            }
        }
    }

}

void InputDtm::extendDataModel() {

    constexpr auto NoValueSignal = StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal;

    StereoVisionApp::ItemDataModel::Category* i = _dataModel->addCategory(tr("Dtm infos"));

    i->addCatProperty<QString, InputDtm, false, NoValueSignal> (tr("File"),
                                                                &InputDtm::getDataSource,
                                                                nullptr,
                                                                &InputDtm::dataSourceChanged);

    i->addCatProperty<QString, InputDtm, true, NoValueSignal> (tr("Crs override"),
                                                                &InputDtm::getCrsOverride,
                                                                &InputDtm::setCrsOverride,
                                                                &InputDtm::crsOverrideChanged);

    StereoVisionApp::ItemDataModel::Category* p = _dataModel->addCategory(tr("Dtm boundaries"));



    p->addCatProperty<QVariant, InputDtm, false, NoValueSignal> (tr("Min height"),
                                                            &InputDtm::minHeight,
                                                            &InputDtm::setMinHeight,
                                                            &InputDtm::minHeightChanged);


    p->addCatProperty<QVariant, InputDtm, false, NoValueSignal> (tr("Max height"),
                                                            &InputDtm::maxHeight,
                                                            &InputDtm::setMaxHeight,
                                                            &InputDtm::maxHeightChanged);

    StereoVisionApp::ItemDataModel::SubItemCollectionManager* im_lm =
            _dataModel->addCollectionManager(tr("Dtm landmarks"),
                                             DtmLandmark::staticMetaObject.className(),
                                             [] (DataBlock* b) {
                                                DtmLandmark* l = qobject_cast<DtmLandmark*>(b);
                                                if (l != nullptr) {
                                                    return l->attachedLandmarkName();
                                                }
                                                return tr("Unvalid image landmark");
                                             });

    im_lm->addCatProperty<StereoVisionApp::floatParameter,
            DtmLandmark,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
                                                                                        &DtmLandmark::x,
                                                                                        &DtmLandmark::setX,
                                                                                        &DtmLandmark::xCoordChanged);

    im_lm->addCatProperty<StereoVisionApp::floatParameter,
            DtmLandmark,
            true,
            StereoVisionApp::ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
                                                                                        &DtmLandmark::y,
                                                                                        &DtmLandmark::setY,
                                                                                        &DtmLandmark::yCoordChanged);
}


DtmLandmark::DtmLandmark(InputDtm* parent) :
    StereoVisionApp::DataBlock(parent)
{

}

qint64 DtmLandmark::attachedLandmarkid() const {
    return _attachedLandmarkId;
}
void DtmLandmark::setAttachedLandmark(qint64 id) {

    if (id == _attachedLandmarkId) {
        return;
    }

    if (id < 0 and _attachedLandmarkId >= 0) {
        removeRefered({_attachedLandmarkId});
        _attachedLandmarkId = -1;
        emit attachedLandmarkidChanged(-1);
        return;

    } else if (id < 0) {
        return;
    }

    qint64 t = id;

    InputDtm* dtm_p = qobject_cast<InputDtm*>(parent());

    if (dtm_p != nullptr) {
        QVector<qint64> imLandmarkIds = dtm_p->listTypedSubDataBlocks(this->metaObject()->className());

        for (qint64 id : imLandmarkIds) {
            DtmLandmark* dtml = dtm_p->getDtmLandmark(id);

            if (dtml != nullptr) {
                if (dtml->attachedLandmarkid() == t) {
                    t = -1;
                    break;
                }
            }
        }
    }

    if (t != _attachedLandmarkId) {
        if (_attachedLandmarkId >= 0) removeRefered({_attachedLandmarkId});
        _attachedLandmarkId = t;
        if (_attachedLandmarkId >= 0) addRefered({_attachedLandmarkId});
        emit attachedLandmarkidChanged(_attachedLandmarkId);
        return;
    }

}
QString DtmLandmark::attachedLandmarkName() const {
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
StereoVisionApp::Landmark* DtmLandmark::attachedLandmark() const {
    StereoVisionApp::Project* p = getProject();

    if (p == nullptr) {
        return nullptr;
    }

    StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(p->getById(attachedLandmarkid()));

    return lm;
}

StereoVisionApp::floatParameter DtmLandmark::x() const {
    return _x;
}
void DtmLandmark::setX(const StereoVisionApp::floatParameter &x) {
    if (!x.isApproximatlyEqual(_x, 1e-4)) {
        _x = x;
        emit xCoordChanged(x);
        isChanged();
    }
}
void DtmLandmark::setX(float x) {
    if (!_x.isApproximatlyEqual(x, 1e-4) or !_x.isSet()) {
        _x.setIsSet(x);
        emit xCoordChanged(_x);
        isChanged();
    }
}

StereoVisionApp::floatParameter DtmLandmark::y() const {
    return _y;
}
void DtmLandmark::setY(const StereoVisionApp::floatParameter &y) {
    if (!y.isApproximatlyEqual(_y, 1e-4)) {
        _y = y;
        emit yCoordChanged(y);
        isChanged();
    }
}
void DtmLandmark::setY(float y) {
    if (!_y.isApproximatlyEqual(y, 1e-4) or !_y.isSet()) {
        _y.setIsSet(y);
        emit yCoordChanged(_y);
        isChanged();
    }
}

void DtmLandmark::setImageCoordinates(QPointF const& point) {
    setX(point.x());
    setY(point.y());
}
QPointF DtmLandmark::imageCoordinates() const {
    return QPointF(_x.value(), _y.value());
}


QJsonObject DtmLandmark::encodeJson() const {
    QJsonObject obj;

    obj.insert("attachedLandmarkId", attachedLandmarkid());

    obj.insert("x", StereoVisionApp::floatParameter::toJson(x()));
    obj.insert("y", StereoVisionApp::floatParameter::toJson(y()));

    return obj;
}

void DtmLandmark::configureFromJson(QJsonObject const& data) {

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

void DtmLandmark::referedCleared(QVector<qint64> const& referedId) {

    DataBlock::referedCleared(referedId);

    if (referedId.front() == _attachedLandmarkId) {
        _attachedLandmarkId = -1;
        emit attachedLandmarkidChanged(-1);
    }

}


InputDtmFactory::InputDtmFactory(QObject* parent) :
    StereoVisionApp::DataBlockFactory(parent)
{

}

QString InputDtmFactory::TypeDescrName() const {
    return tr("Input dtm");
}

InputDtmFactory::FactorizableFlags InputDtmFactory::factorizable() const {
    return RootDataBlock;
}

StereoVisionApp::DataBlock* InputDtmFactory::factorizeDataBlock(StereoVisionApp::Project *parent) const {
    return new InputDtm(parent);
}

QString InputDtmFactory::itemClassName() const {
    return InputDtm::staticMetaObject.className();
}

} // namespace PikaLTools
