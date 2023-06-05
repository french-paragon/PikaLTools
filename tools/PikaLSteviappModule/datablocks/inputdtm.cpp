#include "inputdtm.h"

#include <steviapp/datablocks/itemdatamodel.h>

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

std::optional<GeoRasterData<float, 2>> InputDtm::dtmData() const {
    if (_dtmDataCache.has_value()) {
        return _dtmDataCache;
    }

    _dtmDataCache = readGeoRasterData<float,2>(getDataSource().toStdString());
    return _dtmDataCache;
}

QJsonObject InputDtm::encodeJson() const {
    QJsonObject obj;
    obj.insert("datasource", _dataSource);

    if (_minHeight.has_value()) {
        obj.insert("minHeight", _minHeight.value());
    }

    if (_maxHeight.has_value()) {
        obj.insert("maxHeight", _maxHeight.value());
    }

    return obj;
}

void InputDtm::configureFromJson(QJsonObject const& data) {

    if (data.contains("datasource")) {
        _dataSource = data.value("datasource").toString();
    }

    if (data.contains("minHeight")) {
        _minHeight = data.value("minHeight").toDouble();
    }

    if (data.contains("maxHeight")) {
        _maxHeight = data.value("maxHeight").toDouble();
    }

}

void InputDtm::extendDataModel() {

    constexpr auto NoValueSignal = StereoVisionApp::ItemDataModel::ItemPropertyDescription::NoValueSignal;

    StereoVisionApp::ItemDataModel::Category* p = _dataModel->addCategory(tr("Dtm boundaries"));



    p->addCatProperty<QVariant, InputDtm, false, NoValueSignal> (tr("Min height"),
                                                            &InputDtm::minHeight,
                                                            &InputDtm::setMinHeight,
                                                            &InputDtm::minHeightChanged);


    p->addCatProperty<QVariant, InputDtm, false, NoValueSignal> (tr("Max height"),
                                                            &InputDtm::maxHeight,
                                                            &InputDtm::setMaxHeight,
                                                            &InputDtm::maxHeightChanged);
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
