#ifndef PIKALTOOLS_INPUTDTM_H
#define PIKALTOOLS_INPUTDTM_H


#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/floatparameter.h>
#include <steviapp/datablocks/georeferenceddatablockinterface.h>

#include <optional>

#include "io/georasterreader.h"

namespace PikaLTools {

class InputDtm: public StereoVisionApp::DataBlock
{
    Q_OBJECT

public:
    InputDtm(StereoVisionApp::Project *parent = nullptr);

    QString getDataSource() const;
    void setDataSource(QString const& source);

    QVariant minHeight() const;
    void setMinHeight(QVariant minHeight);

    QVariant maxHeight() const;
    void setMaxHeight(QVariant minHeight);

    std::optional<StereoVisionApp::Geo::GeoRasterData<float, 2>> dtmData() const;

Q_SIGNALS:

    void dataSourceChanged();

    void minHeightChanged();
    void maxHeightChanged();

protected:

    virtual QJsonObject encodeJson() const override;
    virtual void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    QString _dataSource;
    std::optional<float> _minHeight;
    std::optional<float> _maxHeight;

    mutable std::optional<StereoVisionApp::Geo::GeoRasterData<float, 2>> _dtmDataCache;
};

class InputDtmFactory : public StereoVisionApp::DataBlockFactory
{
    Q_OBJECT
public:
    explicit InputDtmFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual StereoVisionApp::DataBlock* factorizeDataBlock(StereoVisionApp::Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_INPUTDTM_H
