#ifndef PIKALTOOLS_INPUTDTM_H
#define PIKALTOOLS_INPUTDTM_H


#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/floatparameter.h>
#include <steviapp/datablocks/georeferenceddatablockinterface.h>

#include <steviapp/datablocks/image.h>

#include <optional>

#include "io/georasterreader.h"

namespace PikaLTools {

class DtmLandmark;

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

    qint64 addDtmLandmark(const QPointF &coordinates, bool uncertain = false, qreal sigma_pos = 1.0);
    qint64 addDtmLandmark(const QPointF &coordinates, qint64 attacheLandmarkId, bool uncertain = false, qreal sigma_pos = 1.0);
    DtmLandmark* getDtmLandmark(qint64 id) const;
    DtmLandmark* getDtmLandmarkByLandmarkId(qint64 id) const;
    void clearDtmLandmark(qint64 id);
    qint64 getDtmLandmarkAt(QPointF const& coord, float tol = 3);
    QVector<qint64> getAttachedLandmarksIds() const;

    Eigen::Array2Xf getDtmLandmarksCoordinates(QVector<qint64> ids) const;

Q_SIGNALS:

    void pointAdded(qint64 pt);
    void pointRemoved(qint64 pt);

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

class DtmLandmark : public StereoVisionApp::DataBlock
{
    Q_OBJECT
public:

    explicit DtmLandmark(InputDtm* parent = nullptr);

    qint64 attachedLandmarkid() const;
    void setAttachedLandmark(qint64 id);
    QString attachedLandmarkName() const;
    StereoVisionApp::Landmark* attachedLandmark() const;

    StereoVisionApp::floatParameter x() const;
    void setX(const StereoVisionApp::floatParameter &x);
    void setX(float x);

    StereoVisionApp::floatParameter y() const;
    void setY(const StereoVisionApp::floatParameter &y);
    void setY(float y);

    void setImageCoordinates(QPointF const& point);
    QPointF imageCoordinates() const;


Q_SIGNALS:

    void attachedLandmarkidChanged(qint64 id);

    void xCoordChanged(StereoVisionApp::floatParameter);
    void yCoordChanged(StereoVisionApp::floatParameter);

    void coordsChanged();

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void referedCleared(QVector<qint64> const& referedId) override;

    qint64 _attachedLandmarkId;

    StereoVisionApp::floatParameter _x;
    StereoVisionApp::floatParameter _y;

    friend class InputDtm;

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
