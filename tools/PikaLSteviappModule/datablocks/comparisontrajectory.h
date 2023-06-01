#ifndef COMPARISONTRAJECTORY_H
#define COMPARISONTRAJECTORY_H

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/floatparameter.h>
#include <steviapp/datablocks/georeferenceddatablockinterface.h>

namespace PikaLTools {

class ComparisonTrajectory : public StereoVisionApp::DataBlock, public StereoVisionApp::GeoReferencedDataBlockInterface
{
    Q_OBJECT
    Q_INTERFACES(StereoVisionApp::GeoReferencedDataBlockInterface)
public:
    ComparisonTrajectory(StereoVisionApp::Project *parent = nullptr);

    QString getDataSource() const;
    void setDataSource(QString const& source);

    inline std::vector<Eigen::Vector3f> const& ecefTrajectory() const {
        if (!_ecefTrajectoryCached) {
            loadCsvData();
        }

        return _ecefTrajectory;
    }

    bool geoReferenceSupportActive() const override;
    Eigen::Array<float,3, Eigen::Dynamic> getLocalPointsEcef() const override;
    QString getCoordinateReferenceSystemDescr(int role = DefaultCRSRole) const override;

Q_SIGNALS:

    void dataSourceChanged();

protected:

    virtual QJsonObject encodeJson() const override;
    virtual void configureFromJson(QJsonObject const& data) override;

    bool loadCsvData() const;

    QString _dataSource;
    QString _crsDescr;

    mutable bool _ecefTrajectoryCached;
    mutable std::vector<Eigen::Vector3f> _ecefTrajectory;
};

class ComparisonTrajectoryFactory : public StereoVisionApp::DataBlockFactory
{
    Q_OBJECT
public:
    explicit ComparisonTrajectoryFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual StereoVisionApp::DataBlock* factorizeDataBlock(StereoVisionApp::Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} //namespace PikaLTools

#endif // COMPARISONTRAJECTORY_H
