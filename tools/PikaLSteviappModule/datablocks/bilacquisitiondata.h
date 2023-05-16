#ifndef PIKALTOOLS_BILACQUISITIONDATA_H
#define PIKALTOOLS_BILACQUISITIONDATA_H

#include <steviapp/datablocks/project.h>

#include <QMap>
#include <QString>
#include <QList>
#include <QJsonObject>

#include "LibStevi/geometry/rotations.h"

namespace PikaLTools {

class BilSequenceAcquisitionData : public StereoVisionApp::DataBlock
{
    Q_OBJECT
public:
    BilSequenceAcquisitionData(StereoVisionApp::Project *parent = nullptr);

    class BilAcquisitionData {
    public:

        BilAcquisitionData(QString path = "");

        void setBilFilePath(QString filePath);

        QString bilFilePath() const;
        QString headerFilePath() const;
        QString lcfFilePath() const;

        QMap<QString, QString> headerData() const;

    protected:
        QString _bil_file_path;
    };

    void setBilSequence(const QList<QString> &bilFiles);
    QList<QString> getBilFiles() const;
    QList<BilAcquisitionData> getBilInfos() const;

    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

    inline StereoVision::Geometry::AffineTransform<float> const& ecef2local() const {
        if (!_lcfDataLoaded) {
            loadLcfData(getBilFiles());
        }

        return _ecef2local;
    }

    inline std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& localTrajectory() const {
        if (!_lcfDataLoaded) {
            loadLcfData(getBilFiles());
        }

        return _localTrajectory;
    }

Q_SIGNALS:

    void bilSequenceChanged();

protected:

    virtual QJsonObject encodeJson() const override;
    virtual void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    /*!
     * \brief loadLcfData load the lcf data from the disk
     * \param files the bil files
     * \return bool on success
     */
    bool loadLcfData(QList<QString> const& files) const;

    QList<BilAcquisitionData> _bilSequence;

    mutable bool _lcfDataLoaded;
    mutable StereoVision::Geometry::AffineTransform<float> _ecef2local;
    mutable std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> _localTrajectory;
};

class BilSequenceAcquisitionDataFactory : public StereoVisionApp::DataBlockFactory
{
    Q_OBJECT
public:
    explicit BilSequenceAcquisitionDataFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual StereoVisionApp::DataBlock* factorizeDataBlock(StereoVisionApp::Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILACQUISITIONDATA_H
