#ifndef PIKALTOOLS_BILACQUISITIONDATA_H
#define PIKALTOOLS_BILACQUISITIONDATA_H

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/floatparameter.h>

#include <MultidimArrays/MultidimArrays.h>

#include <QMap>
#include <QString>
#include <QList>
#include <QJsonObject>
#include <QPoint>
#include <QPointF>

#include <optional>

#include "LibStevi/geometry/rotations.h"

#include "../../libs/io/read_envi_bil.h"

namespace PikaLTools {

class BilSequenceLandmark;

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

        /*!
         * \brief loadLcfData load the lcf data and cache a few additional data along the way
         * \return the lcf data
         */
        std::vector<EnviBilLcfLine> loadLcfData() const;

        inline int getNLines() const {
            if (!_nLines.has_value()) {
                _nLines = headerData().value("lines", "0").toInt();
            }

            return _nLines.value();
        }

        int getLcfNLines() const;

    protected:
        QString _bil_file_path;
        mutable std::optional<int> _nLines;
        mutable std::optional<int> _nLcfLines;
    };

    void setBilSequence(const QList<QString> &bilFiles);
    QList<QString> getBilFiles() const;
    inline QList<BilAcquisitionData> & getBilInfos() {
        return _bilSequence;
    }

    inline QList<BilAcquisitionData> const& getBilInfos() const {
        return _bilSequence;
    }

    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

    inline StereoVision::Geometry::AffineTransform<float> const& ecef2local() const {
        if (!_lcfDataLoaded) {
            loadLcfData();
        }

        return _ecef2local;
    }

    inline std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& localTrajectory() const {
        if (!_lcfDataLoaded) {
            loadLcfData();
        }

        return _localTrajectory;
    }

    template<typename T>
    Multidim::Array<T, 3> getBilData(int startLine, int lastLine) const {

        if (startLine >= lastLine) {
            return Multidim::Array<T, 3>();
        }

        std::vector<std::string> files;

        QStringList qfiles = getBilFiles();

        for (QString f : qfiles) {
            files.push_back(f.toStdString());
        }

        return read_bil_sequence<T>(files, startLine, lastLine);

    }

    template<typename T>
    bool sequenceMatchType() const {

        QString fFile = getBilFiles().first();
        std::string firstFile = fFile.toStdString();

        return envi_bil_img_match_type<T>(firstFile);
    }

    Multidim::Array<float, 3> getFloatBilData(int startLine, int lastLine) const;
    int nLinesInSequence() const;

    qint64 addBilSequenceLandmark(const QPointF &coordinates, bool uncertain = false, qreal sigma_pos = 1.0);
    qint64 addBilSequenceLandmark(const QPointF &coordinates, qint64 attacheLandmarkId, bool uncertain = false, qreal sigma_pos = 1.0);
    BilSequenceLandmark* getBilSequenceLandmark(qint64 id) const;
    QVector<BilSequenceLandmark*> getBilSequenceLandmarkByLandmarkId(qint64 id) const;
    void clearBilSequenceLandmark(qint64 id);
    qint64 getBilSequenceLandmarkAt(QPointF const& coord, float tol = 3);
    QVector<qint64> getAttachedLandmarksIds() const;

    int countPointsRefered(QSet<qint64> const& excluded = {}) const;
    int countPointsRefered(QVector<qint64> const& excluded) const;

Q_SIGNALS:

    void pointAdded(qint64 pt);
    void pointRemoved(qint64 pt);

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
    bool loadLcfData() const;

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

class BilSequenceLandmark : public StereoVisionApp::DataBlock
{
    Q_OBJECT
public:

    explicit BilSequenceLandmark(BilSequenceAcquisitionData* parent = nullptr);

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

    void setBilSequenceCoordinates(QPointF const& point);
    QPointF bilSequenceCoordinates() const;


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

    friend class BilSequenceAcquisitionData;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILACQUISITIONDATA_H