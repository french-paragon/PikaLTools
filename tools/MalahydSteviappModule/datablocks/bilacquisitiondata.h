#ifndef PIKALTOOLS_BILACQUISITIONDATA_H
#define PIKALTOOLS_BILACQUISITIONDATA_H

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/rigidbody.h>
#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/floatparameter.h>
#include <steviapp/datablocks/georeferenceddatablockinterface.h>
#include <steviapp/datablocks/observationssummaryinterface.h>

#include <steviapp/utils/statusoptionalreturn.h>

#include <MultidimArrays/MultidimArrays.h>

#include <QMap>
#include <QString>
#include <QList>
#include <QJsonObject>
#include <QPoint>
#include <QPointF>

#include <optional>

#include <StereoVision/geometry/rotations.h>

#include "../../libs/io/read_envi_bil.h"
#include "../../libs/processing/scanlinecleaner.h"

namespace StereoVisionApp {

class Landmark;
class ImageLandmark;
class Trajectory;
class PushBroomPinholeCamera;
class Mounting;

}

namespace PikaLTools {

class BilSequenceLandmark;

/*!
 * \brief The BilSequenceAcquisitionData class represent a bill sequence, along with a level arm and boresight
 */
class BilSequenceAcquisitionData : public StereoVisionApp::DataBlock, StereoVisionApp::ObservationsSummaryInterface
{
    Q_OBJECT
    Q_INTERFACES(StereoVisionApp::ObservationsSummaryInterface)

public:

    struct SequenceInfos { //this allows to run the optimizer in simulation mode without a bil file
        int lineWidth;
        int nLines;
        double initial_time;
        double time_per_line;
        double fLen;
    };

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

    qint64 assignedCamera() const;
    StereoVisionApp::PushBroomPinholeCamera* getAssignedCamera() const;
    QString getAssignedCameraName() const;
    void assignCamera(qint64 camId);

    qint64 assignedTrajectory() const;
    StereoVisionApp::Trajectory* getAssignedTrajectory() const;
    QString getAssignedTrajectoryName() const;
    void assignTrajectory(qint64 trajId);

    qint64 assignedMounting() const;
    StereoVisionApp::Mounting* getAssignedMounting() const;
    QString getAssignedMountingName() const;
    void assignMounting(qint64 mountingId);

    virtual QVector<GenericObservation> observationsInfos(QVector<qint64> const& target) const override;

    template<typename T>
    Multidim::Array<T, 3> getBilData(int startLine, int lastLine, std::vector<int> const& channels = {}) const {

        if (startLine >= lastLine) {
            return Multidim::Array<T, 3>();
        }

        std::vector<std::string> files;

        QStringList qfiles = getBilFiles();

        int sLine = startLine;
        int lLine = lastLine;

        int nTotalLines = 0;

        for (BilAcquisitionData const& bil : _bilSequence) {
            if  (nTotalLines + bil.getNLines() <= startLine) {
                sLine -= bil.getNLines();
                lLine -= bil.getNLines();

                nTotalLines += bil.getNLines();

                continue;
            }

            if (nTotalLines >= lastLine) {
                break;
            }

            files.push_back(bil.bilFilePath().toStdString());
            nTotalLines += bil.getNLines();
        }

        return read_bil_sequence<T>(files, sLine, lLine, channels);

    }

    template<typename T>
    bool sequenceMatchType() const {

        QString fFile = getBilFiles().first();
        std::string firstFile = fFile.toStdString();

        return envi_bil_img_match_type<T>(firstFile);
    }

    Multidim::Array<float, 3> getFloatBilData(int startLine, int lastLine, std::vector<int> const& channels = {}) const;
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

    bool isInfosOnly() const;
    const SequenceInfos &sequenceInfos() const;
    void setSequenceInfos(const SequenceInfos &newSequenceInfos);

    /*!
     * \brief getTimeFromPixCoord get the time a given (possibly subpixel) y pixel coordinate
     * \param yPos the y pixel index
     * \return the time, or -infinity if no time could be extracted
     */
    double getTimeFromPixCoord(double yPos) const;
    /*!
     * \brief getSensorViewDirections return the view vector, in sensor frame, for all pixels
     * \param optimized if true use optimized parameters, else used basic parameters.
     * \return the list of view directions (in sensor frame).
     */
    std::vector<std::array<double, 3>> getSensorViewDirections(bool optimized=true);

    double getFocalLen() const;
    double getBilWidth() const;

    double timeScale() const;
    void setTimeScale(double timeScale);

    double timeDelta() const;
    void setTimeDelta(double timeDelta);

    QString timeScaleStr() const;
    void setTimeScaleStr(QString timeScale);

    QString timeDeltaStr() const;
    void setTimeDeltaStr(QString timeDelta);

Q_SIGNALS:

    void pointAdded(qint64 pt);
    void pointRemoved(qint64 pt);

    void bilSequenceChanged();
    void assignedCameraChanged();
    void assignedTrajectoryChanged();
    void assignedMountingChanged();

    void timeScaleChanged();
    void timeDeltaChanged();

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

    SequenceInfos _sequenceInfos;

    mutable bool _ecefTrajectoryCached;
    mutable std::vector<StereoVision::Geometry::AffineTransform<float>> _ecefTrajectory; //trajectory, as a sequence of body to ecef poses
    mutable std::vector<double> _ecefTimes;
    double _timeScale; //scaling factor to convert the stored times to reference times in second.
    double _timeDelta;

    qint64 _assignedCamera;
    qint64 _assignedTrajectory; //the trajectory the bil sequence has been taken from.
    qint64 _assignedMounting; //the trajectory the bil sequence has been taken from.

    mutable std::vector<bool> _loadedTimes;
    mutable std::vector<double> _linesTimes;

    mutable std::vector<bool> _mask;
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

    /*!
     * \brief getRayInfos get the information about the ray from this landmark
     * \param optimizationSpace if true the ray is given in the project optimization space, if false, in plain ECEF
     * \return a 3x2 matrix, the first column is the sensor position at the time corresponding to the point, the second is the ray direction from the sensor.
     */
    StereoVisionApp::StatusOptionalReturn<Eigen::Matrix<double,3,2>> getRayInfos(bool optimizationSpace = true);

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
