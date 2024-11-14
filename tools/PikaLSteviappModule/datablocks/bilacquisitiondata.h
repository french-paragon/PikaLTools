#ifndef PIKALTOOLS_BILACQUISITIONDATA_H
#define PIKALTOOLS_BILACQUISITIONDATA_H

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/rigidbody.h>
#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/floatparameter.h>
#include <steviapp/datablocks/georeferenceddatablockinterface.h>

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

}

namespace PikaLTools {

class BilSequenceLandmark;

/*!
 * \brief The BilSequenceAcquisitionData class represent a bill sequence, along with a level arm and boresight
 */
class BilSequenceAcquisitionData : public StereoVisionApp::RigidBody, public StereoVisionApp::GeoReferencedDataBlockInterface
{
    Q_OBJECT
    Q_INTERFACES(StereoVisionApp::GeoReferencedDataBlockInterface)

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

    qint64 assignedTrajectory() const;
    StereoVisionApp::Trajectory* getAssignedTrajectory() const;
    QString getAssignedTrajectoryName() const;
    void assignTrajectory(qint64 trajId);

    inline std::vector<StereoVision::Geometry::AffineTransform<float>> const& ecefTrajectory() const {
        if (!_ecefTrajectoryCached) {
            loadLcfData();
        }

        return _ecefTrajectory;
    }

    inline std::vector<double> const& ecefTimes() const {
        if (!_ecefTrajectoryCached) {
            loadLcfData();
        }

        return _ecefTimes;
    }

    template<typename T>
    Multidim::Array<T, 3> getBilData(int startLine, int lastLine) const {

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

        return read_bil_sequence<T>(files, sLine, lLine);

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

    bool geoReferenceSupportActive() const override;
    Eigen::Array<float,3, Eigen::Dynamic> getLocalPointsEcef() const override;
    QString getCoordinateReferenceSystemDescr(int CRSRole = DefaultCRSRole) const override;

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

    int sensorIndex() const;
    void setSensorIndex(int sensorIndex);

    double timeScale() const;
    void setTimeScale(double timeScale);

    double timeDelta() const;
    void setTimeDelta(double timeDelta);

    QString timeScaleStr() const;
    void setTimeScaleStr(QString timeScale);

    QString timeDeltaStr() const;
    void setTimeDeltaStr(QString timeDelta);


    StereoVisionApp::floatParameter optimizedFLen() const;
    void setOptimizedFLen(StereoVisionApp::floatParameter const& o_f_pix);
    StereoVisionApp::floatParameter optimizedOpticalCenterX() const;
    void setOptimizedOpticalCenterX(StereoVisionApp::floatParameter const& o_c_x);

    StereoVisionApp::floatParameter optimizedA0() const;
    void setOptimizedA0(StereoVisionApp::floatParameter const& o_a0);
    StereoVisionApp::floatParameter optimizedA1() const;
    void setOptimizedA1(StereoVisionApp::floatParameter const& o_a1);
    StereoVisionApp::floatParameter optimizedA2() const;
    void setOptimizedA2(StereoVisionApp::floatParameter const& o_a2);
    StereoVisionApp::floatParameter optimizedA3() const;
    void setOptimizedA3(StereoVisionApp::floatParameter const& o_a3);
    StereoVisionApp::floatParameter optimizedA4() const;
    void setOptimizedA4(StereoVisionApp::floatParameter const& o_a4);
    StereoVisionApp::floatParameter optimizedA5() const;
    void setOptimizedA5(StereoVisionApp::floatParameter const& o_a5);

    StereoVisionApp::floatParameter optimizedB0() const;
    void setOptimizedB0(StereoVisionApp::floatParameter const& o_b0);
    StereoVisionApp::floatParameter optimizedB1() const;
    void setOptimizedB1(StereoVisionApp::floatParameter const& o_b1);
    StereoVisionApp::floatParameter optimizedB2() const;
    void setOptimizedB2(StereoVisionApp::floatParameter const& o_b2);
    StereoVisionApp::floatParameter optimizedB3() const;
    void setOptimizedB3(StereoVisionApp::floatParameter const& o_b3);
    StereoVisionApp::floatParameter optimizedB4() const;
    void setOptimizedB4(StereoVisionApp::floatParameter const& o_b4);
    StereoVisionApp::floatParameter optimizedB5() const;
    void setOptimizedB5(StereoVisionApp::floatParameter const& o_b5);

Q_SIGNALS:

    void pointAdded(qint64 pt);
    void pointRemoved(qint64 pt);

    void bilSequenceChanged();
    void assignedTrajectoryChanged();

    void sensorIndexChanged(int sensorIndex);

    void timeScaleChanged();
    void timeDeltaChanged();


    void optimizedFLenChanged(StereoVisionApp::floatParameter);
    void optimizedOpticalCenterXChanged(StereoVisionApp::floatParameter);

    void optimizedA0Changed(StereoVisionApp::floatParameter);
    void optimizedA1Changed(StereoVisionApp::floatParameter);
    void optimizedA2Changed(StereoVisionApp::floatParameter);
    void optimizedA3Changed(StereoVisionApp::floatParameter);
    void optimizedA4Changed(StereoVisionApp::floatParameter);
    void optimizedA5Changed(StereoVisionApp::floatParameter);

    void optimizedB0Changed(StereoVisionApp::floatParameter);
    void optimizedB1Changed(StereoVisionApp::floatParameter);
    void optimizedB2Changed(StereoVisionApp::floatParameter);
    void optimizedB3Changed(StereoVisionApp::floatParameter);
    void optimizedB4Changed(StereoVisionApp::floatParameter);
    void optimizedB5Changed(StereoVisionApp::floatParameter);

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
    int _sensorIndex; //used to get multiple lines to use the same sensor.

    SequenceInfos _sequenceInfos;

    mutable bool _ecefTrajectoryCached;
    mutable std::vector<StereoVision::Geometry::AffineTransform<float>> _ecefTrajectory; //trajectory, as a sequence of body to ecef poses
    mutable std::vector<double> _ecefTimes;
    double _timeScale; //scaling factor to convert the stored times to reference times in second.
    double _timeDelta;

    qint64 _assignedTrajectory; //the trajectory the bil sequence has been taken from.

    mutable std::vector<bool> _loadedTimes;
    mutable std::vector<double> _linesTimes;

    mutable std::vector<bool> _mask;


    StereoVisionApp::floatParameter _o_f_pix;
    StereoVisionApp::floatParameter _o_c_x;

    //Horizontal distortions
    StereoVisionApp::floatParameter _o_a0;
    StereoVisionApp::floatParameter _o_a1;
    StereoVisionApp::floatParameter _o_a2;
    StereoVisionApp::floatParameter _o_a3;
    StereoVisionApp::floatParameter _o_a4;
    StereoVisionApp::floatParameter _o_a5;

    //vertical distortions
    StereoVisionApp::floatParameter _o_b0;
    StereoVisionApp::floatParameter _o_b1;
    StereoVisionApp::floatParameter _o_b2;
    StereoVisionApp::floatParameter _o_b3;
    StereoVisionApp::floatParameter _o_b4;
    StereoVisionApp::floatParameter _o_b5;
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
    std::optional<Eigen::Matrix<double,3,2>> getRayInfos(bool optimizationSpace = true);

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
