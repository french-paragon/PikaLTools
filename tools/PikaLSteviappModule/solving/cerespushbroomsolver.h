#ifndef PIKALTOOLS_CERESPUSHBROOMSOLVER_H
#define PIKALTOOLS_CERESPUSHBROOMSOLVER_H

#include <steviapp/sparsesolver/sparsesolverbase.h>

#include <steviapp/datablocks/datatable.h>
#include <steviapp/vision/indexed_timed_sequence.h>

#include <ceres/ceres.h>

#include <QMap>

#include <optional>

#include "../datablocks/bilacquisitiondata.h"
#include "../gui/datacolumnsselectionwidget.h"

namespace StereoVisionApp {
    class DataTable;
}

namespace PikaLTools {

class BilSequenceAcquisitionData;

class CeresPushBroomSolver : public StereoVisionApp::SparseSolverBase
{
public:

    enum RotationRepresentations {
        AngleAxis,
        Quaternion,
        EulerXYZ
    };

    struct viewInfos {
        qint64 lmId;
        qint64 additionalPtId;
        double x;
        double y;
    };

    struct pointInfos {
        qint64 additionalPtId;
        double x;
        double y;
        double z;
    };

    struct timedVecMeasurement {
        double time;
        double x;
        double y;
        double z;
    };

    using SequenceInfos = BilSequenceAcquisitionData::SequenceInfos; //this allows to run the optimizer in simulation mode without a bil file

    using IndexedVec3TimeSeq = StereoVisionApp::IndexedTimeSequence<std::array<float,3>, double>;

    struct AdditionalViewsInfos {
        StereoVisionApp::DataTable* dataTable;

        QString colLmId; //set if the point correspond to an existing landmark
        QString colPtId; //set if the point do not correspond to an existing landmark
        QString colPosX; //pos in the line
        QString colPosY; //pos in scanlines (across time)

        QVector<CeresPushBroomSolver::viewInfos> compilePoints();
    };

    struct AdditionalPointsInfos {
        StereoVisionApp::DataTable* dataTable;

        QString colPtId; //correspond to id in AdditionalViewsInfos
        QString colPosX;
        QString colPosY;
        QString colPosZ;

        QVector<CeresPushBroomSolver::pointInfos> compilePoints();
    };

    struct GPSMeasurementInfos {
        StereoVisionApp::DataTable* dataTable;

        QString colTime;
        QString colPosX;
        QString colPosY;
        QString colPosZ;

        /*!
         * \brief configureFromColumnsSelection configure a GPSMeasurementInfos from a ColumnSelectionInfos
         * \param colSelectionInfos the column selection info struct
         * \return optionally an error message.
         */
        std::optional<QString> configureFromColumnsSelection(DataColumnsSelectionWidget::ColumnSelectionInfos const& colSelectionInfos);
        IndexedVec3TimeSeq compileSequence();
    };

    struct InitialOrientationInfos {
        StereoVisionApp::DataTable* dataTable;

        RotationRepresentations rotationRepresentation;

        QString colTime;
        QString colRotW;
        QString colRotX;
        QString colRotY;
        QString colRotZ;

        /*!
         * \brief configureFromColumnsSelection configure an InitialOrientationInfos from a ColumnSelectionInfos
         * \param colSelectionInfos the column selection info struct
         * \return optionally an error message.
         */
        std::optional<QString> configureFromColumnsSelection(DataColumnsSelectionWidget::ColumnSelectionInfos const& colSelectionInfos);

        /*!
         * \brief compileSequence give the sequence of orientations as axis angles
         * \return the sequence represented as axis angles
         */
        IndexedVec3TimeSeq compileSequence();
    };

    struct AccMeasurementInfos {
        StereoVisionApp::DataTable* dataTable;

        QString colTime;
        QString colAccX;
        QString colAccY;
        QString colAccZ;

        IndexedVec3TimeSeq compileSequence();
    };

    struct GyroMeasurementInfos {
        StereoVisionApp::DataTable* dataTable;

        RotationRepresentations rotationRepresentation;

        QString colTime;
        QString colAngSpeedW;
        QString colAngSpeedX;
        QString colAngSpeedY;
        QString colAngSpeedZ;


        /*!
         * \brief compileSequence give the sequence of angular speed as axis angles
         * \return the sequence represented as axis angles
         */
        IndexedVec3TimeSeq compileSequence();
    };

    CeresPushBroomSolver(StereoVisionApp::Project* p,
                         StereoVision::Geometry::AffineTransform<double> const& initialLeverArm,
                         BilSequenceAcquisitionData* push_broom_sequence,
                         GPSMeasurementInfos const& gpsMeasurements,
                         InitialOrientationInfos const& initialOrientations,
                         AccMeasurementInfos const& insMeasurements,
                         GyroMeasurementInfos const& imuMeasurements,
                         AdditionalPointsInfos const& additionalPoints,
                         AdditionalViewsInfos const& additionalViews,
                         bool computeUncertainty = true,
                         bool sparse = true,
                         QObject* parent = nullptr);

    CeresPushBroomSolver(StereoVisionApp::Project* p,
                         StereoVision::Geometry::AffineTransform<double> const& initialLeverArm,
                         SequenceInfos const& pseudo_push_broom_sequence,
                         GPSMeasurementInfos const& gpsMeasurements,
                         InitialOrientationInfos const& initialOrientations,
                         AccMeasurementInfos const& insMeasurements,
                         GyroMeasurementInfos const& imuMeasurements,
                         AdditionalPointsInfos const& additionalPoints,
                         AdditionalViewsInfos const& additionalViews,
                         bool computeUncertainty = true,
                         bool sparse = true,
                         QObject* parent = nullptr);

    int uncertaintySteps() const override;
    bool hasUncertaintyStep() const override;

    double gpsAccuracy() const;
    void setGpsAccuracy(double newGpsAccuracy);

    double sensorWidth() const;
    void setSensorWidth(double newSensorWidth);

    double gyroAccuracy() const;
    void setGyroAccuracy(double newGyroAccuracy);

    double accAccuracy() const;
    void setAccAccuracy(double newAccAccuracy);

    double tiepointsAccuracy() const;
    void setTiepointsAccuracy(double newTiepointsAccuracy);

    void setResultsDataTable(StereoVisionApp::DataTable *newResultsDataTable);

protected:

    bool init() override;
    bool opt_step() override;
    bool std_step() override;
    bool writeResults() override;
    bool writeUncertainty() override;
    void cleanup() override;

    bool splitOptSteps() const override;

    bool _sparse;
    bool _compute_marginals;
    bool _not_first_step;

    ceres::Problem _problem;

    struct LandmarkPos {
        std::array<double, 3> position;
    };

    std::vector<LandmarkPos> _landmarksParameters;
    QMap<qint64, std::size_t> _landmarkParametersIndex;
    QMap<qint64, std::size_t> _additionalPointsParametersIndex;

    struct FramePoseParameters {
        double time;
        std::array<double, 3> rAxis; //rotation axis
        std::array<double, 3> wAxis; //rotation speed
        std::array<double, 3> t; //position
        std::array<double, 3> v; // speed
    };

    std::vector<FramePoseParameters> _frameParameters;
    QMap<qint64, std::size_t> _frameParametersIndex;

    struct LeverArmParameters {
        std::array<double, 3> rAxis; //rotation axis
        std::array<double, 3> t; //position
    };

    //lever arm represent transformation from body to camera frame.
    LeverArmParameters _initialLeverArm;
    LeverArmParameters _leverArm;

    struct CameraIntrinsicParameters {
        std::array<double, 1> fLen; //f
        std::array<double, 1> principalPoint; //pp
        std::array<double, 6> lateralDistortion; //as
        std::array<double, 6> frontalDistortion; //bs
    };

    CameraIntrinsicParameters _sensorParameters; //it is assume that the whole acquisition is done with a single sensor

    struct Gravity {
        std::array<double, 3> vec;
    };

    Gravity _gravity;

    BilSequenceAcquisitionData* _sequence;

    SequenceInfos _pseudo_sequence;

    GPSMeasurementInfos _gpsMeasurements;
    InitialOrientationInfos _initialOrientations;
    AccMeasurementInfos _accMeasurements;
    GyroMeasurementInfos _gyroMeasurements;
    AdditionalPointsInfos _additionalPoints;
    AdditionalViewsInfos _additionalViews; //allow to specify ties points from a data table rather than a bilsequence datablock.

    double _gpsAccuracy;
    double _gyroAccuracy;
    double _accAccuracy;
    double _tiepointsAccuracy;

    double _sensorWidth;

    StereoVisionApp::DataTable* _resultsDataTable;

    friend class CeresPushBroomSolverIterationCallback;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_CERESPUSHBROOMSOLVER_H
