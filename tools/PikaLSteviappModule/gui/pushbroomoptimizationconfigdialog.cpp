#include "pushbroomoptimizationconfigdialog.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>


namespace PikaLTools {

PushBroomOptimizationConfigDialog::PushBroomOptimizationConfigDialog(StereoVisionApp::Project* project, QWidget *parent) :
    QDialog(parent)
{
    QVBoxLayout* layout = new QVBoxLayout();

    //additional points
    //QVBoxLayout* pointsBoxLayout = new QVBoxLayout();

    //_pointsColsWidget = new DataColumnsSelectionWidget(project, this);
    //_pointsColsWidget->setVariableType(DataColumnsSelectionWidget::Position);
    //_pointsColsWidget->setIndexType(DataColumnsSelectionWidget::Index);
    //pointsBoxLayout->addWidget(_pointsColsWidget);

    //QGroupBox* pointsBox = new QGroupBox(tr("Points data"), this);
    //pointsBox->setLayout(pointsBoxLayout);

    //layout->addWidget(pointsBox);

    //points view
    //QVBoxLayout* pointsViewBoxLayout = new QVBoxLayout();

    //_pointsViewColsWidget = new DataColumnsSelectionWidget(project, this);
    //_pointsViewColsWidget->setVariableType(DataColumnsSelectionWidget::Position2D);
    //_pointsViewColsWidget->setIndexType(DataColumnsSelectionWidget::Index);
    //pointsViewBoxLayout->addWidget(_pointsViewColsWidget);

    //QGroupBox* pointsViewBox = new QGroupBox(tr("Points data"), this);
    //pointsViewBox->setLayout(pointsViewBoxLayout);

    //layout->addWidget(pointsViewBox);


    //gps
    QVBoxLayout* gpsMeasureBoxLayout = new QVBoxLayout();

    _gpsColsWidget = new DataColumnsSelectionWidget(project, this);
    _gpsColsWidget->setVariableType(DataColumnsSelectionWidget::Position);
    _gpsColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _gpsColsWidget->fixVariableType(true);
    gpsMeasureBoxLayout->addWidget(_gpsColsWidget);

    QGroupBox* gpsMeasureBox = new QGroupBox(tr("GPS data"), this);
    gpsMeasureBox->setLayout(gpsMeasureBoxLayout);

    layout->addWidget(gpsMeasureBox);

    //initial orientations
    QVBoxLayout* initialOrientationsBoxLayout = new QVBoxLayout();

    _initialOrientationColsWidget = new DataColumnsSelectionWidget(project, this);
    _initialOrientationColsWidget->setVariableType(DataColumnsSelectionWidget::Orientation);
    _initialOrientationColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _initialOrientationColsWidget->fixVariableType(true);
    initialOrientationsBoxLayout->addWidget(_initialOrientationColsWidget);

    QGroupBox* initialAlignementBox = new QGroupBox(tr("Initial alignement data"), this);
    initialAlignementBox->setLayout(initialOrientationsBoxLayout);

    layout->addWidget(initialAlignementBox);

    //gyro
    QVBoxLayout* gyroMeasureBoxLayout = new QVBoxLayout();

    _gyroColsWidget = new DataColumnsSelectionWidget(project, this);
    _gyroColsWidget->setVariableType(DataColumnsSelectionWidget::AngularSpeed);
    _gyroColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _gyroColsWidget->fixVariableType(true);
    gyroMeasureBoxLayout->addWidget(_gyroColsWidget);

    QGroupBox* gyroMeasureBox = new QGroupBox(tr("Gyroscope data"), this);
    gyroMeasureBox->setLayout(gyroMeasureBoxLayout);

    layout->addWidget(gyroMeasureBox);

    //accelerometer
    QVBoxLayout* accMeasureBoxLayout = new QVBoxLayout();

    _accColsWidget = new DataColumnsSelectionWidget(project, this);
    _accColsWidget->setVariableType(DataColumnsSelectionWidget::Acceleration);
    _accColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _accColsWidget->fixVariableType(true);
    accMeasureBoxLayout->addWidget(_accColsWidget);

    QGroupBox* accMeasureBox = new QGroupBox(tr("Accelerometer data"), this);
    accMeasureBox->setLayout(accMeasureBoxLayout);

    layout->addWidget(accMeasureBox);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                         | QDialogButtonBox::Cancel);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    layout->addWidget(buttonBox);

    setLayout(layout);

    resize(sizeHint());

}

/*DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsPointsColsSelectionInfos() const {
    return _pointsColsWidget->getColumnsSelection();
}
DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsPointsViewColsSelectionInfos() const {
    return _pointsViewColsWidget->getColumnsSelection();
}*/

DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsInitialOrientationColsSelectionInfos() const {
    return _initialOrientationColsWidget->getColumnsSelection();
}
DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsGpsColsSelectionInfos() const {
    return _gpsColsWidget->getColumnsSelection();
}
DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsGyroColsSelectionInfos() const {
    return _gyroColsWidget->getColumnsSelection();
}
DataColumnsSelectionWidget::ColumnSelectionInfos PushBroomOptimizationConfigDialog::getsAccColsSelectionInfos() const {
    return _accColsWidget->getColumnsSelection();
}

} // namespace PikaLTools
