#include "simulatepushbroomtiepointsoptiondialog.h"

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QDialogButtonBox>

namespace PikaLTools {

SimulatePushBroomTiePointsOptionDialog::SimulatePushBroomTiePointsOptionDialog(StereoVisionApp::Project *project, QWidget *parent) :
    QDialog(parent)
{
    QVBoxLayout* layout = new QVBoxLayout();

    //Basic options
    QFormLayout* optionsLayout = new QFormLayout();

    _n_points = new QSpinBox(this);
    _n_points->setMinimum(1);
    _n_points->setMaximum(10000);
    _n_points->setValue(500);

    _n_bands = new QSpinBox(this);
    _n_bands->setMinimum(1);
    _n_bands->setMaximum(100000);
    _n_bands->setValue(24000);

    _n_pixels = new QSpinBox(this);
    _n_pixels->setMinimum(1);
    _n_pixels->setMaximum(10000);
    _n_pixels->setValue(700);

    _fLen = new QDoubleSpinBox(this);
    _fLen->setMinimum(0.1);
    _fLen->setMaximum(10000);
    _fLen->setSingleStep(0.1);
    _fLen->setDecimals(1);
    _fLen->setValue(600);

    _random_seed = new QSpinBox(this);
    _random_seed->setMinimum(-1);
    _random_seed->setMaximum(9999999);
    _random_seed->setValue(4269);

    optionsLayout->addRow(tr("# ground points"), _n_points);
    optionsLayout->addRow(tr("# bands"), _n_bands);
    optionsLayout->addRow(tr("# pixels"), _n_pixels);
    optionsLayout->addRow(tr("focal lenght"), _fLen);
    optionsLayout->addRow(tr("seed"), _random_seed);

    QGroupBox* optionsBox = new QGroupBox(tr("Options"), this);
    optionsBox->setLayout(optionsLayout);

    layout->addWidget(optionsBox);

    //position
    QVBoxLayout* positionMeasureBoxLayout = new QVBoxLayout();

    _positionColsWidget = new DataColumnsSelectionWidget(project, this);
    _positionColsWidget->setVariableType(DataColumnsSelectionWidget::Position);
    _positionColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _positionColsWidget->fixVariableType(true);
    positionMeasureBoxLayout->addWidget(_positionColsWidget);

    QGroupBox* positionMeasureBox = new QGroupBox(tr("Ground truth positions"), this);
    positionMeasureBox->setLayout(positionMeasureBoxLayout);

    layout->addWidget(positionMeasureBox);

    //initial orientations
    QVBoxLayout* orientationsBoxLayout = new QVBoxLayout();

    _orientationColsWidget = new DataColumnsSelectionWidget(project, this);
    _orientationColsWidget->setVariableType(DataColumnsSelectionWidget::Orientation);
    _orientationColsWidget->setIndexType(DataColumnsSelectionWidget::Time);
    _orientationColsWidget->fixVariableType(true);
    orientationsBoxLayout->addWidget(_orientationColsWidget);

    QGroupBox* orientationBox = new QGroupBox(tr("Initial alignement data"), this);
    orientationBox->setLayout(orientationsBoxLayout);

    layout->addWidget(orientationBox);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                         | QDialogButtonBox::Cancel);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    layout->addWidget(buttonBox);

    setLayout(layout);

    resize(sizeHint());

}

DataColumnsSelectionWidget::ColumnSelectionInfos SimulatePushBroomTiePointsOptionDialog::getsOrientationColsSelectionInfos() const {
    return _orientationColsWidget->getColumnsSelection();
}
DataColumnsSelectionWidget::ColumnSelectionInfos SimulatePushBroomTiePointsOptionDialog::getsPositionColsSelectionInfos() const {
    return _positionColsWidget->getColumnsSelection();
}

int SimulatePushBroomTiePointsOptionDialog::nBands() const {
    return _n_bands->value();
}

int SimulatePushBroomTiePointsOptionDialog::nPoints() const {
    return _n_points->value();
}

int SimulatePushBroomTiePointsOptionDialog::nPixels() const {
    return _n_pixels->value();
}

double SimulatePushBroomTiePointsOptionDialog::focalLength() const {
    return _fLen->value();
}

int SimulatePushBroomTiePointsOptionDialog::randomSeed() const {
    return _random_seed->value();
}

} // namespace PikaLTools
