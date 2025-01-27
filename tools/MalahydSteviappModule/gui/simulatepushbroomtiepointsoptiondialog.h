#ifndef PIKALTOOLS_SIMULATEPUSHBROOMTIEPOINTSOPTIONDIALOG_H
#define PIKALTOOLS_SIMULATEPUSHBROOMTIEPOINTSOPTIONDIALOG_H


#include <QDialog>

#include "./datacolumnsselectionwidget.h"

class QSpinBox;
class QDoubleSpinBox;

namespace PikaLTools {

class SimulatePushBroomTiePointsOptionDialog : public QDialog
{
public:
    SimulatePushBroomTiePointsOptionDialog(StereoVisionApp::Project* project, QWidget *parent = nullptr);

    DataColumnsSelectionWidget::ColumnSelectionInfos getsOrientationColsSelectionInfos() const;
    DataColumnsSelectionWidget::ColumnSelectionInfos getsPositionColsSelectionInfos() const;

    int nBands() const;
    int nPoints() const;
    int nPixels() const;

    double focalLength() const;

    int randomSeed() const;

protected:

    QSpinBox* _n_bands;
    QSpinBox* _n_points;
    QSpinBox* _n_pixels;
    QSpinBox* _random_seed;

    QDoubleSpinBox* _fLen;

    DataColumnsSelectionWidget* _orientationColsWidget;
    DataColumnsSelectionWidget* _positionColsWidget;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_SIMULATEPUSHBROOMTIEPOINTSOPTIONDIALOG_H
