#ifndef PIKALTOOLS_PUSHBROOMOPTIMIZATIONCONFIGDIALOG_H
#define PIKALTOOLS_PUSHBROOMOPTIMIZATIONCONFIGDIALOG_H

#include <QDialog>

#include "./datacolumnsselectionwidget.h"

namespace StereoVisionApp {
    class Project;
    class DataTable;
}

namespace PikaLTools {

class PushBroomOptimizationConfigDialog : public QDialog
{
public:
    PushBroomOptimizationConfigDialog(StereoVisionApp::Project* project, QWidget *parent = nullptr);

    //DataColumnsSelectionWidget::ColumnSelectionInfos getsPointsColsSelectionInfos() const;
    //DataColumnsSelectionWidget::ColumnSelectionInfos getsPointsViewColsSelectionInfos() const;
    DataColumnsSelectionWidget::ColumnSelectionInfos getsInitialOrientationColsSelectionInfos() const;
    DataColumnsSelectionWidget::ColumnSelectionInfos getsGpsColsSelectionInfos() const;
    DataColumnsSelectionWidget::ColumnSelectionInfos getsGyroColsSelectionInfos() const;
    DataColumnsSelectionWidget::ColumnSelectionInfos getsAccColsSelectionInfos() const;

protected:

    //DataColumnsSelectionWidget* _pointsColsWidget;
    //DataColumnsSelectionWidget* _pointsViewColsWidget;
    DataColumnsSelectionWidget* _initialOrientationColsWidget;
    DataColumnsSelectionWidget* _gpsColsWidget;
    DataColumnsSelectionWidget* _gyroColsWidget;
    DataColumnsSelectionWidget* _accColsWidget;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_PUSHBROOMOPTIMIZATIONCONFIGDIALOG_H
