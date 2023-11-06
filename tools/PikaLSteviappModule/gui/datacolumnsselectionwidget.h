#ifndef PIKALTOOLS_DATACOLUMNSSELECTIONWIDGET_H
#define PIKALTOOLS_DATACOLUMNSSELECTIONWIDGET_H

#include <QWidget>

class QComboBox;
class QLabel;
class QSpacerItem;
class QGridLayout;

namespace StereoVisionApp {
    class Project;
    class DataTable;
}

namespace PikaLTools {

class DataColumnsSelectionWidget : public QWidget
{
    Q_OBJECT
public:

    enum IndexType {
        Index,
        Time
    };

    enum VariableType {
        Position,
        Position2D,
        Speed,
        Acceleration,
        Orientation,
        AngularSpeed,
        AngularAcceleration
    };

    enum AngleRepresentation {
        AxisAngle,
        Quaternion,
        EulerXYZ
    };


    struct ColumnSelectionInfos {
        StereoVisionApp::DataTable* dataTable;
        VariableType variableType;
        AngleRepresentation angleRepType;
        QString indexingCol;
        QVector<QString> columnsNames;
    };

    explicit DataColumnsSelectionWidget(StereoVisionApp::Project* project, QWidget *parent = nullptr);

    void fixVariableType(bool fixedVariableType);
    bool variableTypeIsFixed() const;

    void setVariableType(VariableType varType);
    VariableType variableType() const;

    void setAngleRepresentaitonType(AngleRepresentation varType);
    AngleRepresentation angleRepresentationType() const;

    void setIndexType(IndexType type);

    ColumnSelectionInfos getColumnsSelection() const;

Q_SIGNALS:

protected:

    StereoVisionApp::Project* _currentProject;
    StereoVisionApp::DataTable* _dataTable;

    QLabel* _dataTableLabel;
    QComboBox* _dataTableSelectionComboBox;
    QSpacerItem* _dataTableSpacer;

    QLabel* _variableTypeLabel;
    QComboBox* _variableTypeSelectionBox;
    QSpacerItem* _variableTypeSpacer;

    QLabel* _angleRepLabel;
    QComboBox* _angleRepresentationSelectionBox;
    QSpacerItem* _angleRepSpacer;

    QLabel* _colIndexingLabel;
    QComboBox* _colIndexingSelectionComboBox;
    QSpacerItem* _colIndexingSpacer;

    QLabel* _col1Label;
    QComboBox* _col1SelectionComboBox;
    QSpacerItem* _col1Spacer;

    QLabel* _col2Label;
    QComboBox* _col2SelectionComboBox;
    QSpacerItem* _col2Spacer;

    QLabel* _col3Label;
    QComboBox* _col3SelectionComboBox;

    QLabel* _col0Label;
    QComboBox* _col0SelectionComboBox;
    QSpacerItem* _col0Spacer;

    QGridLayout* _layout;

    void onDataTableSelectionChanged();
    void onVariableTypeChanged();
    void onAngleRepresentationTypeChanged();

};

} // namespace PikaLTools

#endif // PIKALTOOLS_DATACOLUMNSSELECTIONWIDGET_H
