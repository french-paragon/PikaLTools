#include "datacolumnsselectionwidget.h"

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/datatable.h>

#include <QComboBox>
#include <QLabel>
#include <QGridLayout>
#include <QSpacerItem>

namespace PikaLTools {

DataColumnsSelectionWidget::DataColumnsSelectionWidget(StereoVisionApp::Project* project, QWidget *parent)
    : QWidget{parent},
      _currentProject(project),
      _dataTable(nullptr)
{

    _variableTypeLabel = new QLabel(tr("Variable type:"), this);
    _variableTypeSelectionBox = new QComboBox(this);

    _variableTypeSelectionBox->addItem(tr("Position"), QVariant(static_cast<int>(Position)));
    _variableTypeSelectionBox->addItem(tr("Point"), QVariant(static_cast<int>(Position2D)));
    _variableTypeSelectionBox->addItem(tr("Speed"), QVariant(static_cast<int>(Speed)));
    _variableTypeSelectionBox->addItem(tr("Acceleration"), QVariant(static_cast<int>(Acceleration)));

    _variableTypeSelectionBox->addItem(tr("Orientation"), QVariant(static_cast<int>(Orientation)));
    _variableTypeSelectionBox->addItem(tr("Angular Speed"), QVariant(static_cast<int>(AngularSpeed)));
    _variableTypeSelectionBox->addItem(tr("Angular Acceleration"), QVariant(static_cast<int>(AngularAcceleration)));

    _angleRepLabel = new QLabel(this);
    _angleRepresentationSelectionBox = new QComboBox(this);

    _angleRepresentationSelectionBox->addItem(tr("Axis angle"), QVariant(static_cast<int>(AxisAngle)));
    _angleRepresentationSelectionBox->addItem(tr("Quaternion"), QVariant(static_cast<int>(Quaternion)));
    _angleRepresentationSelectionBox->addItem(tr("Euler Angles (XYZ)"), QVariant(static_cast<int>(EulerXYZ)));

    _dataTableSelectionComboBox = new QComboBox(this);


    if (_currentProject == nullptr) {
        return;
    }
    _dataTableLabel = new QLabel(tr("Data table:"), this);

    QVector<qint64> dataTablesIdxs = _currentProject->getIdsByClass(StereoVisionApp::DataTable::staticMetaObject.className());

    for (qint64 id : dataTablesIdxs) {
        StereoVisionApp::DataBlock* datablock = _currentProject->getById(id);
        _dataTableSelectionComboBox->addItem(datablock->objectName(), id);
    }

    connect(_dataTableSelectionComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &DataColumnsSelectionWidget::onDataTableSelectionChanged);

    connect(_variableTypeSelectionBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &DataColumnsSelectionWidget::onVariableTypeChanged);

    connect(_angleRepresentationSelectionBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &DataColumnsSelectionWidget::onAngleRepresentationTypeChanged);

    _layout = new QGridLayout();

    _layout->addWidget(_dataTableLabel, 0,0);
    _layout->addWidget(_dataTableSelectionComboBox, 1,0);

    _dataTableSpacer = new QSpacerItem(5,1);
    _layout->addItem(_dataTableSpacer, 0, 1);

    _layout->addWidget(_variableTypeLabel, 0,2);
    _layout->addWidget(_variableTypeSelectionBox, 1,2);

    _variableTypeSpacer = new QSpacerItem(5,1);
    _layout->addItem(_variableTypeSpacer, 0, 3);

    _layout->addWidget(_angleRepLabel, 0,4);
    _layout->addWidget(_angleRepresentationSelectionBox, 1,4);

    _angleRepSpacer = new QSpacerItem(5,1);
    _layout->addItem(_angleRepSpacer, 0, 5);

    _colIndexingLabel = new QLabel(this);
    _colIndexingSelectionComboBox = new QComboBox(this);
    _layout->addWidget(_colIndexingLabel, 0,6);
    _layout->addWidget(_colIndexingSelectionComboBox, 1,6);

    _colIndexingSpacer = new QSpacerItem(5,1);
    _layout->addItem(_colIndexingSpacer, 0, 7);

    _col0Label = new QLabel(this);
    _col0SelectionComboBox = new QComboBox(this);
    _layout->addWidget(_col0Label, 0,8);
    _layout->addWidget(_col0SelectionComboBox, 1,8);

    _col0Spacer = new QSpacerItem(5,1);
    _layout->addItem(_col0Spacer, 0, 9);

    _col1Label = new QLabel(this);
    _col1SelectionComboBox = new QComboBox(this);
    _layout->addWidget(_col1Label, 0,10);
    _layout->addWidget(_col1SelectionComboBox, 1,10);

    _col1Spacer = new QSpacerItem(5,1);
    _layout->addItem(_col1Spacer, 0, 11);

    _col2Label = new QLabel(this);
    _col2SelectionComboBox = new QComboBox(this);
    _layout->addWidget(_col2Label, 0,12);
    _layout->addWidget(_col2SelectionComboBox, 1,12);

    _col2Spacer = new QSpacerItem(5,1);
    _layout->addItem(_col2Spacer, 0, 13);

    _col3Label = new QLabel(this);
    _col3SelectionComboBox = new QComboBox(this);
    _layout->addWidget(_col3Label, 0,14);
    _layout->addWidget(_col3SelectionComboBox, 1,14);

    _layout->setColumnStretch(0, 3);
    _layout->setColumnStretch(1, 0);
    _layout->setColumnStretch(2, 2);
    _layout->setColumnStretch(3, 0);
    _layout->setColumnStretch(4, 2);
    _layout->setColumnStretch(5, 0);
    _layout->setColumnStretch(6, 0);
    _layout->setColumnStretch(7, 0);
    _layout->setColumnStretch(8, 0);
    _layout->setColumnStretch(9, 0);
    _layout->setColumnStretch(10, 0);
    _layout->setColumnStretch(11, 0);
    _layout->setColumnStretch(12, 0);
    _layout->setColumnStretch(13, 0);
    _layout->setColumnStretch(14, 0);

    _layout->setVerticalSpacing(5);
    _layout->setMargin(5);

    setLayout(_layout);

    onDataTableSelectionChanged();
    onVariableTypeChanged();
    setIndexType(Index);

}

void DataColumnsSelectionWidget::fixVariableType(bool fixedVariableType) {
    if (fixedVariableType) {
        _variableTypeLabel->hide();
        _variableTypeSelectionBox->hide();
        _variableTypeSpacer->changeSize(0,1);
        _layout->setColumnStretch(2, 0);
    } else {
        _variableTypeLabel->show();
        _variableTypeSelectionBox->show();
        _variableTypeSpacer->changeSize(5,1);
        _layout->setColumnStretch(2, 2);
    }
}
bool DataColumnsSelectionWidget::variableTypeIsFixed() const {
    return _variableTypeSelectionBox->isHidden();
}

void DataColumnsSelectionWidget::setVariableType(VariableType varType) {
    int idx = _variableTypeSelectionBox->findData(static_cast<int>(varType));
    _variableTypeSelectionBox->setCurrentIndex(idx);
}

DataColumnsSelectionWidget::VariableType DataColumnsSelectionWidget::variableType() const {

    return static_cast<VariableType>(_variableTypeSelectionBox->currentData().toInt());
}

void DataColumnsSelectionWidget::setAngleRepresentaitonType(AngleRepresentation varType) {

    int idx = _angleRepresentationSelectionBox->findData(static_cast<int>(varType));
    _angleRepresentationSelectionBox->setCurrentIndex(idx);

}
DataColumnsSelectionWidget::AngleRepresentation DataColumnsSelectionWidget::angleRepresentationType() const {

    return static_cast<AngleRepresentation>(_angleRepresentationSelectionBox->currentData().toInt());
}

void DataColumnsSelectionWidget::setIndexType(IndexType type) {

    switch (type) {
    case Index:
        _colIndexingLabel->setText(tr("Index"));
        break;
    case Time:
        _colIndexingLabel->setText(tr("Time"));
        break;
    }

}

DataColumnsSelectionWidget::ColumnSelectionInfos DataColumnsSelectionWidget::getColumnsSelection() const {

    int nCols = 3;

    if (variableType() == Orientation or
            variableType() == AngularSpeed or
            variableType() == AngularAcceleration) {
        if (angleRepresentationType() == Quaternion) {
            nCols = 4;
        }
    }

    if (variableType() == Position2D) {
        nCols = 2;
    }

    QVector<QString> colsNames(nCols);

    if (nCols == 4) {

        colsNames[0] = _col0SelectionComboBox->currentText();
        colsNames[1] = _col1SelectionComboBox->currentText();
        colsNames[2] = _col2SelectionComboBox->currentText();
        colsNames[3] = _col3SelectionComboBox->currentText();

    } else if (nCols == 3) {

        colsNames[0] = _col1SelectionComboBox->currentText();
        colsNames[1] = _col2SelectionComboBox->currentText();
        colsNames[2] = _col3SelectionComboBox->currentText();

    } else {

        colsNames[0] = _col1SelectionComboBox->currentText();
        colsNames[1] = _col2SelectionComboBox->currentText();

    }

    return {_dataTable, variableType(), angleRepresentationType(), _colIndexingSelectionComboBox->currentText(), colsNames};


}

void DataColumnsSelectionWidget::onDataTableSelectionChanged() {

    _colIndexingSelectionComboBox->clear();
    _col1SelectionComboBox->clear();
    _col2SelectionComboBox->clear();
    _col3SelectionComboBox->clear();
    _col0SelectionComboBox->clear();

    if (_currentProject == nullptr) {
        return;
    }

    qint64 id = _dataTableSelectionComboBox->currentData().toInt();
    _dataTable = _currentProject->getDataBlock<StereoVisionApp::DataTable>(id);

    QVector<QString> colsNames = _dataTable->columns();

    for (QString const& str : colsNames) {
        _colIndexingSelectionComboBox->addItem(str);
        _col1SelectionComboBox->addItem(str);
        _col2SelectionComboBox->addItem(str);
        _col3SelectionComboBox->addItem(str);
        _col0SelectionComboBox->addItem(str);
    }
}
void DataColumnsSelectionWidget::onVariableTypeChanged() {

    VariableType type = variableType();

    _col3Label->setVisible(true);
    _col3SelectionComboBox->setVisible(true);

    _layout->setColumnStretch(4, 0);

    switch (type) {
    case Position:
        _col1Label->setText(tr("Position X"));
        _col2Label->setText(tr("Position Y"));
        _col3Label->setText(tr("Position Z"));

        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);

        _angleRepLabel->setVisible(false);
        _angleRepresentationSelectionBox->setVisible(false);
        _angleRepSpacer->changeSize(0,1);
        break;
    case Position2D:
        _col1Label->setText(tr("Position X"));
        _col2Label->setText(tr("Position Y"));

        _col3Label->setVisible(false);
        _col3SelectionComboBox->setVisible(false);

        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);

        _angleRepLabel->setVisible(false);
        _angleRepresentationSelectionBox->setVisible(false);
        _angleRepSpacer->changeSize(0,1);
        break;

    case Speed:
        _col1Label->setText(tr("Speed X"));
        _col2Label->setText(tr("Speed Y"));
        _col3Label->setText(tr("Speed Z"));

        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);

        _angleRepLabel->setVisible(false);
        _angleRepresentationSelectionBox->setVisible(false);
        _angleRepSpacer->changeSize(0,1);
        break;
    case Acceleration:
        _col1Label->setText(tr("Specific force X"));
        _col2Label->setText(tr("Specific force Y"));
        _col3Label->setText(tr("Specific force Z"));

        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);

        _angleRepLabel->setVisible(false);
        _angleRepresentationSelectionBox->setVisible(false);
        _angleRepSpacer->changeSize(0,1);
        break;
    case Orientation:
    case AngularSpeed:
    case AngularAcceleration:
        onAngleRepresentationTypeChanged();
        break;
    }

}
void DataColumnsSelectionWidget::onAngleRepresentationTypeChanged() {

    VariableType type = variableType();
    AngleRepresentation angleRep = angleRepresentationType();

    switch (type) {
    case Position:
    case Position2D:
    case Speed:
    case Acceleration:
        return;
    case Orientation:
        _angleRepLabel->setText(tr("Angular acc type"));
        _layout->setColumnStretch(4, 2);
        break;
    case AngularSpeed:
        _angleRepLabel->setText(tr("Angular speed type"));
        _layout->setColumnStretch(4, 2);
        break;
    case AngularAcceleration:
        _angleRepLabel->setText(tr("Angular acc type"));
        _layout->setColumnStretch(4, 2);
        break;
    }

    _angleRepLabel->setVisible(true);
    _angleRepresentationSelectionBox->setVisible(true);
    _angleRepSpacer->changeSize(5,1);

    switch (angleRep) {
    case AxisAngle:
        _col1Label->setText(tr("Angle Axis X"));
        _col2Label->setText(tr("Angle Axis Y"));
        _col3Label->setText(tr("Angle Axis Z"));
        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);
        break;
    case Quaternion:
        _col1Label->setText(tr("Quat X"));
        _col2Label->setText(tr("Quat Y"));
        _col3Label->setText(tr("Quat Z"));
        _col0Label->setText(tr("Quat W"));
        _col0Label->setVisible(true);
        _col0SelectionComboBox->setVisible(true);
        _col0Spacer->changeSize(5,1);
        break;
    case EulerXYZ:
        _col1Label->setText(tr("Euler X"));
        _col2Label->setText(tr("Euler Y"));
        _col3Label->setText(tr("Euler Z"));
        _col0Label->setVisible(false);
        _col0SelectionComboBox->setVisible(false);
        _col0Spacer->changeSize(0,1);
        break;
    }
}

} // namespace PikaLTools
