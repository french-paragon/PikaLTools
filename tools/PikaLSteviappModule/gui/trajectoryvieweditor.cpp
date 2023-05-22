#include "trajectoryvieweditor.h"

#include "datablocks/bilacquisitiondata.h"

#include <steviapp/gui/opengl3dsceneviewwidget.h>
#include <steviapp/gui/openGlDrawables/opengldrawablescenegrid.h>

#include "opengldrawables/opengldrawabletrajectory.h"

#include <QVBoxLayout>
#include <QHBoxLayout>

#include <QLabel>
#include <QSpinBox>

namespace PikaLTools {

TrajectoryViewEditor::TrajectoryViewEditor(QWidget *parent) :
    StereoVisionApp::Editor(parent)
{
    _viewScene = new StereoVisionApp::OpenGl3DSceneViewWidget(this);

    QHBoxLayout* toolsLayout = new QHBoxLayout();

    QLabel* startLineLabel = new QLabel(this);
    startLineLabel->setText(tr("Start line:"));

    _startLineSpinBox = new QSpinBox(this);
    _startLineSpinBox->setMinimum(-1);
    _startLineSpinBox->setMaximum(1);
    _startLineSpinBox->setValue(0);

    connect(_startLineSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &TrajectoryViewEditor::setStartLine);

    QLabel* endLineLabel = new QLabel(this);
    endLineLabel->setText(tr("End line:"));

    _endLineSpinBox = new QSpinBox(this);
    _endLineSpinBox->setMinimum(-1);
    _endLineSpinBox->setMaximum(1000);
    _endLineSpinBox->setValue(1000);

    connect(_endLineSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &TrajectoryViewEditor::setEndLine);

    toolsLayout->addWidget(startLineLabel);
    toolsLayout->addWidget(_startLineSpinBox);
    toolsLayout->addSpacing(5);

    toolsLayout->addWidget(endLineLabel);
    toolsLayout->addWidget(_endLineSpinBox);
    toolsLayout->addSpacing(5);

    toolsLayout->addStretch();

    toolsLayout->setMargin(5);
    toolsLayout->setSpacing(3);

    QVBoxLayout* layout = new QVBoxLayout();

    layout->addLayout(toolsLayout);

    layout->addWidget(_viewScene);
    layout->setMargin(0);
    layout->setSpacing(0);

    _grid = new StereoVisionApp::OpenGlDrawableSceneGrid(_viewScene);
    _grid->setGridDistance(10.);
    _grid->setGridSplits(50);
    _viewScene->addDrawable(_grid);

    _drawableTrajectory = new OpenGlDrawableTrajectory(_viewScene);
    _viewScene->addDrawable(_drawableTrajectory);

    setLayout(layout);
}

void TrajectoryViewEditor::setStartLine(int bilLine) {

    if (_startLineSpinBox->value() != bilLine) {
        _startLineSpinBox->setValue(bilLine);
    }

    _drawableTrajectory->setSegmentStart(bil2lcfLineId(bilLine));
}
void TrajectoryViewEditor::setEndLine(int bilLine) {

    if (_endLineSpinBox->value() != bilLine) {
        _endLineSpinBox->setValue(bilLine);
    }

    _drawableTrajectory->setSegmentEnd(bil2lcfLineId(bilLine));
}

void TrajectoryViewEditor::setTrajectory(const BilSequenceAcquisitionData &bilSequence) {

    std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> trajectory = bilSequence.localTrajectory();

    float minXy = -1;
    float maxXy = 1;

    for (int i = 0; i < trajectory.size(); i++) {
        Eigen::Vector3f pos = trajectory[i].t;

        if (pos.x() < minXy) {
            minXy = pos.x();
        }

        if (pos.x() > maxXy) {
            maxXy = pos.x();
        }

        if (pos.y() < minXy) {
            minXy = pos.x();
        }

        if (pos.y() > maxXy) {
            maxXy = pos.x();
        }
    }

    float maxDist = std::max(std::abs(minXy), maxXy);
    _drawableTrajectory->setSceneScale(12./maxDist);

    _drawableTrajectory->setTrajectory(trajectory);

    _bil2lcfLines.clear();

    int nLines = 0;
    int nLcfLines = 0;

    _bil2lcfLines.insert(nLines, nLcfLines);

    for (BilSequenceAcquisitionData::BilAcquisitionData const& bil : bilSequence.getBilInfos()) {

        nLines += bil.getNLines();
        nLcfLines += bil.getLcfNLines();

        _bil2lcfLines.insert(nLines, nLcfLines);
    }

    _startLineSpinBox->setMaximum(trajectory.size());
    _endLineSpinBox->setMaximum(trajectory.size());

    setStartLine(_startLineSpinBox->value());
    setEndLine(_endLineSpinBox->value());

}
void TrajectoryViewEditor::clearTrajectory() {
    _drawableTrajectory->clearTrajectory();

    _startLineSpinBox->setMaximum(1);
    _startLineSpinBox->setValue(0);
    _endLineSpinBox->setMaximum(1000);
    _endLineSpinBox->setValue(1000);
}

TrajectoryViewEditorFactory::TrajectoryViewEditorFactory(QObject *parent) :
    StereoVisionApp::EditorFactory(parent)
{

}

QString TrajectoryViewEditorFactory::TypeDescrName() const {
    return tr("PikaL lcf trajectory viewer");
}
QString TrajectoryViewEditorFactory::itemClassName() const {
    return TrajectoryViewEditor::staticMetaObject.className();
}
StereoVisionApp::Editor* TrajectoryViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new TrajectoryViewEditor(parent);
}

} // namespace PikaLTools
