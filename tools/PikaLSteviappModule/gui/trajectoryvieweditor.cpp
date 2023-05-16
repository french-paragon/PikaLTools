#include "trajectoryvieweditor.h"

#include <steviapp/gui/opengl3dsceneviewwidget.h>
#include <steviapp/gui/openGlDrawables/opengldrawablescenegrid.h>

#include "opengldrawables/opengldrawabletrajectory.h"

#include <QVBoxLayout>

namespace PikaLTools {

TrajectoryViewEditor::TrajectoryViewEditor(QWidget *parent) :
    StereoVisionApp::Editor(parent)
{
    _viewScene = new StereoVisionApp::OpenGl3DSceneViewWidget(this);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(_viewScene);
    layout->setMargin(0);
    layout->setSpacing(0);

    _grid = new StereoVisionApp::OpenGlDrawableSceneGrid(_viewScene);
    _grid->setGridDistance(10.);
    _grid->setGridSplits(50);
    _viewScene->addDrawable(_grid);

    _drawableTrajectory = new OpenGlDrawableTrajectory(_viewScene);
    _viewScene->addDrawable(_drawableTrajectory);
}

void TrajectoryViewEditor::setTrajectory(std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& trajectory) {

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
}
void TrajectoryViewEditor::clearTrajectory() {
    _drawableTrajectory->clearTrajectory();
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
