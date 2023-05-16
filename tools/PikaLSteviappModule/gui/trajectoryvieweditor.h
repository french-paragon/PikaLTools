#ifndef PIKALTOOLS_TRAJECTORYVIEWEDITOR_H
#define PIKALTOOLS_TRAJECTORYVIEWEDITOR_H

#include <steviapp/gui/editor.h>

#include "LibStevi/geometry/rotations.h"

namespace StereoVisionApp {

class OpenGl3DSceneViewWidget;
class OpenGlDrawableSceneGrid;

}

namespace PikaLTools {

class OpenGlDrawableTrajectory;

class TrajectoryViewEditor : public StereoVisionApp::Editor
{
    Q_OBJECT
public:
    TrajectoryViewEditor(QWidget* parent = nullptr);

    void setTrajectory(std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& trajectory);
    void clearTrajectory();

protected:

    StereoVisionApp::OpenGl3DSceneViewWidget* _viewScene;

    StereoVisionApp::OpenGlDrawableSceneGrid* _grid;
    OpenGlDrawableTrajectory* _drawableTrajectory;
};

class TrajectoryViewEditorFactory : public StereoVisionApp::EditorFactory
{
    Q_OBJECT
public:

    explicit TrajectoryViewEditorFactory(QObject *parent = nullptr);

    QString TypeDescrName() const override;
    QString itemClassName() const override;
    StereoVisionApp::Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_TRAJECTORYVIEWEDITOR_H
