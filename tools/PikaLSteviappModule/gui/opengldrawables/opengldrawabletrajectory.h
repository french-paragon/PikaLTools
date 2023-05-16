#ifndef PIKALTOOLS_OPENGLDRAWABLETRAJECTORY_H
#define PIKALTOOLS_OPENGLDRAWABLETRAJECTORY_H

#include <steviapp/gui/opengl3dsceneviewwidget.h>

#include "LibStevi/geometry/rotations.h"

namespace PikaLTools {

class OpenGlDrawableTrajectory : public StereoVisionApp::OpenGlDrawable
{
    Q_OBJECT
public:
    OpenGlDrawableTrajectory(StereoVisionApp::OpenGl3DSceneViewWidget* parent);

    void initializeGL();
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView);
    void clearViewRessources();

    void setTrajectory(std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& trajectory);
    void clearTrajectory();

    void setSceneScale(float newSceneScale);

protected:

    bool _has_data;
    bool _has_to_reset_gl_buffers;

    float _sceneScale;

    QOpenGLVertexArrayObject _scene_vao;
    QOpenGLVertexArrayObject _scene_ids_vao;

    QOpenGLBuffer _traj_buffer;

    QOpenGLShaderProgram* _trajectoryProgram;

    std::vector<GLfloat> _traj_pos;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_OPENGLDRAWABLETRAJECTORY_H
