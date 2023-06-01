#ifndef PIKALTOOLS_OPENGLDRAWABLEDTM_H
#define PIKALTOOLS_OPENGLDRAWABLEDTM_H

#include <steviapp/gui/opengl3dsceneviewwidget.h>

#include "LibStevi/geometry/rotations.h"


namespace PikaLTools {

class OpenGlDrawableDtm : public StereoVisionApp::OpenGlDrawable
{
public:
    OpenGlDrawableDtm(StereoVisionApp::OpenGl3DSceneViewWidget* parent);

    void initializeGL();
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView);
    void clearViewRessources();

    void setDtm(const Multidim::Array<float, 3> &points, Multidim::Array<bool,2> const* validPixels = nullptr);
    void clearDtm();

    void setSceneScale(float newSceneScale);

protected:

    bool _has_data;
    bool _has_to_reset_gl_buffers;

    float _sceneScale;

    QOpenGLVertexArrayObject _scene_vao;

    QOpenGLBuffer _pos_buffer;
    QOpenGLBuffer _normal_buffer;
    QOpenGLBuffer _idx_buffer;

    QOpenGLShaderProgram* _dtmProgram;

    std::vector<GLfloat> _vertices_pos;
    std::vector<GLfloat> _vertices_normals;
    std::vector<GLuint> _faces_idxs;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_OPENGLDRAWABLEDTM_H
