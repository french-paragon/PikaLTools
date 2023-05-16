#include "opengldrawabletrajectory.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

namespace PikaLTools {

OpenGlDrawableTrajectory::OpenGlDrawableTrajectory(StereoVisionApp::OpenGl3DSceneViewWidget* parent) :
    StereoVisionApp::OpenGlDrawable(parent)
{

}

void OpenGlDrawableTrajectory::initializeGL() {

    _scene_vao.create();

    _traj_buffer.create();
    _traj_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _trajectoryProgram = new QOpenGLShaderProgram();
    _trajectoryProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/glShaders/trajectoryViewerLine.vert");
    _trajectoryProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/glShaders/trajectoryViewerLine.frag");

    _trajectoryProgram->link();

}
void OpenGlDrawableTrajectory::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_has_data) {

        int vertexLocation;

        if (!_traj_pos.empty()) {

            _trajectoryProgram->bind();
            _scene_vao.bind();
            _traj_buffer.bind();

            if (_has_to_reset_gl_buffers) {
                _traj_buffer.allocate(_traj_pos.data(), _traj_pos.size()*sizeof (GLfloat));
                _has_to_reset_gl_buffers = false;
            }

            vertexLocation = _trajectoryProgram->attributeLocation("in_location");
            _trajectoryProgram->enableAttributeArray(vertexLocation);
            _trajectoryProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

            _trajectoryProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
            _trajectoryProgram->setUniformValue("sceneScale", _sceneScale);

            f->glDrawArrays(GL_LINE_STRIP, 0, _traj_pos.size()/3);

            _scene_vao.release();
            _traj_buffer.release();

            _trajectoryProgram->disableAttributeArray(vertexLocation);
            _trajectoryProgram->release();

        }
    }
}

void OpenGlDrawableTrajectory::clearViewRessources() {

    if (_trajectoryProgram != nullptr) {
        delete _trajectoryProgram;
    }

    if (_traj_buffer.isCreated()) {
        _traj_buffer.destroy();
    }
    _scene_vao.destroy();
}

/*!
 * \brief OpenGlDrawableTrajectory::setTrajectory set the trajectory that is displayed
 * \param trajectory the trajectory, as a series of body to world transforms
 */
void OpenGlDrawableTrajectory::setTrajectory(std::vector<StereoVision::Geometry::ShapePreservingTransform<float>> const& trajectory) {
    _traj_pos.clear();
    _traj_pos.resize(trajectory.size()*3);

    int i = 0;
    for (StereoVision::Geometry::ShapePreservingTransform<float> pose : trajectory) {
        _traj_pos[i++] = pose.t[0];
        _traj_pos[i++] = pose.t[1];
        _traj_pos[i++] = pose.t[2];
    }

    _has_data = true;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

void OpenGlDrawableTrajectory::clearTrajectory() {
    _traj_pos.clear();

    _has_data = false;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

void OpenGlDrawableTrajectory::setSceneScale(float newSceneScale) {
    _sceneScale = newSceneScale;
}

} // namespace PikaLTools
