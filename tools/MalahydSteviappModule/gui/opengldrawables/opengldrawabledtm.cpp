#include "opengldrawabledtm.h"

#include "io/georasterreader.h"

#include <steviapp/datablocks/project.h>

#include <MultidimArrays/MultidimIndexManipulators.h>

#include <QOpenGLShaderProgram>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

#include "datablocks/inputdtm.h"

#include <proj.h>

namespace PikaLTools {

OpenGlDrawableDtm::OpenGlDrawableDtm(StereoVisionApp::OpenGl3DSceneViewWidget* parent) :
    StereoVisionApp::OpenGlDrawable(parent),
    _has_data(false),
    _idx_buffer(QOpenGLBuffer::IndexBuffer)
{

}

void OpenGlDrawableDtm::initializeGL() {

    _scene_vao.create();

    _pos_buffer.create();
    _pos_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _normal_buffer.create();
    _normal_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _idx_buffer.create();
    _idx_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _dtmProgram = new QOpenGLShaderProgram();
    _dtmProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/glShaders/dtmViewer.vert");
    _dtmProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/glShaders/dtmViewer.frag");

    _dtmProgram->link();

}
void OpenGlDrawableDtm::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    GLenum err = glGetError();
    while(err != GL_NO_ERROR) {
        err = glGetError();
    }

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_has_data) {

        int posLocation;
        int normalLocation;

        if (!_vertices_pos.empty()) {

            posLocation = _dtmProgram->attributeLocation("in_location");
            normalLocation = _dtmProgram->attributeLocation("in_normal");

            bool hasToreload = _has_to_reset_gl_buffers;
            _has_to_reset_gl_buffers = false;

            _dtmProgram->bind();
            _scene_vao.bind();
            _pos_buffer.bind();

            if (hasToreload) {
                _pos_buffer.allocate(_vertices_pos.data(), _vertices_pos.size()*sizeof (GLfloat));
            }

            _dtmProgram->setAttributeBuffer(posLocation, GL_FLOAT, 0, 3);
            _dtmProgram->enableAttributeArray(posLocation);

            _normal_buffer.bind();

            if (hasToreload) {
                _normal_buffer.allocate(_vertices_normals.data(), _vertices_normals.size()*sizeof (GLfloat));
            }

            _dtmProgram->setAttributeBuffer(normalLocation, GL_FLOAT, 0, 3);
            _dtmProgram->enableAttributeArray(normalLocation);

            _idx_buffer.bind();

            if (hasToreload) {
                _idx_buffer.allocate(_faces_idxs.data(), _faces_idxs.size()*sizeof (GLuint));
            }

            _dtmProgram->setUniformValue("matrixViewProjection", projectionView*modelView);

            _dtmProgram->setUniformValue("sceneScale", _sceneScale);

            _dtmProgram->setUniformValue("lightDirection", GLfloat(0.5), GLfloat(0.5), GLfloat(0.5));

            f->glDrawElements(GL_TRIANGLES, _faces_idxs.size(), GL_UNSIGNED_INT, 0);

            _scene_vao.release();
            _normal_buffer.release();

            _dtmProgram->release();

        }
    }

    err = glGetError();
    if (err != GL_NO_ERROR) {
        qDebug() << "OpenGL error while rendering dtm: " << err;
    }

}
void OpenGlDrawableDtm::clearViewRessources() {

    if (_dtmProgram != nullptr) {
        delete _dtmProgram;
    }

    if (_pos_buffer.isCreated()) {
        _pos_buffer.destroy();
    }

    if (_normal_buffer.isCreated()) {
        _normal_buffer.destroy();
    }

    if (_idx_buffer.isCreated()) {
        _idx_buffer.destroy();
    }
    _scene_vao.destroy();

}

void OpenGlDrawableDtm::setDtm(Multidim::Array<float,3> const& points, Multidim::Array<bool,2> const* validPixels) {

    int nFaces = 2*(points.shape()[0]-1)*(points.shape()[1]-1);

    int nPoints = points.shape()[0]*points.shape()[1];

    bool useValidPixelsMask = false;

    if (validPixels != nullptr) {
        if (validPixels->shape()[0] == points.shape()[0] and validPixels->shape()[1] == points.shape()[1]) {

            useValidPixelsMask = true;
            nPoints = 0;

            for (int i = 0; i < points.shape()[0]; i++) {
                for (int j = 0; j < points.shape()[1]; j++) {
                    if (validPixels->valueUnchecked(i,j)) {
                        nPoints++;
                    }
                }
            }
        }
    }

    std::vector<int> pointsIdxMap;

    _vertices_pos.clear();
    _vertices_pos.resize(3*nPoints);

    _vertices_normals.clear();
    _vertices_normals.resize(3*nPoints);

    _faces_idxs.clear();
    _faces_idxs.reserve(nFaces);

    auto shape = points.shape();

    auto coords2idxs = [&shape] (int i, int j) {
        return shape[1]*i + j;
    };

    if (useValidPixelsMask) {
        pointsIdxMap.resize(points.shape()[0]*points.shape()[1]);

        int id = 0;

        for (int i = 0; i < points.shape()[0]; i++) {
            for (int j = 0; j < points.shape()[1]; j++) {

                int flatId = coords2idxs(i,j);

                if (validPixels->valueUnchecked(i,j)) {
                    pointsIdxMap[flatId] = id;
                    id++;
                } else {
                    pointsIdxMap[flatId] = -1;
                }
            }
        }
    }


    for (int i = 0; i < points.shape()[0]; i++) {
        for (int j = 0; j < points.shape()[1]; j++) {

            int imgId = coords2idxs(i,j);
            int flatId = imgId;

            if (useValidPixelsMask) {
                flatId = pointsIdxMap[imgId];

                if (flatId < 0) {
                    continue;
                }
            }

            Eigen::Vector3f currentPos;

            for (int c = 0; c < points.shape()[2]; c++) {
                _vertices_pos[3*flatId+c] = points.valueUnchecked(i,j,c);
                currentPos[c] = points.valueUnchecked(i,j,c);
            }

            Eigen::Vector3f sum = Eigen::Vector3f::Zero();

            for (int di : {-1, 1}) {

                if (i+di < 0 or i+di >= points.shape()[0]) {
                    continue;
                }

                if (useValidPixelsMask) {
                    if (!validPixels->valueUnchecked(i+di,j)) {
                        continue;
                    }
                }

                for (int dj : {-1, 1}) {

                    if (di == 0 and dj == 0) {
                        continue;
                    }

                    if (j+dj < 0 or j+dj >= points.shape()[1]) {
                        continue;
                    }

                    if (useValidPixelsMask) {
                        if (!validPixels->valueUnchecked(i,j+dj)) {
                            continue;
                        }
                    }

                    Eigen::Vector3f vec1;
                    Eigen::Vector3f vec2;

                    for (int c = 0; c < points.shape()[2]; c++) {
                        vec1[c] = points.valueUnchecked(i+di,j,c);
                        vec2[c] = points.valueUnchecked(i,j+dj,c);
                    }

                    Eigen::Vector3f delta1 = vec1 - currentPos;
                    Eigen::Vector3f delta2 = vec2 - currentPos;

                    Eigen::Vector3f normal = (di*dj)*delta1.cross(delta2);

                    sum += normal.normalized();

                }
            }

            sum.normalize();

            for (int c = 0; c < points.shape()[2]; c++) {
                _vertices_normals[3*flatId+c] = sum[c];
            }


            if (i < points.shape()[0]-1 and j < points.shape()[1]-1) {
                int id0 = flatId;
                int id1 = coords2idxs(i+1,j);
                int id2 = coords2idxs(i,j+1);

                if (useValidPixelsMask) {
                    id1 = pointsIdxMap[id1];
                    id2 = pointsIdxMap[id2];
                }

                if (id0 >= 0 and id1 >= 0 and id2 >= 0) {
                    _faces_idxs.push_back(id0);
                    _faces_idxs.push_back(id1);
                    _faces_idxs.push_back(id2);
                }
            }

            if (i > 0 and j > 0) {
                int id0 = flatId;
                int id1 = coords2idxs(i-1,j);
                int id2 = coords2idxs(i,j-1);

                if (useValidPixelsMask) {
                    id1 = pointsIdxMap[id1];
                    id2 = pointsIdxMap[id2];
                }

                if (id0 >= 0 and id1 >= 0 and id2 >= 0) {
                    _faces_idxs.push_back(id0);
                    _faces_idxs.push_back(id1);
                    _faces_idxs.push_back(id2);
                }
            }
        }
    }

    _has_data = true;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();

}

void OpenGlDrawableDtm::setDtm(InputDtm* inputDtmBlock) {

    StereoVisionApp::Project* project = inputDtmBlock->getProject();

    if (project == nullptr) {
        return;
    }

    if (!project->hasLocalCoordinateFrame()) {
        //TODO: return an error message in that case
        return;
    }

    auto rasterDataOpt = inputDtmBlock->dtmData();

    if (!rasterDataOpt.has_value()) {
        return;
    }

    constexpr int maxSideVertices = 5000;

    StereoVisionApp::Geo::GeoRasterData<float,2>& rasterData = rasterDataOpt.value();


    auto inShape = rasterData.raster.shape();
    float scale = 1;
    int max = maxSideVertices;

    for (int i = 0; i < inShape.size(); i++) {
        if (inShape[i] > max) {
            max = inShape[i];
        }
    }

    auto outShape = inShape;

    if (max > maxSideVertices) {

        scale = float(max)/maxSideVertices;

        for (int i = 0; i < inShape.size(); i++) {
            long s = inShape[i];
            s *= maxSideVertices;
            s /= max;

            outShape[i] = s;
        }
    }

    int nPoints = outShape[0]*outShape[1];

    Multidim::Array<double,3> vertices_pos({outShape[0], outShape[1], 3}, {3*outShape[1],3,1});
    Multidim::Array<bool,2> vertices_valid(outShape);

    int nValid = 0;

    for (int i = 0; i < outShape[0]; i++) {
        for (int j = 0; j < outShape[1]; j++) {

            float in_i = scale*i;
            float in_j = scale*j;

            std::array<int, 2> in_is = {int(std::floor(in_i)), int(std::ceil(in_i))};
            std::array<int, 2> in_js = {int(std::floor(in_j)), int(std::ceil(in_j))};

            float h = std::nanf("");

            for (int in_i : in_is) {
                for (int in_j : in_js) {
                    float cand = rasterData.raster.valueOrAlt({in_i, in_j}, std::nanf(""));

                    if (!std::isfinite(cand)) {
                        continue;
                    }

                    if (!std::isfinite(h) or cand > h) {
                        h = cand;
                    }
                }
            }

            if (!std::isfinite(h)) {
                vertices_valid.atUnchecked(i,j) = false;
                continue;
            }

            bool ok;
            float thresh = inputDtmBlock->minHeight().toFloat(&ok);

            if (ok and h < thresh) {
                vertices_valid.atUnchecked(i,j) = false;
                continue;
            }

            thresh = inputDtmBlock->maxHeight().toFloat(&ok);

            if (ok and h > thresh) {
                vertices_valid.atUnchecked(i,j) = false;
                continue;
            }

            Eigen::Vector3d homogeneousImgCoord(in_j,in_i,1);
            Eigen::Vector2d geoCoord = rasterData.geoTransform*homogeneousImgCoord;

            vertices_pos.atUnchecked(i,j,0) = geoCoord.x();
            vertices_pos.atUnchecked(i,j,1) = geoCoord.y();

            vertices_pos.atUnchecked(i,j,2) = h;

            vertices_valid.atUnchecked(i,j) = true;
            nValid++;

        }
    }

    PJ_CONTEXT* ctx = proj_context_create();


    const char* wgs84_ecef = "EPSG:4978";


    PJ* reprojector = proj_create_crs_to_crs(ctx, rasterData.crsInfos.c_str(), wgs84_ecef, nullptr);

    if (reprojector == 0) { //in case of error
        return;
    }

    proj_trans_generic(reprojector, PJ_FWD,
                       &vertices_pos.atUnchecked(0,0,0), 3*sizeof(double), nPoints,
                       &vertices_pos.atUnchecked(0,0,1), 3*sizeof(double), nPoints,
                       &vertices_pos.atUnchecked(0,0,2), 3*sizeof(double), nPoints,
                       nullptr,0,0); //reproject to ecef coordinates

    StereoVision::Geometry::AffineTransform<float> transformation = project->ecef2local();


    Multidim::Array<float,3> vertices_local_pos({outShape[0], outShape[1], 3}, {3*outShape[1],3,1});

    for (int i = 0; i < outShape[0]; i++) {
        for (int j = 0; j < outShape[1]; j++) {

            if (!vertices_valid.valueUnchecked(i,j)) {
                continue;
            }

            Eigen::Vector3f coord(vertices_pos.valueUnchecked(i,j,0),
                                  vertices_pos.valueUnchecked(i,j,1),
                                  vertices_pos.valueUnchecked(i,j,2));

            Eigen::Vector3f local = transformation*coord;

            vertices_local_pos.atUnchecked(i,j,0) = local.x();
            vertices_local_pos.atUnchecked(i,j,1) = local.y();
            vertices_local_pos.atUnchecked(i,j,2) = local.z();

        }
    }

    proj_destroy(reprojector);
    proj_context_destroy(ctx);

    setDtm(vertices_local_pos, &vertices_valid);

}

void OpenGlDrawableDtm::clearDtm() {
    _vertices_pos.clear();
    _vertices_normals.clear();
    _faces_idxs.clear();

    _has_data = false;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

void OpenGlDrawableDtm::setSceneScale(float newSceneScale) {
    _sceneScale = newSceneScale;
}

} // namespace PikaLTools
