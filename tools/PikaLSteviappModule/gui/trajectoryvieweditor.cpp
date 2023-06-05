#include "trajectoryvieweditor.h"

#include "geo/georasterreader.h"

#include "datablocks/bilacquisitiondata.h"
#include "datablocks/comparisontrajectory.h"
#include "datablocks/inputdtm.h"

#include <steviapp/gui/opengl3dsceneviewwidget.h>
#include <steviapp/gui/openGlDrawables/opengldrawablescenegrid.h>

#include "opengldrawables/opengldrawabledtm.h"
#include "opengldrawables/opengldrawabletrajectory.h"

#include <proj.h>

#include <QVBoxLayout>
#include <QHBoxLayout>

#include <QLabel>
#include <QSpinBox>
#include <QMessageBox>

namespace PikaLTools {

TrajectoryViewEditor::TrajectoryViewEditor(QWidget *parent) :
    StereoVisionApp::Editor(parent),
    _baseTrajectoryTimes(),
    _compareTrajectoryTimes()
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

    _drawableDtm = new OpenGlDrawableDtm(_viewScene);

    _drawableTrajectory = new OpenGlDrawableTrajectory(_viewScene);
    _drawableTrajectory->setBaseColor(QColor(210, 0, 0, 255));
    _drawableTrajectory->setHighlightSegmentColor(QColor(240, 190, 0, 255));

    _comparisonTrajectory = new OpenGlDrawableTrajectory(_viewScene);
    _comparisonTrajectory->setBaseColor(QColor(0, 210, 0, 255));
    _comparisonTrajectory->setHighlightSegmentColor(QColor(10, 10, 210, 255));
    _comparisonTrajectory->setSegmentStart(-1);
    _comparisonTrajectory->setSegmentEnd(-1);

    setDrawableScale(0.1);

    _viewScene->addDrawable(_drawableTrajectory);
    _viewScene->addDrawable(_comparisonTrajectory);
    _viewScene->addDrawable(_drawableDtm);

    setLayout(layout);
}

void TrajectoryViewEditor::setStartLine(int bilLine) {

    if (_startLineSpinBox->value() != bilLine) {
        _startLineSpinBox->setValue(bilLine);
    }

    int lcfLineId = bil2lcfLineId(bilLine);
    _drawableTrajectory->setSegmentStart(lcfLineId);

    if (!_baseTrajectoryTimes.empty() and !_compareTrajectoryTimes.empty()) {
        double lcfTime = _baseTrajectoryTimes[std::min<int>(lcfLineId, _baseTrajectoryTimes.size()-1)];

        int compareLineId = _compareTrajectoryTimes.size();

        for (int i = 0; i < _compareTrajectoryTimes.size(); i++) {
            if (_compareTrajectoryTimes[i] > lcfTime) {
                compareLineId = i-1;
                break;
            }
        }

        _comparisonTrajectory->setSegmentStart(compareLineId);
    } else {
        _comparisonTrajectory->setSegmentStart(-1);
        _comparisonTrajectory->setSegmentEnd(-1);
    }
}
void TrajectoryViewEditor::setEndLine(int bilLine) {

    if (_endLineSpinBox->value() != bilLine) {
        _endLineSpinBox->setValue(bilLine);
    }

    int lcfLineId = bil2lcfLineId(bilLine);
    _drawableTrajectory->setSegmentEnd(lcfLineId);

    if (!_baseTrajectoryTimes.empty() and !_compareTrajectoryTimes.empty()) {
        double lcfTime = _baseTrajectoryTimes[std::min<int>(lcfLineId, _baseTrajectoryTimes.size()-1)];

        int compareLineId = _compareTrajectoryTimes.size();

        for (int i = 0; i < _compareTrajectoryTimes.size(); i++) {
            if (_compareTrajectoryTimes[i] > lcfTime) {
                compareLineId = i;
                break;
            }
        }

        _comparisonTrajectory->setSegmentEnd(compareLineId);
    } else {
        _comparisonTrajectory->setSegmentStart(-1);
        _comparisonTrajectory->setSegmentEnd(-1);
    }
}

void TrajectoryViewEditor::setDrawableScale(float scale) {
    _drawableTrajectory->setSceneScale(scale);
    _comparisonTrajectory->setSceneScale(scale);
    _drawableDtm->setSceneScale(scale);
}

void TrajectoryViewEditor::setTrajectory(const BilSequenceAcquisitionData &bilSequence) {

    std::vector<StereoVision::Geometry::AffineTransform<float>> trajectory = bilSequence.ecefTrajectory();

    StereoVisionApp::Project* project = bilSequence.getProject();

    if (project == nullptr) {
        return;
    }

    if (!project->hasLocalCoordinateFrame()) {
        QMessageBox::warning(this,
                             tr("Project does not have a local frame of reference"),
                             tr("Setup a local frame of reference before showing the lcf trajectory!"));
        return;
    }

    StereoVision::Geometry::AffineTransform<float> transformation = project->ecef2local();

    std::vector<StereoVision::Geometry::AffineTransform<float>> localTrajectory;
    localTrajectory.reserve(trajectory.size());

    float minXy = -1;
    float maxXy = 1;

    for (int i = 0; i < trajectory.size(); i++) {
        Eigen::Vector3f pos = transformation*trajectory[i].t;

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

        localTrajectory.push_back(StereoVision::Geometry::AffineTransform<float>(transformation.R*trajectory[i].R, pos));
    }

    float maxDist = std::max(std::abs(minXy), maxXy);
    setDrawableScale(12./maxDist);

    _drawableTrajectory->setTrajectory(localTrajectory);

    _bil2lcfLines.clear();

    int nLines = 0;
    int nLcfLines = 0;

    _bil2lcfLines.insert(nLines, nLcfLines);

    for (BilSequenceAcquisitionData::BilAcquisitionData const& bil : bilSequence.getBilInfos()) {

        nLines += bil.getNLines();
        nLcfLines += bil.getLcfNLines();

        _bil2lcfLines.insert(nLines, nLcfLines);
    }

    _baseTrajectoryTimes = bilSequence.ecefTimes();

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

void TrajectoryViewEditor::setComparisonTrajectory(const ComparisonTrajectory &comparisonTrajectory) {

    std::vector<Eigen::Vector3f> trajectory = comparisonTrajectory.ecefTrajectory();

    StereoVisionApp::Project* project = comparisonTrajectory.getProject();

    if (project == nullptr) {
        return;
    }

    if (!project->hasLocalCoordinateFrame()) {
        QMessageBox::warning(this,
                             tr("Project does not have a local frame of reference"),
                             tr("Setup a local frame of reference before showing the comparison trajectory!"));
        return;
    }

    StereoVision::Geometry::AffineTransform<float> transformation = project->ecef2local();

    std::vector<Eigen::Vector3f> localTrajectory;
    localTrajectory.reserve(trajectory.size());

    for (int i = 0; i < trajectory.size(); i++) {
        localTrajectory.push_back(transformation*trajectory[i]);
    }

    _comparisonTrajectory->setTrajectory(localTrajectory);

    _compareTrajectoryTimes = comparisonTrajectory.ecefTimes();

}
void TrajectoryViewEditor::clearComparisonTrajectory() {
    _comparisonTrajectory->clearTrajectory();
}

void TrajectoryViewEditor::setDtm(InputDtm* bilSequence) {

    StereoVisionApp::Project* project = bilSequence->getProject();

    if (project == nullptr) {
        return;
    }

    if (!project->hasLocalCoordinateFrame()) {
        QMessageBox::warning(this,
                             tr("Project does not have a local frame of reference"),
                             tr("Setup a local frame of reference before showing the lcf trajectory!"));
        return;
    }

    auto rasterDataOpt = readGeoRasterData<float,2>(bilSequence->getDataSource().toStdString());

    if (!rasterDataOpt.has_value()) {
        return;
    }

    GeoRasterData<float,2>& rasterData = rasterDataOpt.value();

    int nPoints = rasterData.raster.flatLenght();

    auto inShape = rasterData.raster.shape();

    OGRSpatialReference ogrSpatialRef(rasterData.crsInfos.c_str());
    bool invertXY = ogrSpatialRef.EPSGTreatsAsLatLong(); //ogr will always treat coordinates as lon then lat, but proj will stick to the epsg order definition. This mean we might need to invert the order.

    Multidim::Array<double,3> vertices_pos({inShape[0], inShape[1], 3}, {3*inShape[1],3,1});
    Multidim::Array<bool,2> vertices_valid(inShape);

    for (int i = 0; i < rasterData.raster.shape()[0]; i++) {
        for (int j = 0; j < rasterData.raster.shape()[1]; j++) {

            float h = rasterData.raster.valueUnchecked(i,j);

            bool ok;
            float thresh = bilSequence->minHeight().toFloat(&ok);

            if (ok and h < thresh) {
                vertices_valid.atUnchecked(i,j) = false;
                continue;
            }

            thresh = bilSequence->maxHeight().toFloat(&ok);

            if (ok and h > thresh) {
                vertices_valid.atUnchecked(i,j) = false;
                continue;
            }

            Eigen::Vector3d homogeneousImgCoord(j,i,1);
            Eigen::Vector2d geoCoord = rasterData.geoTransform*homogeneousImgCoord;

            if (invertXY) {
                vertices_pos.atUnchecked(i,j,0) = geoCoord.y();
                vertices_pos.atUnchecked(i,j,1) = geoCoord.x();
            } else {
                vertices_pos.atUnchecked(i,j,0) = geoCoord.x();
                vertices_pos.atUnchecked(i,j,1) = geoCoord.y();
            }

            vertices_pos.atUnchecked(i,j,2) = h;

            vertices_valid.atUnchecked(i,j) = true;

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


    Multidim::Array<float,3> vertices_local_pos({inShape[0], inShape[1], 3}, {3*inShape[1],3,1});

    for (int i = 0; i < rasterData.raster.shape()[0]; i++) {
        for (int j = 0; j < rasterData.raster.shape()[1]; j++) {

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

    _drawableDtm->setDtm(vertices_local_pos, &vertices_valid);

}
void TrajectoryViewEditor::clearDtm() {
    _drawableDtm->clearDtm();
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
