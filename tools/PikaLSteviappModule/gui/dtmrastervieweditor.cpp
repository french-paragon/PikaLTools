#include "dtmrastervieweditor.h"

#include <steviapp/gui/imagewidget.h>
#include <steviapp/control/mainwindow.h>

#include <steviapp/datablocks/landmark.h>

#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <QMenu>
#include <QInputDialog>
#include <QDebug>

#include "../datablocks/inputdtm.h"

#include "imageOverlays/inputdtmlandmarksoverlay.h"

namespace PikaLTools {

DTMRasterViewEditor::DTMRasterViewEditor(QWidget *parent)
    : StereoVisionApp::Editor(parent),
      _currentDtm(nullptr),
      _dtm_data_scale(1),
      _displayAdapter(nullptr),
      _current_landmark_id(-1)
{

    _viewWidget = new StereoVisionApp::ImageWidget(this);

    QLabel* levelsLabel = new QLabel(this);
    levelsLabel->setText(tr("Levels:"));

    QLabel* blackLevelLabel = new QLabel(this);
    blackLevelLabel->setText(tr("b:"));

    _blackLevelChannelSpinBox = new QDoubleSpinBox(this);
    _blackLevelChannelSpinBox->setMinimum(0);
    _blackLevelChannelSpinBox->setMaximum(2);
    _blackLevelChannelSpinBox->setValue(0);
    _blackLevelChannelSpinBox->setDecimals(2);

    QLabel* whiteLevelLabel = new QLabel(this);
    whiteLevelLabel->setText(tr("w:"));

    _whiteLevelChannelSpinBox = new QDoubleSpinBox(this);
    _whiteLevelChannelSpinBox->setMinimum(0);
    _whiteLevelChannelSpinBox->setMaximum(2);
    _whiteLevelChannelSpinBox->setValue(0);
    _whiteLevelChannelSpinBox->setDecimals(2);

    QPushButton* commitLevelsButton = new QPushButton(this);
    commitLevelsButton->setText(tr("Set Levels"));

    QLabel* activeLandmarkLabel = new QLabel(this);
    activeLandmarkLabel->setText(tr("Active landmark:"));

    _currentLandmarkMenu = new QComboBox(this);

    QHBoxLayout* toolsLayout = new QHBoxLayout();

    toolsLayout->addStretch();

    toolsLayout->addWidget(levelsLabel);
    toolsLayout->addSpacing(3);

    toolsLayout->addWidget(blackLevelLabel);
    toolsLayout->addWidget(_blackLevelChannelSpinBox);
    toolsLayout->addSpacing(2);

    toolsLayout->addWidget(whiteLevelLabel);
    toolsLayout->addWidget(_whiteLevelChannelSpinBox);
    toolsLayout->addSpacing(2);

    toolsLayout->addSpacing(5);

    toolsLayout->addWidget(commitLevelsButton);

    toolsLayout->addSpacing(5);
    toolsLayout->addStretch();

    toolsLayout->addWidget(activeLandmarkLabel);
    toolsLayout->addWidget(_currentLandmarkMenu);

    toolsLayout->setMargin(3);
    toolsLayout->setSpacing(0);

    //main layout
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addLayout(toolsLayout);
    layout->addWidget(_viewWidget);

    layout->setSpacing(0);
    layout->setMargin(0);

    setLayout(layout);


    _displayOverlay = new InputDtmLandmarksOverlay(_viewWidget);

    _viewWidget->addOverlay(_displayOverlay);
    _viewWidget->installEventFilter(_displayOverlay);

    connect(_displayOverlay, &InputDtmLandmarksOverlay::newPointClick, this, &DTMRasterViewEditor::addPoint);
    connect(_displayOverlay, &InputDtmLandmarksOverlay::menuPointClick, this, &DTMRasterViewEditor::openPointMenu);
    connect(_viewWidget, &StereoVisionApp::ImageWidget::menuNonPointClick, this, &DTMRasterViewEditor::openNonPointMenu);

    _moveToNextLandmark = new QAction(tr("Next landmark"), this);
    _moveToPrevLandmark = new QAction(tr("Previous landmark"), this);

    _moveToNextLandmark->setShortcut(Qt::CTRL + Qt::Key_Right);
    _moveToPrevLandmark->setShortcut(Qt::CTRL + Qt::Key_Left);

    connect(_moveToNextLandmark, &QAction::triggered, this, &DTMRasterViewEditor::moveToNextLandmark);
    connect(_moveToPrevLandmark, &QAction::triggered, this, &DTMRasterViewEditor::moveToPreviousLandmark);

    addAction(_moveToNextLandmark);
    addAction(_moveToPrevLandmark);

    connect(commitLevelsButton, &QPushButton::pressed, this, &DTMRasterViewEditor::commit_bwLevels_changes);

}

void DTMRasterViewEditor::setDtm(InputDtm* dtm) {

    clearDtm();

    if (dtm != nullptr) {

        _currentDtm = dtm;
        _displayOverlay->setInputDtmDatablock(_currentDtm);

        connect(_currentDtm, &QObject::destroyed, this, &DTMRasterViewEditor::clearDtm);

        if (_displayAdapter != nullptr) {
            _viewWidget->setImage(nullptr);
            _displayAdapter->deleteLater();
        }

        if (_currentDtm == nullptr) {
            _displayAdapter = nullptr;
            return;
        }

        auto dtmDataOpt = _currentDtm->dtmData();

        if (!dtmDataOpt.has_value()) {
            _displayAdapter = nullptr;
            return;
        }

        constexpr int maxDtmSide = 10000; //We admit that we do not want a DTM/DSM with higher resolution
        //TODO: make that a configurable option somewhere

        bool fitSize = true;
        int maxSize = maxDtmSide;

        for (int i = 0; i < 2; i++) {
            if (dtmDataOpt.value().raster.shape()[i] > maxSize) {
                fitSize = false;
                maxSize = dtmDataOpt.value().raster.shape()[i];
            }
        }

        if (fitSize) {
            _dtm_data = std::move(dtmDataOpt.value().raster);
            _dtm_data_scale = 1;
            _displayOverlay->setImageRescale(1);
        } else {
            _dtm_data = downScaleDtmDataRaster(dtmDataOpt.value().raster, maxSize, maxDtmSide);
            _dtm_data_scale = double(maxDtmSide)/maxSize;
            _displayOverlay->setImageRescale(_dtm_data_scale);
        }

        _blackLevel = 0;
        _whiteLevel = 1;

        std::function<QColor(float)> colorFunc = [] (float prop) {
            std::array<std::array<float,3>,5> colors =
            {std::array<float,3>{102,0,172},
            std::array<float,3>{33,87,166},
            std::array<float,3>{78,156,103},
             std::array<float,3>{252,255,146},
             std::array<float,3>{220,26,0}};

            float range = prop * colors.size();

            int colId1 = std::floor(range);
            int colId2 = std::ceil(range);

            if (colId1 < 0) {
                colId1 = 0;
            }

            if (colId2 < 0) {
                colId2 = 0;
            }

            if (colId1 >= colors.size()) {
                colId1 = colors.size()-1;
            }

            if (colId2 >= colors.size()) {
                colId2 = colors.size()-1;
            }

            float w = range - std::floor(range);

            uint8_t red = (1-w)*colors[colId1][0] + w*colors[colId2][0];
            uint8_t green = (1-w)*colors[colId1][1] + w*colors[colId2][1];
            uint8_t blue = (1-w)*colors[colId1][2] + w*colors[colId2][2];

            return QColor(red, green, blue);


        };

        _displayAdapter = new StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>(&_dtm_data, _blackLevel, _whiteLevel);
        _displayAdapter->setColorMap(colorFunc);

        recomputeBlackAndWhiteLevel();

        _viewWidget->setImage(_displayAdapter);
    }

}

void DTMRasterViewEditor::clearDtm() {

    if (_currentDtm != nullptr) {

        if (_displayAdapter != nullptr) {
            _viewWidget->setImage(nullptr);
            _displayAdapter->deleteLater();
        }

        disconnect(_currentDtm, nullptr, this, nullptr);
        _currentDtm = nullptr;

        _displayOverlay->setInputDtmDatablock(nullptr);
    }

}

void DTMRasterViewEditor::afterProjectChange(StereoVisionApp::Project* op) {

    if (op != nullptr) {
        disconnect(op, nullptr, this, nullptr);
    }

    if (activeProject() != nullptr) {
        connect(activeProject(), &StereoVisionApp::Project::modelAboutToBeReset, this, &DTMRasterViewEditor::clearCombobox);
        connect(activeProject(), &StereoVisionApp::Project::modelReset, this, &DTMRasterViewEditor::setComboboxIndices, Qt::QueuedConnection);
    }

    if (activeProject() != op) {
        clearCombobox();
    }

    setComboboxIndices();

}

void DTMRasterViewEditor::addPoint(QPointF const& imageCoordinates) {

    StereoVisionApp::Project* p = activeProject();

    if (p == nullptr) {
        return;
    }

    if (_currentDtm == nullptr) {
        return;
    }

    QModelIndex rootLm = _currentLandmarkMenu->rootModelIndex();
    QModelIndex itemIndex = p->index(_currentLandmarkMenu->currentIndex(), 0, rootLm);

    qint64 lmId;

    if (itemIndex == QModelIndex()) { //landmark does not exist yet.

        QString newLmName = QInputDialog::getText(this, tr("New landmark"), tr("Name"), QLineEdit::Normal, tr("New landmark"));

        if (newLmName.isEmpty()) {
            return;
        }

        lmId = p->createDataBlock(StereoVisionApp::Landmark::staticMetaObject.className());

        if (lmId < 0) {
            return;
        }

        StereoVisionApp::Landmark* lm = qobject_cast<StereoVisionApp::Landmark*>(p->getById(lmId));
        lm->setObjectName(newLmName);

        _currentLandmarkMenu->setCurrentIndex(p->countTypeInstances(StereoVisionApp::Landmark::staticMetaObject.className())-1);

    } else {

        lmId = p->data(itemIndex, StereoVisionApp::Project::IdRole).toInt();
    }

    DtmLandmark* lm = _currentDtm->getDtmLandmarkByLandmarkId(lmId);

    QPointF scaledImgCoordinates = imageCoordinates/_dtm_data_scale;

    if (lm == nullptr) {
        bool isUncertain = true; //TODO: add input
        _currentDtm->addDtmLandmark(scaledImgCoordinates, lmId, isUncertain, 1);
    } else {
        lm->setImageCoordinates(scaledImgCoordinates);
    }

}
void DTMRasterViewEditor::openPointMenu(qint64 id, QImageDisplay::ImageWidget *widget, QPoint const& pos) {

    if (widget != _viewWidget) {
        return;
    }

    if (_currentDtm == nullptr) {
        return;
    }

    QMenu cMenu;

    QAction* rm = new QAction(tr("remove"), &cMenu);

    connect(rm, &QAction::triggered, [this, id] () {
        _currentDtm->clearDtmLandmark(id);
    });

    cMenu.addAction(rm);

    cMenu.exec(_viewWidget->mapToGlobal(pos));

}
void DTMRasterViewEditor::openNonPointMenu(QPoint const& pos) {

    QMenu cMenu;

    cMenu.addAction(_moveToNextLandmark);
    cMenu.addAction(_moveToPrevLandmark);

    cMenu.exec(_viewWidget->mapToGlobal(pos));

}

void DTMRasterViewEditor::moveToNextLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();
    comboId += 1;

    if (comboId >= _currentLandmarkMenu->count()) {
        comboId = -1;
    }

    _currentLandmarkMenu->setCurrentIndex(comboId);

}
void DTMRasterViewEditor::moveToPreviousLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();

    if (comboId == -1) {
        comboId = _currentLandmarkMenu->count();
    }

    comboId -= 1;

    _currentLandmarkMenu->setCurrentIndex(comboId);

}

void DTMRasterViewEditor::onCurrentLandmarkIndexChanged() {

    StereoVisionApp::Project* p = activeProject();

    if (p == nullptr) {
        return;
    }

    QModelIndex rootLm = p->indexOfClass(StereoVisionApp::Landmark::staticMetaObject.className());
    QModelIndex itemIndex = p->index(_currentLandmarkMenu->currentIndex(), 0, rootLm);

    if (itemIndex == QModelIndex()) {
        _displayOverlay->displaySinglePoint(-1);
        return;
    } else {
        _current_landmark_id = itemIndex.data(StereoVisionApp::Project::IdRole).toInt();
    }

}

void DTMRasterViewEditor::clearCombobox() {

    _currentLandmarkMenu->blockSignals(true);
    _currentLandmarkMenu->setCurrentIndex(-1);
    _currentLandmarkMenu->clear();
    _currentLandmarkMenu->blockSignals(false);

}
void DTMRasterViewEditor::setComboboxIndices() {

    if (activeProject() == nullptr) {
        return;
    }

    qint64 currentLmIndex = _current_landmark_id;

    _currentLandmarkMenu->setModel(activeProject());
    _currentLandmarkMenu->setRootModelIndex(activeProject()->indexOfClass(StereoVisionApp::Landmark::staticMetaObject.className()));

    if (currentLmIndex != -1) {
        QVector<qint64> ids = activeProject()->getIdsByClass(StereoVisionApp::Landmark::staticMetaObject.className());
        _currentLandmarkMenu->setCurrentIndex(ids.indexOf(currentLmIndex));
    } else {
        _currentLandmarkMenu->setCurrentIndex(-1);
    }

}

void DTMRasterViewEditor::recomputeBlackAndWhiteLevel(bool all) {

    int j0;
    int jn;

    int i0;
    int in;

    if (all) {

        i0 = 0;
        j0 = 0;

        in = _dtm_data.shape()[0];
        jn = _dtm_data.shape()[1];

    } else {
        QPoint origin(0,0);
        QPoint end(_viewWidget->width(), _viewWidget->height());

        QPointF start = _viewWidget->widgetToImageCoordinates(origin);
        QPointF finish = _viewWidget->widgetToImageCoordinates(end);

        j0 = std::floor(start.x());
        jn = std::ceil(finish.x());

        i0 = std::floor(start.y());
        in = std::ceil(finish.y());
    }

    if (i0 < 0) {
        i0 = 0;
    }

    if (j0 < 0) {
        j0 = 0;
    }

    if (in > _dtm_data.shape()[0]) {
        in = _dtm_data.shape()[0];
    }

    if (jn > _dtm_data.shape()[1]) {
        jn = _dtm_data.shape()[1];
    }

    if (in <= i0) {

        _blackLevel = 0;
        _whiteLevel = 0;

    } else if (jn <= j0) {

        _blackLevel = 0;
        _whiteLevel = 0;

    } else {

        _blackLevel = std::numeric_limits<float>::infinity();
        _whiteLevel = -std::numeric_limits<float>::infinity();

        for (int i = i0; i < in; i++) {
            for (int j = j0; j < jn; j++) {

                float value = _dtm_data.valueUnchecked(i,j);

                if (!std::isfinite(value)) {
                    continue;
                }

                if (value < -1e10) {
                    continue;
                }

                if (_blackLevel > value) {
                    _blackLevel = value;
                }

                if (_whiteLevel < value) {
                    _whiteLevel = value;
                }

            }
        }
    }

    qDebug() << "whiteLevel: " << _whiteLevel << " blackLevel: " << _blackLevel;

    if (all) {
        _blackLevelChannelSpinBox->setMinimum(_blackLevel);
        _blackLevelChannelSpinBox->setMaximum(_whiteLevel);
        _whiteLevelChannelSpinBox->setMinimum(_blackLevel);
        _whiteLevelChannelSpinBox->setMaximum(_whiteLevel);
    }

    _blackLevelChannelSpinBox->setValue(_blackLevel);
    _whiteLevelChannelSpinBox->setValue(_whiteLevel);

    _displayAdapter->setBWLevel(_blackLevel, _whiteLevel);
    _displayAdapter->update();

}


Multidim::Array<float, 2> DTMRasterViewEditor::downScaleDtmDataRaster(Multidim::Array<float, 2> const& fullRes, int currentSize, int targetSize) {

    if (currentSize <= targetSize) {
        return fullRes;
    }

    double invScale = double(currentSize)/targetSize;

    std::array<int,2> shape;

    for (int i = 0; i < 2; i++) {
        long s = fullRes.shape()[i];
        s *= targetSize;
        s /= currentSize;
        shape[i] = s;
    }

    Multidim::Array<float, 2> ret(shape);

    for (int i = 0; i < shape[0]; i++) {
        for (int j = 0; j < shape[1]; j++) {
            double si = invScale*i;
            double sj = invScale*j;

            std::array<int,2> is = {int(std::floor(si)), int(std::ceil(si))};
            std::array<int,2> js = {int(std::floor(sj)), int(std::ceil(sj))};

            float data = fullRes.valueOrAlt({is[0], js[0]}, 0);

            for (int i : is) {
                for (int j : js) {
                    float cand = fullRes.valueOrAlt({i, j}, 0);

                    if (cand > data) {
                        data = cand;
                    }
                }
            }

            ret.atUnchecked(i,j) = data;
        }
    }

    return ret;

}

void DTMRasterViewEditor::commit_bwLevels_changes() {

    if (_displayAdapter == nullptr) {
        return;
    }

    _blackLevel = _blackLevelChannelSpinBox->value();
    _whiteLevel = _whiteLevelChannelSpinBox->value();

    _displayAdapter->setBWLevel(_blackLevel, _whiteLevel);
    _displayAdapter->update();

}

DTMRasterViewEditorFactory::DTMRasterViewEditorFactory(QObject *parent) :
    StereoVisionApp::EditorFactory(parent)
{

}

QString DTMRasterViewEditorFactory::TypeDescrName() const {
    return tr("DTM raster viewer");
}
QString DTMRasterViewEditorFactory::itemClassName() const {
    return DTMRasterViewEditor::staticMetaObject.className();
}
StereoVisionApp::Editor* DTMRasterViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new DTMRasterViewEditor(parent);
}

} // namespace PikaLTools
