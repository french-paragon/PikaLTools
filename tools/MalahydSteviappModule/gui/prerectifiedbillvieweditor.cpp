#include "prerectifiedbillvieweditor.h"

#include <steviapp/gui/imagewidget.h>
#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/landmark.h>

#include <QMouseEvent>
#include <QLabel>
#include <QComboBox>
#include <QHBoxLayout>
#include <QAction>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenu>

#include "../datablocks/bilacquisitiondata.h"

#include "./imageOverlays/bilsequencelandmarksoverlay.h"

namespace PikaLTools {

class PreRectifiedBilSequenceLandmarksOverlay : public BilSequenceLandmarksOverlay
{
public:
    PreRectifiedBilSequenceLandmarksOverlay(QWidget *parent = nullptr) :
        BilSequenceLandmarksOverlay(parent)
    {
        _rectifiedEditor = qobject_cast<PreRectifiedBillViewEditor*>(parent);
    }
protected:

    PreRectifiedBillViewEditor* _rectifiedEditor;

    virtual QPointF getPointCoordinates(qint64 id) const override {
        PreRectifiedBillViewEditor* editor = _rectifiedEditor;

        if (editor == nullptr) {
            return QPointF(-1,-1);
        }

        BilSequenceLandmark* lm = _currentDataBlock->getBilSequenceLandmark(id);
        QPointF coords = lm->bilSequenceCoordinates();

        QPointF ret = editor->mapFromOriginalSequence(coords);
        return ret;
    }
};

PreRectifiedBillViewEditor::PreRectifiedBillViewEditor(QWidget *parent)
    : StereoVisionApp::Editor{parent}
{
    _viewWidget = new StereoVisionApp::ImageWidget(this);

    _viewWidget->setMouseMoveHandler([this] (QMouseEvent* event) -> bool {
        QPointF pos = _viewWidget->widgetToImageCoordinates(event->pos());

        QPointF originalPos = mapToOriginalSequence(pos);

        QString msg = PreRectifiedBillViewEditor::tr("Image Pos %1, %2 (original %3, %4)")
                          .arg(pos.y()).arg(pos.x())
                          .arg(originalPos.y()).arg(originalPos.x());
        sendStatusMessage(msg);
        event->accept();
        return true;
    });

    QLabel* activeLandmarkLabel = new QLabel(this);
    activeLandmarkLabel->setText(tr("Active landmark:"));

    _currentLandmarkMenu = new QComboBox(this);

    QHBoxLayout* toolsLayout = new QHBoxLayout();

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

    //image setup

    _img_display_adapter = new StereoVisionApp::ImageDataDisplayAdapter(this);
    _viewWidget->setImage(_img_display_adapter);

    _img_landmark_overlay = new PreRectifiedBilSequenceLandmarksOverlay(this);

    _viewWidget->addOverlay(_img_landmark_overlay);
    _viewWidget->installEventFilter(_img_landmark_overlay);

    connect(_img_landmark_overlay, &BilSequenceLandmarksOverlay::newPointClick, this, &PreRectifiedBillViewEditor::addPoint);
    connect(_img_landmark_overlay, &BilSequenceLandmarksOverlay::menuPointClick, this, &PreRectifiedBillViewEditor::openPointMenu);
    connect(_viewWidget, &StereoVisionApp::ImageWidget::menuNonPointClick, this, &PreRectifiedBillViewEditor::openNonPointMenu);

    _moveToNextLandmark = new QAction(tr("Next landmark"), this);
    _moveToPrevLandmark = new QAction(tr("Previous landmark"), this);

    _moveToNextLandmark->setShortcut(Qt::CTRL + Qt::Key_Right);
    _moveToPrevLandmark->setShortcut(Qt::CTRL + Qt::Key_Left);

    connect(_moveToNextLandmark, &QAction::triggered, this, &PreRectifiedBillViewEditor::moveToNextLandmark);
    connect(_moveToPrevLandmark, &QAction::triggered, this, &PreRectifiedBillViewEditor::moveToPreviousLandmark);

    addAction(_moveToNextLandmark);
    addAction(_moveToPrevLandmark);
}

PreRectifiedBillViewEditor::~PreRectifiedBillViewEditor() {

}

QPointF PreRectifiedBillViewEditor::mapToOriginalSequence(QPointF const& source) const {
    int line = std::round(source.y());

    int deltaLine = 0;

    if (line < 0) {
        deltaLine = line;
        line = 0;
    } else if (line >= _verticalLinesMatch.size()) {
        deltaLine = line - _verticalLinesMatch.size() + 1;
        line = _verticalLinesMatch.size()-1;
    }

    int originalLine = _verticalLinesMatch[line];

    int horizontalOffsetId = std::max(0, std::min(_horizontalLinesOffsets.size()-1, originalLine));

    QPointF ret (source.x() + _horizontalLinesOffsets[horizontalOffsetId],
                originalLine + deltaLine);

    return ret;
}

QPointF PreRectifiedBillViewEditor::mapFromOriginalSequence(QPointF const& source) const {
    int line = std::round(source.y());

    float y = line;

    if (line > 0 and line < _verticalLinesMatch.last()) {

        for (int i = 1; i < _verticalLinesMatch.size(); i++) {
            if (_verticalLinesMatch[i-1] <= line and _verticalLinesMatch[i] >= line) {
                float alpha = float(line - _verticalLinesMatch[i-1])/
                              float(_verticalLinesMatch[i] - _verticalLinesMatch[i-1]);

                y = i-1+alpha;
            }
        }

    } else if (line >= _verticalLinesMatch.last()) {
        int delta = line - _verticalLinesMatch.last();
        y = _verticalLinesMatch.size() + delta;
    }

    int horizontalOffsetId = std::max(0, std::min(_horizontalLinesOffsets.size()-1, line));

    float x = source.x() - _horizontalLinesOffsets[horizontalOffsetId];

    return QPointF(x,y);
}

void PreRectifiedBillViewEditor::setSequenceData(BilSequenceAcquisitionData* sequence,
                                                 QVector<int> const& verticalLinesMatch,
                                                 QVector<float> const& horizontalLinesOffsets,
                                                 std::array<int,3> const& imgShape) {

    _sequence = sequence;
    _img_landmark_overlay->setBilSequenceDatablock(_sequence);
    _img_landmark_overlay->setInitialLine(0);
    _img_landmark_overlay->setFinalLine(imgShape[0]);
    _img_landmark_overlay->setBilWidth(imgShape[1]);
    _verticalLinesMatch = verticalLinesMatch;
    _horizontalLinesOffsets = horizontalLinesOffsets;
}

void PreRectifiedBillViewEditor::afterProjectChange(StereoVisionApp::Project* op) {
    if (op != nullptr) {
        disconnect(op, &StereoVisionApp::Project::modelAboutToBeReset,
                   this, &PreRectifiedBillViewEditor::clearCombobox);
        disconnect(op, &StereoVisionApp::Project::modelReset,
                   this, &PreRectifiedBillViewEditor::setComboboxIndices);
    }

    if (activeProject() != nullptr) {
        connect(activeProject(), &StereoVisionApp::Project::modelAboutToBeReset,
                this, &PreRectifiedBillViewEditor::clearCombobox);
        connect(activeProject(), &StereoVisionApp::Project::modelReset,
                this, &PreRectifiedBillViewEditor::setComboboxIndices, Qt::QueuedConnection);
    }

    if (activeProject() != op) {
        clearCombobox();
    }

    setComboboxIndices();
}

void PreRectifiedBillViewEditor::addPoint(QPointF const& imageCoordinates) {

    QPointF bilSequenceCoordinates = mapToOriginalSequence(imageCoordinates);

    StereoVisionApp::Project* p = activeProject();

    if (p == nullptr) {
        return;
    }

    if (_sequence == nullptr) {
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

    //we can add multiple bil sequence landmarks for the same point
    _sequence->addBilSequenceLandmark(bilSequenceCoordinates, lmId, false, 1.0);


}
void PreRectifiedBillViewEditor::openPointMenu(qint64 id,
                                               QImageDisplay::ImageWidget *widget,
                                               QPoint const& pos) {
    if (widget != _viewWidget) {
        return;
    }

    if (_sequence == nullptr) {
        return;
    }

    QMenu cMenu;

    QAction* rm = new QAction(tr("remove"), &cMenu);

    connect(rm, &QAction::triggered, [this, id] () {
        _sequence->clearBilSequenceLandmark(id);
    });

    cMenu.addAction(rm);

    cMenu.exec(_viewWidget->mapToGlobal(pos));
}
void PreRectifiedBillViewEditor::openNonPointMenu(QPoint const& pos) {

    QMenu cMenu;

    cMenu.addAction(_moveToNextLandmark);
    cMenu.addAction(_moveToPrevLandmark);

    cMenu.exec(_viewWidget->mapToGlobal(pos));
}

void PreRectifiedBillViewEditor::moveToNextLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();
    comboId += 1;

    if (comboId >= _currentLandmarkMenu->count()) {
        comboId = -1;
    }

    _currentLandmarkMenu->setCurrentIndex(comboId);
}
void PreRectifiedBillViewEditor::moveToPreviousLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();

    if (comboId == -1) {
        comboId = _currentLandmarkMenu->count();
    }

    comboId -= 1;

    _currentLandmarkMenu->setCurrentIndex(comboId);
}

void PreRectifiedBillViewEditor::onCurrentLandmarkIndexChanged() {

    StereoVisionApp::Project* p = activeProject();

    if (p == nullptr) {
        return;
    }

    QModelIndex rootLm = p->indexOfClass(StereoVisionApp::Landmark::staticMetaObject.className());
    QModelIndex itemIndex = p->index(_currentLandmarkMenu->currentIndex(), 0, rootLm);

    if (itemIndex == QModelIndex()) {
        _img_landmark_overlay->displaySinglePoint(-1);
        return;
    } else {
        _current_landmark_id = itemIndex.data(StereoVisionApp::Project::IdRole).toInt();
    }
}

void PreRectifiedBillViewEditor::clearCombobox() {

    _currentLandmarkMenu->blockSignals(true);
    _currentLandmarkMenu->setCurrentIndex(-1);
    _currentLandmarkMenu->clear();
    _currentLandmarkMenu->blockSignals(false);
}

void PreRectifiedBillViewEditor::setComboboxIndices() {

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

PreRectifiedBillViewEditorFactory::PreRectifiedBillViewEditorFactory(QObject *parent) :
    StereoVisionApp::EditorFactory(parent)
{

}

QString PreRectifiedBillViewEditorFactory::TypeDescrName() const {
    return tr("Pre-rectified BIL cube viewer");
}
QString PreRectifiedBillViewEditorFactory::itemClassName() const {
    return PreRectifiedBillViewEditor::staticMetaObject.className();
}
StereoVisionApp::Editor* PreRectifiedBillViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new PreRectifiedBillViewEditor(parent);
}

} // namespace PikaLTools
