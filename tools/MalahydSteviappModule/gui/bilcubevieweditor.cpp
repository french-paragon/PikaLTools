#include "bilcubevieweditor.h"

#include <steviapp/gui/imagewidget.h>
#include <steviapp/control/mainwindow.h>

#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCommonStyle>
#include <QGroupBox>
#include <QFormLayout>
#include <QCheckBox>
#include <QToolButton>

#include <QMenu>
#include <QInputDialog>

#include "../datablocks/bilacquisitiondata.h"
#include "./trajectoryvieweditor.h"

#include "./imageOverlays/bilsequencelandmarksoverlay.h"

namespace PikaLTools {

BilCubeViewEditor::BilCubeViewEditor(QWidget* parent) :
    StereoVisionApp::Editor(parent),
    _currentSequence(nullptr),
    _sequence_nLines(0),
    _displayAdapter(nullptr),
    _current_landmark_id(-1)
{
    QCommonStyle style;
    _viewWidget = new StereoVisionApp::ImageWidget(this);

    _channels = {0,0,0};

    QLabel* startLineLabel = new QLabel(this);
    startLineLabel->setText(tr("Start line:"));

    _startLineSpinBox = new QSpinBox(this);
    _startLineSpinBox->setMinimum(0);
    _startLineSpinBox->setMaximum(1);
    _startLineSpinBox->setValue(0);

    QLabel* endLineLabel = new QLabel(this);
    endLineLabel->setText(tr("End line:"));

    _endLineSpinBox = new QSpinBox(this);
    _endLineSpinBox->setMinimum(0);
    _endLineSpinBox->setMaximum(1);
    _endLineSpinBox->setValue(1);

    _startLineSpinBox->setEnabled(false);
    _endLineSpinBox->setEnabled(false);

    connect(_startLineSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this] (int value) {
        _endLineSpinBox->setMinimum(value+1);
    });

    connect(_endLineSpinBox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this] (int value) {
        _startLineSpinBox->setMaximum(value-1);
    });

    QPushButton* commitLinesButton = new QPushButton(this);
    commitLinesButton->setText(tr("Set lines"));

    _displayOptionButton = new QToolButton(this);

    _displayOptionButton->setText("Display options");
    _displayOptionButton->setIcon(style.standardIcon(QStyle::SP_ArrowDown));
    _displayOptionButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

    connect(_displayOptionButton, &QPushButton::pressed, this, &BilCubeViewEditor::showDisplayOptionPopup);

    setupDisplayOptionPopup();

    QLabel* activeLandmarkLabel = new QLabel(this);
    activeLandmarkLabel->setText(tr("Active landmark:"));

    _currentLandmarkMenu = new QComboBox(this);

    QHBoxLayout* toolsLayout = new QHBoxLayout();

    toolsLayout->addWidget(startLineLabel);
    toolsLayout->addWidget(_startLineSpinBox);

    toolsLayout->addSpacing(5);

    toolsLayout->addWidget(endLineLabel);
    toolsLayout->addWidget(_endLineSpinBox);

    toolsLayout->addSpacing(5);

    toolsLayout->addWidget(commitLinesButton);

    toolsLayout->addStretch();

    toolsLayout->addWidget(_displayOptionButton);

    toolsLayout->addSpacing(5);

    toolsLayout->addWidget(activeLandmarkLabel);
    toolsLayout->addWidget(_currentLandmarkMenu);

    toolsLayout->setMargin(3);
    toolsLayout->setSpacing(0);

    connect(commitLinesButton, &QPushButton::pressed, this, &BilCubeViewEditor::commit_lines_changes);

    //main layout
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addLayout(toolsLayout);
    layout->addWidget(_viewWidget);

    layout->setSpacing(0);
    layout->setMargin(0);

    setLayout(layout);

    _displayOverlay = new BilSequenceLandmarksOverlay(_viewWidget);

    _viewWidget->addOverlay(_displayOverlay);
    _viewWidget->installEventFilter(_displayOverlay);

    connect(_displayOverlay, &BilSequenceLandmarksOverlay::newPointClick, this, &BilCubeViewEditor::addPoint);
    connect(_displayOverlay, &BilSequenceLandmarksOverlay::menuPointClick, this, &BilCubeViewEditor::openPointMenu);
    connect(_viewWidget, &StereoVisionApp::ImageWidget::menuNonPointClick, this, &BilCubeViewEditor::openNonPointMenu);

    _moveToNextLandmark = new QAction(tr("Next landmark"), this);
    _moveToPrevLandmark = new QAction(tr("Previous landmark"), this);

    _moveToNextLandmark->setShortcut(Qt::CTRL + Qt::Key_Right);
    _moveToPrevLandmark->setShortcut(Qt::CTRL + Qt::Key_Left);

    connect(_moveToNextLandmark, &QAction::triggered, this, &BilCubeViewEditor::moveToNextLandmark);
    connect(_moveToPrevLandmark, &QAction::triggered, this, &BilCubeViewEditor::moveToPreviousLandmark);

    addAction(_moveToNextLandmark);
    addAction(_moveToPrevLandmark);

}

void BilCubeViewEditor::setSequence(BilSequenceAcquisitionData* sequence) {

    clearSequence();

    if (sequence != nullptr) {

        _sequence_nLines = sequence->nLinesInSequence();

        if (_sequence_nLines <= 0) {
            clearSequence();
            return;
        }

        _startLineSpinBox->blockSignals(true);
        _endLineSpinBox->blockSignals(true);

        _startLineSpinBox->setValue(0);
        _endLineSpinBox->setMaximum(_sequence_nLines);
        _endLineSpinBox->setValue(std::min(1000, _sequence_nLines)); //load 1000 lines max initially

        _endLineSpinBox->setMinimum(_startLineSpinBox->value()+1);
        _startLineSpinBox->setMaximum(_endLineSpinBox->value()-1);

        _startLineSpinBox->blockSignals(false);
        _endLineSpinBox->blockSignals(false);

        BilSequenceAcquisitionData::BilAcquisitionData data = sequence->getBilInfos().first();

        QMap<QString, QString> headerData = data.headerData();

        _blackLevel = 0;

        bool ok;
        _whiteLevel = headerData.value("ceiling", "1.0").toFloat(&ok);

        if (!ok) {

            _whiteLevel = 1.0;
        }

        _blackLevelChannelSpinBox->setMaximum(2*_whiteLevel);
        _whiteLevelChannelSpinBox->setMaximum(2*_whiteLevel);

        _blackLevelChannelSpinBox->setValue(_blackLevel);
        _whiteLevelChannelSpinBox->setValue(_whiteLevel);

        _autoBWLevelCheckbox->setEnabled(true);

        QVector<float> waveLengths;

        if (headerData.contains("wavelength")) {
            QString elements = headerData["wavelength"].mid(1,headerData["wavelength"].size()-2);
            QStringList splittedElements = elements.split(",");
            waveLengths.reserve(splittedElements.size());

            for (QString const& element : splittedElements) {
                bool ok;
                float value = element.toFloat(&ok);

                if (ok) {
                    waveLengths.push_back(value);
                }
            }
        }

        int nSpetrcalBands = headerData.value("bands", "0").toInt(&ok);

        if (!ok) {
            clearSequence();
            return;
        }

        _channels = {nSpetrcalBands/4,
                     nSpetrcalBands/2,
                     nSpetrcalBands-nSpetrcalBands/4};

        if (_channels[2] >= nSpetrcalBands) {
            _channels[2] = nSpetrcalBands-1;
        }

        if (waveLengths.size() == nSpetrcalBands) {

            std::array<float, 3> referenceWl = {630, 532, 465};
            std::array<float, 3> currentWl = {waveLengths[_channels[0]], waveLengths[_channels[1]], waveLengths[_channels[2]]};


            for (int i = 0; i < waveLengths.size(); i++) {
                for (int c = 0; c < 3; c++) {

                    float candDelta = std::fabs(waveLengths[i] - referenceWl[c]);
                    float currentDelta = std::fabs(currentWl[c] - referenceWl[c]);

                    if (candDelta < currentDelta) {
                        currentWl[c] = waveLengths[i];
                        _channels[c] = i;
                    }
                }
            }

        }

        _redChannelSpinBox->setMaximum(nSpetrcalBands);
        _greenChannelSpinBox->setMaximum(nSpetrcalBands);
        _blueChannelSpinBox->setMaximum(nSpetrcalBands);

        _redChannelSpinBox->setValue(_channels[0]);
        _greenChannelSpinBox->setValue(_channels[1]);
        _blueChannelSpinBox->setValue(_channels[2]);

        _startLineSpinBox->setEnabled(true);
        _endLineSpinBox->setEnabled(true);

        _redChannelSpinBox->setEnabled(true);
        _greenChannelSpinBox->setEnabled(true);
        _blueChannelSpinBox->setEnabled(true);

        _currentSequence = sequence;
        _displayOverlay->setBilSequenceDatablock(_currentSequence);

        connect(_currentSequence, &QObject::destroyed, this, &BilCubeViewEditor::clearSequence);

        commit_lines_changes();
        commit_channels_changes();
    }

}
void BilCubeViewEditor::clearSequence() {

    if (_currentSequence != nullptr) {
        if (_displayAdapter != nullptr) {
            _viewWidget->setImage(nullptr);
            _displayAdapter->deleteLater();
        }

        disconnect(_currentSequence, &QObject::destroyed, this, &BilCubeViewEditor::clearSequence);
        _currentSequence = nullptr;
        _displayOverlay->setBilSequenceDatablock(nullptr);

        _startLineSpinBox->setEnabled(false);
        _endLineSpinBox->setEnabled(false);

        _redChannelSpinBox->setEnabled(false);
        _greenChannelSpinBox->setEnabled(false);
        _blueChannelSpinBox->setEnabled(false);

        _blackLevelChannelSpinBox->setEnabled(false);
        _whiteLevelChannelSpinBox->setEnabled(false);
    }

}

void BilCubeViewEditor::setupDisplayOptionPopup() {

    _displayOptionPopup = new QWidget(this);
    _displayOptionPopup->setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
    _displayOptionPopup->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    QVBoxLayout* displayOptionLayout = new QVBoxLayout(_displayOptionPopup);

    _redChannelSpinBox = new QSpinBox(_displayOptionPopup);
    _redChannelSpinBox->setMinimum(0);
    _redChannelSpinBox->setMaximum(1);
    _redChannelSpinBox->setValue(_channels[0]);

    _greenChannelSpinBox = new QSpinBox(_displayOptionPopup);
    _greenChannelSpinBox->setMinimum(0);
    _greenChannelSpinBox->setMaximum(1);
    _greenChannelSpinBox->setValue(_channels[0]);

    _blueChannelSpinBox = new QSpinBox(_displayOptionPopup);
    _blueChannelSpinBox->setMinimum(0);
    _blueChannelSpinBox->setMaximum(1);
    _blueChannelSpinBox->setValue(_channels[0]);

    _autoBWLevelCheckbox = new QCheckBox(_displayOptionPopup);
    _autoBWLevelCheckbox->setTristate(false);
    _autoBWLevelCheckbox->setChecked(false);

    _blackLevelChannelSpinBox = new QDoubleSpinBox(_displayOptionPopup);
    _blackLevelChannelSpinBox->setMinimum(0);
    _blackLevelChannelSpinBox->setMaximum(2);
    _blackLevelChannelSpinBox->setValue(0);
    _blackLevelChannelSpinBox->setDecimals(2);

    _whiteLevelChannelSpinBox = new QDoubleSpinBox(_displayOptionPopup);
    _whiteLevelChannelSpinBox->setMinimum(0);
    _whiteLevelChannelSpinBox->setMaximum(2);
    _whiteLevelChannelSpinBox->setValue(0);
    _whiteLevelChannelSpinBox->setDecimals(2);

    QGroupBox* channelGroupBox = new QGroupBox(_displayOptionPopup);
    channelGroupBox->setTitle(tr("Channels"));
    QFormLayout* channelLayout = new QFormLayout(channelGroupBox);

    channelLayout->addRow(tr("Red:"), _redChannelSpinBox);
    channelLayout->addRow(tr("Green:"), _greenChannelSpinBox);
    channelLayout->addRow(tr("Blue:"), _blueChannelSpinBox);

    displayOptionLayout->addWidget(channelGroupBox);

    QGroupBox* levelsGroupBox = new QGroupBox(_displayOptionPopup);
    levelsGroupBox->setTitle(tr("Levels"));
    QFormLayout* levelsLayout = new QFormLayout(levelsGroupBox);

    levelsLayout->addRow(tr("Auto:"), _autoBWLevelCheckbox);
    levelsLayout->addRow(tr("Black:"), _blackLevelChannelSpinBox);
    levelsLayout->addRow(tr("White:"), _whiteLevelChannelSpinBox);

    displayOptionLayout->addWidget(levelsGroupBox);

    QPushButton* commitVisualChanges = new QPushButton(_displayOptionPopup);
    commitVisualChanges->setText(tr("Commit"));

    displayOptionLayout->addWidget(commitVisualChanges);

    //editing disabled by default

    _redChannelSpinBox->setEnabled(false);
    _greenChannelSpinBox->setEnabled(false);
    _blueChannelSpinBox->setEnabled(false);

    _autoBWLevelCheckbox->setEnabled(false);
    _autoBWLevelCheckbox->setChecked(false);

    connect(_autoBWLevelCheckbox, &QCheckBox::stateChanged, this, [this] () {
        if (_autoBWLevelCheckbox->isChecked()) {

            _blackLevelChannelSpinBox->setEnabled(false);
            _whiteLevelChannelSpinBox->setEnabled(false);
        }

        _blackLevelChannelSpinBox->setEnabled(true);
        _whiteLevelChannelSpinBox->setEnabled(true);
    });

    _blackLevelChannelSpinBox->setEnabled(true);
    _whiteLevelChannelSpinBox->setEnabled(true);

    connect(commitVisualChanges, &QPushButton::pressed, this, [this] () {
        commit_channels_changes();
        commit_bwLevels_changes();
    });

}

void BilCubeViewEditor::showDisplayOptionPopup() {

    QPoint globalPos = _displayOptionButton->mapToGlobal(QPoint(0,0));
    _displayOptionPopup->show();
    _displayOptionPopup->move(globalPos.x() + _displayOptionButton->width() - _displayOptionPopup->width(),
                              globalPos.y() + _displayOptionButton->height());

}

void BilCubeViewEditor::commit_lines_changes() {

    if (_displayAdapter != nullptr) {
        _viewWidget->setImage(nullptr);
        _displayAdapter->deleteLater();
    }

    if (_currentSequence == nullptr) {
        _displayAdapter = nullptr;
        return;
    }

    int sLine = _startLineSpinBox->value();
    int eLine = _endLineSpinBox->value();

    _endLineSpinBox->setMinimum(_startLineSpinBox->value()+1);
    _startLineSpinBox->setMaximum(_endLineSpinBox->value()-1);

    _channels[0] = _redChannelSpinBox->value();
    _channels[1] = _greenChannelSpinBox->value();
    _channels[2] = _blueChannelSpinBox->value();

    _bil_data = _currentSequence->getFloatBilData(sLine, eLine, std::vector<int>(_channels.begin(), _channels.end()));

    _displayAdapter = new HyperspectralSimplePseudocolorDisplayAdapter<float>(&_bil_data, _blackLevel, _whiteLevel);

    _displayAdapter->setChannels({0,1,2});

    commit_bwLevels_changes();

    _viewWidget->setImage(_displayAdapter);

    _displayOverlay->setInitialLine(sLine);
    _displayOverlay->setFinalLine(eLine);
    _displayOverlay->setBilWidth(_bil_data.shape()[SamplesAxis]);

    StereoVisionApp::MainWindow* mw = StereoVisionApp::MainWindow::getActiveMainWindow();

    if (mw != nullptr) {
        StereoVisionApp::Editor* e = mw->openedEditor(TrajectoryViewEditor::staticMetaObject.className());
        TrajectoryViewEditor* trjve = qobject_cast<TrajectoryViewEditor*>(e);

        if (trjve != nullptr) {
            trjve->setStartLine(sLine);
            trjve->setEndLine(eLine);
        }
    }

}
void BilCubeViewEditor::commit_channels_changes() {

    if (_displayAdapter == nullptr) {
        return;
    }

    if (_channels[0] == _redChannelSpinBox->value()
            and _channels[1] == _greenChannelSpinBox->value()
            and _channels[2] == _blueChannelSpinBox->value()) {

        //no need to change anything
        return;

    }

    _channels[0] = _redChannelSpinBox->value();
    _channels[1] = _greenChannelSpinBox->value();
    _channels[2] = _blueChannelSpinBox->value();

    commit_lines_changes();

}

void BilCubeViewEditor::commit_bwLevels_changes() {

    if (_displayAdapter == nullptr) {
        return;
    }

    bool autoLevel = _autoBWLevelCheckbox->isChecked();

    if (!autoLevel) {
        _blackLevel = _blackLevelChannelSpinBox->value();
        _whiteLevel = _whiteLevelChannelSpinBox->value();

        _displayAdapter->setBlackAndWhiteLevel(_blackLevel, _whiteLevel);
        return;
    }

    constexpr int nChannels = 3;

    std::array<float,nChannels> blackLevels;
    std::array<float,nChannels> whiteLevels;

    int nPoints = _bil_data.shape()[0]*_bil_data.shape()[1];

    for (int c = 0; c < nChannels; c++) {
        std::vector<float> values;
        values.reserve(nPoints);

        for (int i = 0; i < _bil_data.shape()[0]; i++) {
            for (int j = 0; j < _bil_data.shape()[1]; j++) {
                values.push_back(_bil_data.valueUnchecked(i,j,c));
            }
        }

        int bLevelId = 0.05*nPoints;
        int wLevelId = 0.95*nPoints;

        if (wLevelId >= nPoints) {
            wLevelId = nPoints-1;
        }

        if (bLevelId == wLevelId) {
            bLevelId = 0;
            wLevelId = nPoints-1;
        }

        auto bLevelElement = values.begin();
        std::advance(bLevelElement, bLevelId);
        auto wLevelElement = values.begin();
        std::advance(wLevelElement, wLevelId);

        std::nth_element(values.begin(), bLevelElement, values.end());
        std::nth_element(values.begin(), wLevelElement, values.end());

        blackLevels[c] = *bLevelElement;
        whiteLevels[c] = *wLevelElement;
    }

    _displayAdapter->setBlackAndWhiteLevel(blackLevels, whiteLevels);

}

void BilCubeViewEditor::afterProjectChange(StereoVisionApp::Project* op) {
    if (op != nullptr) {
        disconnect(op, &StereoVisionApp::Project::modelAboutToBeReset, this, &BilCubeViewEditor::clearCombobox);
        disconnect(op, &StereoVisionApp::Project::modelReset, this, &BilCubeViewEditor::setComboboxIndices);
    }

    if (activeProject() != nullptr) {
        connect(activeProject(), &StereoVisionApp::Project::modelAboutToBeReset, this, &BilCubeViewEditor::clearCombobox);
        connect(activeProject(), &StereoVisionApp::Project::modelReset, this, &BilCubeViewEditor::setComboboxIndices, Qt::QueuedConnection);
    }

    if (activeProject() != op) {
        clearCombobox();
    }

    setComboboxIndices();
}

void BilCubeViewEditor::addPoint(QPointF const& imageCoordinates) {

    StereoVisionApp::Project* p = activeProject();

    if (p == nullptr) {
        return;
    }

    if (_currentSequence == nullptr) {
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
    _currentSequence->addBilSequenceLandmark(imageCoordinates, lmId, false, 1.0);

}
void BilCubeViewEditor::openPointMenu(qint64 id, QImageDisplay::ImageWidget *widget, QPoint const& pos) {

    if (widget != _viewWidget) {
        return;
    }

    if (_currentSequence == nullptr) {
        return;
    }

    QMenu cMenu;

    QAction* rm = new QAction(tr("remove"), &cMenu);

    connect(rm, &QAction::triggered, [this, id] () {
        _currentSequence->clearBilSequenceLandmark(id);
    });

    cMenu.addAction(rm);

    cMenu.exec(_viewWidget->mapToGlobal(pos));

}
void BilCubeViewEditor::openNonPointMenu(QPoint const& pos) {

    QMenu cMenu;

    cMenu.addAction(_moveToNextLandmark);
    cMenu.addAction(_moveToPrevLandmark);

    cMenu.exec(_viewWidget->mapToGlobal(pos));


}

void BilCubeViewEditor::moveToNextLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();
    comboId += 1;

    if (comboId >= _currentLandmarkMenu->count()) {
        comboId = -1;
    }

    _currentLandmarkMenu->setCurrentIndex(comboId);

}
void BilCubeViewEditor::moveToPreviousLandmark() {

    int comboId = _currentLandmarkMenu->currentIndex();

    if (comboId == -1) {
        comboId = _currentLandmarkMenu->count();
    }

    comboId -= 1;

    _currentLandmarkMenu->setCurrentIndex(comboId);

}

void BilCubeViewEditor::onCurrentLandmarkIndexChanged() {


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

    /*if (ui->viewSingleLandmarkButton->isChecked()) {
        _displayOverlay->displaySinglePoint(_current_landmark_id);
    } else {
        _displayOverlay->displaySinglePoint(-1);
    }*/
}

void BilCubeViewEditor::clearCombobox() {

    _currentLandmarkMenu->blockSignals(true);
    _currentLandmarkMenu->setCurrentIndex(-1);
    _currentLandmarkMenu->clear();
    _currentLandmarkMenu->blockSignals(false);

}
void BilCubeViewEditor::setComboboxIndices() {

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

BilCubeViewEditorFactory::BilCubeViewEditorFactory(QObject *parent) {

}

QString BilCubeViewEditorFactory::TypeDescrName() const {
    return tr("Hyperspectral BIL cube viewer");
}

QString BilCubeViewEditorFactory::itemClassName() const {
    return BilCubeViewEditor::staticMetaObject.className();
}

StereoVisionApp::Editor* BilCubeViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new BilCubeViewEditor(parent);
}

} // namespace PikaLTools
