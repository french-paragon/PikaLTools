#ifndef PIKALTOOLS_BILCUBEVIEWEDITOR_H
#define PIKALTOOLS_BILCUBEVIEWEDITOR_H


#include <steviapp/gui/editor.h>

#include <StereoVision/geometry/rotations.h>

#include "../../libs/gui/hyperspectralsimplepseudocolordisplayadapter.h"

class QSpinBox;
class QDoubleSpinBox;
class QComboBox;

namespace StereoVisionApp {
    class ImageWidget;
}

namespace QImageDisplay {
    class ImageWidget;
}

namespace PikaLTools {

class BilSequenceAcquisitionData;
class BilSequenceLandmarksOverlay;

class BilCubeViewEditor : public StereoVisionApp::Editor
{
    Q_OBJECT
public:
    BilCubeViewEditor(QWidget* parent = nullptr);

    void setSequence(BilSequenceAcquisitionData* sequence);
    void clearSequence();

protected:

    void commit_lines_changes();
    void commit_channels_changes();
    void commit_bwLevels_changes();

    void afterProjectChange(StereoVisionApp::Project* op) override;

    void addPoint(QPointF const& imageCoordinates);
    void openPointMenu(qint64 id, QImageDisplay::ImageWidget *widget, QPoint const& pos);
    void openNonPointMenu(QPoint const& pos);

    void moveToNextLandmark();
    void moveToPreviousLandmark();

    void onCurrentLandmarkIndexChanged();

    void clearCombobox();
    void setComboboxIndices();

    BilSequenceAcquisitionData* _currentSequence;
    int _sequence_nLines;

    QSpinBox* _startLineSpinBox;
    QSpinBox* _endLineSpinBox;

    QSpinBox* _redChannelSpinBox;
    QSpinBox* _greenChannelSpinBox;
    QSpinBox* _blueChannelSpinBox;

    QDoubleSpinBox* _blackLevelChannelSpinBox;
    QDoubleSpinBox* _whiteLevelChannelSpinBox;

    QComboBox* _currentLandmarkMenu;

    StereoVisionApp::ImageWidget* _viewWidget;

    Multidim::Array<float, 3> _bil_data;
    float _whiteLevel;
    float _blackLevel;

    HyperspectralSimplePseudocolorDisplayAdapter<float>* _displayAdapter;
    BilSequenceLandmarksOverlay* _displayOverlay;



    QAction* _moveToNextLandmark;
    QAction* _moveToPrevLandmark;

    qint64 _current_landmark_id;
};


class BilCubeViewEditorFactory : public StereoVisionApp::EditorFactory
{
    Q_OBJECT
public:

    explicit BilCubeViewEditorFactory(QObject *parent = nullptr);

    QString TypeDescrName() const override;
    QString itemClassName() const override;
    StereoVisionApp::Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILCUBEVIEWEDITOR_H
