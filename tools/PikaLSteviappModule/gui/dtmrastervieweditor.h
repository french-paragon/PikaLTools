#ifndef PIKALTOOLS_DTMRASTERVIEWEDITOR_H
#define PIKALTOOLS_DTMRASTERVIEWEDITOR_H

#include <steviapp/gui/editor.h>

#include <StereoVision/gui/arraydisplayadapter.h>

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

class InputDtm;
class InputDtmLandmarksOverlay;

class DTMRasterViewEditor : public StereoVisionApp::Editor
{
    Q_OBJECT
public:
    explicit DTMRasterViewEditor(QWidget *parent = nullptr);

    void setDtm(InputDtm* dtm);
    void clearDtm();

Q_SIGNALS:

protected:

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

    void recomputeBlackAndWhiteLevel(bool all = true);

    InputDtm* _currentDtm;

    QDoubleSpinBox* _blackLevelChannelSpinBox;
    QDoubleSpinBox* _whiteLevelChannelSpinBox;

    QComboBox* _currentLandmarkMenu;

    StereoVisionApp::ImageWidget* _viewWidget;

    Multidim::Array<float, 2> _bil_data;
    float _whiteLevel;
    float _blackLevel;

    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _displayAdapter;
    InputDtmLandmarksOverlay* _displayOverlay;



    QAction* _moveToNextLandmark;
    QAction* _moveToPrevLandmark;

    qint64 _current_landmark_id;

};


class DTMRasterViewEditorFactory : public StereoVisionApp::EditorFactory
{
    Q_OBJECT
public:

    explicit DTMRasterViewEditorFactory(QObject *parent = nullptr);

    QString TypeDescrName() const override;
    QString itemClassName() const override;
    StereoVisionApp::Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_DTMRASTERVIEWEDITOR_H
