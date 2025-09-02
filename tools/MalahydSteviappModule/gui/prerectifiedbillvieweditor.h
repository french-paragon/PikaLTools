#ifndef PRERECTIFIEDBILLVIEWEDITOR_H
#define PRERECTIFIEDBILLVIEWEDITOR_H

#include <steviapp/gui/editor.h>
#include <steviapp/gui/imageadapters/imagedatadisplayadapter.h>

class QComboBox;

namespace QImageDisplay {
    class ImageWidget;
}

namespace StereoVisionApp {

class Image;
class ImageWidget;

} // namespace StereoVisionApp

namespace PikaLTools {

class BilSequenceAcquisitionData;
class BilSequenceLandmarksOverlay;

class PreRectifiedBillViewEditor : public StereoVisionApp::Editor
{
    Q_OBJECT
public:
    explicit PreRectifiedBillViewEditor(QWidget *parent = nullptr);
    ~PreRectifiedBillViewEditor();

    template<typename T>
    void setImageFromArray(BilSequenceAcquisitionData* sequence,
                           QVector<int> const& verticalLinesMatch,
                           QVector<float> const& horizontalLinesOffsets,
                           Multidim::Array<T,3> const& arr,
                           T whiteLevel,
                           T blackLevel,
                           bool logConvert = false) {

        setSequenceData(sequence,
                        verticalLinesMatch,
                        horizontalLinesOffsets,
                        arr.shape());
        _img_display_adapter->setImageFromArray(arr, whiteLevel, blackLevel, logConvert);
    }

    QPointF mapToOriginalSequence(QPointF const& source) const;
    QPointF mapFromOriginalSequence(QPointF const& source) const;


Q_SIGNALS:

protected:

    void setSequenceData(BilSequenceAcquisitionData* sequence,
                         QVector<int> const& verticalLinesMatch,
                         QVector<float> const& horizontalLinesOffsets,
                         std::array<int,3> const& imgShape);

    void afterProjectChange(StereoVisionApp::Project* op) override;

    void addPoint(QPointF const& imageCoordinates);
    void openPointMenu(qint64 id, QImageDisplay::ImageWidget *widget, QPoint const& pos);
    void openNonPointMenu(QPoint const& pos);

    void moveToNextLandmark();
    void moveToPreviousLandmark();

    void onCurrentLandmarkIndexChanged();

    void clearCombobox();
    void setComboboxIndices();

private:

    StereoVisionApp::ImageDataDisplayAdapter* _img_display_adapter;
    BilSequenceLandmarksOverlay* _img_landmark_overlay;

    QComboBox* _currentLandmarkMenu;

    StereoVisionApp::ImageWidget* _viewWidget;

    QAction* _moveToNextLandmark;
    QAction* _moveToPrevLandmark;

    qint64 _current_landmark_id;

    BilSequenceAcquisitionData* _sequence;

    QVector<int> _verticalLinesMatch;
    QVector<float> _horizontalLinesOffsets;
};


class PreRectifiedBillViewEditorFactory : public StereoVisionApp::EditorFactory
{
    Q_OBJECT
public:

    explicit PreRectifiedBillViewEditorFactory(QObject *parent = nullptr);

    QString TypeDescrName() const override;
    QString itemClassName() const override;
    StereoVisionApp::Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace PikaLTools

#endif // PRERECTIFIEDBILLVIEWEDITOR_H
