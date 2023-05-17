#ifndef PIKALTOOLS_BILSEQUENCELANDMARKSOVERLAY_H
#define PIKALTOOLS_BILSEQUENCELANDMARKSOVERLAY_H

#include <qImageDisplayWidget/overlay.h>

#include <QMap>

namespace QImageDisplay {
    class ImageWidget;
}

namespace PikaLTools {

class BilSequenceAcquisitionData;

class BilSequenceLandmarksOverlay : public QImageDisplay::Overlay
{
    Q_OBJECT
public:
    BilSequenceLandmarksOverlay(QWidget *parent = nullptr);

    inline bool hasBilSequenceDatablock() const {
        return _currentDataBlock != nullptr;
    }

    inline BilSequenceAcquisitionData* currentBilSequenceDatablock() const {
        return _currentDataBlock;
    }

    void setBilSequenceDatablock(BilSequenceAcquisitionData* seq);

    bool eventFilter(QObject *watched, QEvent *event) override;

    inline qint64 activePoint() const {
        return _activePoint;
    }

    void setActivePoint(const qint64 &ap);

    inline void displaySinglePoint(qint64 pointId) {
        if (pointId != _drawOnlyPoint) {
            _drawOnlyPoint = pointId;
            requestFullRepainting();
        }
    }
    inline qint64 singleDisplayedPoint() const {
        return _drawOnlyPoint;
    }

    inline int initialLine() const {
        return _initialLine;
    }

    void setInitialLine(int initialLine);

    inline int finalLine() const {
        return _finalLine;
    }

    void setFinalLine(int finalLine);

    inline int bilWidth() const {
        return _bilWidth;
    }

    void setBilWidth(int bilWidth);

Q_SIGNALS:

    void newPointClick(QPointF imagePos);
    void menuPointClick(qint64 id, QImageDisplay::ImageWidget* widget, QPoint const& pos);

    void activePointChanged(qint64 activePt);

protected:

    void requestFullRepainting();

    void paintItemImpl(QPainter* painter) const override;

    void registerLandmark(qint64 id);
    void unregisterLandmark(qint64 id);
    void unregisterAllLandmark();

    void imageDeleted();
    void imageLandmarkAdded(qint64 id);
    void imageLandmarkDeleted(qint64 id);

    void drawLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName) const;
    void drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName) const;

    bool mousePressEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);
    bool mouseReleaseEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);
    bool mouseMoveEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);

    qint64 pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const;


    BilSequenceAcquisitionData* _currentDataBlock;

    QMap<qint64, QList<QMetaObject::Connection>> _landmarkConnections;

    qint64 _activePoint;
    qint64 _drawOnlyPoint;

    Qt::MouseButtons _previously_pressed;
    bool _control_pressed_when_clicked;

    QPoint _motion_origin_pos;
    QPoint _motion_origin_t;

    int _initialLine;
    int _finalLine;
    int _bilWidth;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILSEQUENCELANDMARKSOVERLAY_H
