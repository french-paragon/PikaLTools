#include "inputdtmlandmarksoverlay.h"

#include "datablocks/inputdtm.h"

#include <steviapp/datablocks/landmark.h>

#include <StereoVision/QImageDisplayWidget/imagewidget.h>

#include <QEvent>
#include <QMouseEvent>
#include <QGuiApplication>

namespace PikaLTools {

InputDtmLandmarksOverlay::InputDtmLandmarksOverlay(QWidget *parent) :
    QImageDisplay::Overlay(parent),
    _currentDataBlock(nullptr),
    _imageRescale(1),
    _activePoint(0),
    _drawOnlyPoint(-1)
{

}

void InputDtmLandmarksOverlay::setInputDtmDatablock(InputDtm* dtm) {

    if (_currentDataBlock != nullptr) {
        unregisterAllLandmark();

        disconnect(_currentDataBlock, &QObject::destroyed, this, &InputDtmLandmarksOverlay::imageDeleted);
        disconnect(_currentDataBlock, &InputDtm::pointAdded, this, &InputDtmLandmarksOverlay::imageLandmarkAdded);
        disconnect(_currentDataBlock, &InputDtm::pointRemoved, this, &InputDtmLandmarksOverlay::imageLandmarkDeleted);
    }

    _currentDataBlock = dtm;

    if (dtm != nullptr) {

        for (qint64 lmid : dtm->listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className())) {
            registerLandmark(lmid);
        }

        connect(_currentDataBlock, &QObject::destroyed, this, &InputDtmLandmarksOverlay::imageDeleted);
        connect(_currentDataBlock, &InputDtm::pointAdded, this, &InputDtmLandmarksOverlay::imageLandmarkAdded);
        connect(_currentDataBlock, &InputDtm::pointRemoved, this, &InputDtmLandmarksOverlay::imageLandmarkDeleted);
    }

    requestFullRepainting();

}

bool InputDtmLandmarksOverlay::eventFilter(QObject *watched, QEvent *event) {

    QImageDisplay::ImageWidget* widget = qobject_cast<QImageDisplay::ImageWidget*>(watched);

    if (widget != nullptr) {

        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mousePressEvent(widget, mouseEvent);
        }

        if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mouseReleaseEvent(widget, mouseEvent);
        }

        if (event->type() == QEvent::MouseMove) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mouseMoveEvent(widget, mouseEvent);
        }
    }

    return false;
}

void InputDtmLandmarksOverlay::setActivePoint(const qint64 &ap) {
    qint64 activePoint = ap;

    if (_currentDataBlock == nullptr) {
        activePoint = -1;
    } else if (qobject_cast<DtmLandmark*>(_currentDataBlock->getById(activePoint)) == nullptr) {
        activePoint = -1;
    }

    if (activePoint != _activePoint) {
        _activePoint = activePoint;
        requestFullRepainting();
    }
}

void InputDtmLandmarksOverlay::requestFullRepainting() {
    Q_EMIT repaintingRequested(QRect());
}

void InputDtmLandmarksOverlay::paintItemImpl(QPainter* painter) const {

    if (_currentDataBlock == nullptr) {
        return;
    }

    DtmLandmark* active = nullptr;

    QVector<qint64> pointsids = _currentDataBlock->listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());

    for(qint64 id : pointsids) {
        DtmLandmark* lm = _currentDataBlock->getDtmLandmark(id);

        if (id == _activePoint) {
            active = lm;
            continue;
        }

        if (_drawOnlyPoint >= 0 and _drawOnlyPoint != lm->attachedLandmarkid()) {
            continue;
        }

        auto coords = lm->imageCoordinates();

        drawLandmark(painter, coords, false, lm->attachedLandmarkName());

    }

    if (active != nullptr) {

        bool ok = false;

        auto coords = active->imageCoordinates();

        if (_drawOnlyPoint >= 0 and _drawOnlyPoint == active->attachedLandmarkid()) {
            ok = true;
        } else if (_drawOnlyPoint < 0) {
            ok = true;
        }

        if (ok) {
            drawLandmark(painter, coords, true, active->attachedLandmarkName());
        }
    }
}

void InputDtmLandmarksOverlay::registerLandmark(qint64 id) {

    if (_currentDataBlock == nullptr) {
        return;
    }

    DtmLandmark* dtmlm = _currentDataBlock->getDtmLandmark(id);
    StereoVisionApp::Landmark* lm = dtmlm->attachedLandmark();

    connect(dtmlm, &DtmLandmark::coordsChanged, this, &InputDtmLandmarksOverlay::requestFullRepainting);
    connect(lm, &QObject::objectNameChanged, this, &InputDtmLandmarksOverlay::requestFullRepainting);

}
void InputDtmLandmarksOverlay::unregisterLandmark(qint64 id) {

    if (_currentDataBlock == nullptr) {
        return;
    }

    DtmLandmark* dtmlm = _currentDataBlock->getDtmLandmark(id);
    StereoVisionApp::Landmark* lm = dtmlm->attachedLandmark();

    disconnect(dtmlm, nullptr, this, nullptr);
    disconnect(lm, nullptr, this, nullptr);

}
void InputDtmLandmarksOverlay::unregisterAllLandmark() {

    if (_currentDataBlock == nullptr) {
        return;
    }

    QVector<qint64> keys = _currentDataBlock->listTypedSubDataBlocks(DtmLandmark::staticMetaObject.className());

    for (qint64 id : keys) {
        unregisterLandmark(id);
    }
}

void InputDtmLandmarksOverlay::imageDeleted() {
    unregisterAllLandmark();
    _currentDataBlock = nullptr;
    requestFullRepainting();
}
void InputDtmLandmarksOverlay::imageLandmarkAdded(qint64 id) {
    registerLandmark(id);
    Q_EMIT repaintingRequested(QRect());
}
void InputDtmLandmarksOverlay::imageLandmarkDeleted(qint64 id) {
    unregisterLandmark(id);
    Q_EMIT repaintingRequested(QRect());
}

void InputDtmLandmarksOverlay::drawLandmark(QPainter* painter,
                                            QPointF const& imagePoint,
                                            bool isActive,
                                            const QString &ptn) const {

    QTransform img2paint = imageToPaintArea();

    QString ptName = ptn;

    if (ptName.isEmpty()) {
        ptName = "No name";
    }

    QPointF wCoord = img2paint.map(_imageRescale*imagePoint);
    int hPw = 12;
    QPointF hExtend(hPw, hPw);

    QColor pColor(106, 193, 74);
    QColor tColor(255, 255, 255);
    QColor black(0, 0, 0);

    if (isActive) {
        tColor = pColor;
        pColor = QColor(255, 255, 255);
    }

    QBrush fill;
    QPen line(black);
    line.setWidth(4);

    painter->setFont(QFont("sans", 8, QFont::Normal));
    int hMTw = 50;
    QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
    QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

    QRectF textBg(wCoord + QPointF(-hMTw, -hPw-b.height()) + b.topLeft(),
                  wCoord + QPointF(-hMTw, -hPw-b.height()) + b.bottomRight());

    QRectF outterRect(wCoord - hExtend, wCoord + hExtend);

    painter->setBrush(fill);
    painter->setPen(line);

    painter->drawRect(outterRect);
    painter->drawRect(textBg);
    painter->drawPoint(wCoord);

    line.setColor(pColor);
    line.setWidth(2);
    painter->setPen(line);

    painter->drawRect(outterRect);
    painter->drawPoint(wCoord);

    fill = QBrush(pColor);
    painter->setBrush(fill);

    painter->drawRect(textBg);

    line.setColor(tColor);
    painter->setPen(line);

    painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);

}
void InputDtmLandmarksOverlay::drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptn) const {

    QTransform img2paint = imageToPaintArea();
    QSize size = paintAreaSize();

    QString ptName = ptn;

    if (ptName.isEmpty()) {
        ptName = "No name";
    }

    QPointF wCoord = img2paint.map(_imageRescale*imagePoint);

    QPointF constrained = wCoord;

    if (constrained.x() < 0) {
        constrained.setX(0);
    }
    if (constrained.y() < 0) {
        constrained.setY(0);
    }
    if (constrained.x() > size.width()) {
        constrained.setX(size.width());
    }
    if (constrained.y() > size.height()) {
        constrained.setY(size.height());
    }

    int hPw = 12;

    QPointF hExtend(hPw, hPw);

    QColor pColor(191, 51, 26);
    QColor tColor(255, 255, 255);
    QColor black(0, 0, 0);

    if (isActive) {
        tColor = pColor;
        pColor = QColor(255, 255, 255);
    }

    QBrush fill;
    QPen line(black);
    line.setWidth(4);

    painter->setFont(QFont("sans", 8, QFont::Normal));
    int hMTw = 50;
    QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
    QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

    QPolygonF arrow;

    QRectF textBg;

    if (constrained.x() == size.width() and constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, size.height())
              << QPointF(size.width(), size.height() - hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == size.width() and constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, 0)
              << QPointF(size.width(), hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

    } else if (constrained.x() == 0 and constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(hPw, size.height())
              << QPointF(0, size.height() - hPw);

        textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

    }  else if (constrained.x() == 0 and constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(hPw, 0)
              << QPointF(0, hPw);

        textBg = QRectF(constrained + QPointF(hPw , hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == size.width()) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, constrained.y() - hPw)
              << QPointF(size.width() - hPw, constrained.y() + hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

    } else if (constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(constrained.x() - hPw, size.height() - hPw)
              << QPointF(constrained.x() + hPw, size.height() - hPw);

        textBg = QRectF(constrained + QPointF(-hMTw -hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(-hMTw -hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == 0) {
        arrow << QPointF(constrained)
              << QPointF(hPw, constrained.y() - hPw)
              << QPointF(hPw, constrained.y() + hPw);

        textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(constrained.x() - hPw, hPw)
              << QPointF(constrained.x() + hPw, hPw);

        textBg = QRectF(constrained + QPointF(-hMTw -hPw , hPw) + b.topLeft(),
                          constrained + QPointF(-hMTw -hPw, hPw) + b.bottomRight());
    }



    painter->setBrush(fill);
    painter->setPen(line);

    painter->drawConvexPolygon(arrow);
    painter->drawRect(textBg);
    painter->drawPoint(constrained);

    line.setColor(pColor);
    line.setWidth(2);
    painter->setPen(line);

    painter->drawConvexPolygon(arrow);
    painter->drawPoint(constrained);

    fill = QBrush(pColor);
    painter->setBrush(fill);

    painter->drawRect(textBg);

    line.setColor(tColor);
    painter->setPen(line);

    painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);

}

bool InputDtmLandmarksOverlay::mousePressEvent(QImageDisplay::ImageWidget* interactionWidget,
                                               QMouseEvent *e) {

    QGuiApplication* gapp = qGuiApp;

    if (gapp != nullptr) {
        Qt::KeyboardModifiers kmods = gapp->keyboardModifiers();

        if (kmods & Qt::ControlModifier) {
            _control_pressed_when_clicked = true;
        } else {
            _control_pressed_when_clicked = false;
        }
    }

    _previously_pressed = e->buttons();

    if (_previously_pressed == Qt::LeftButton) {

        _motion_origin_pos = e->pos();
        _motion_origin_t = interactionWidget->translation();

        qint64 pt = pointAt(interactionWidget, e->pos());
        setActivePoint(pt);

        if (pt >= 0) {
            _control_pressed_when_clicked = false;
            return true;
        }

    } else if (_previously_pressed == Qt::RightButton) {
        qint64 pt = pointAt(interactionWidget, e->pos());
        setActivePoint(pt);

        if (pt >= 0) {
            _control_pressed_when_clicked = false;
            Q_EMIT menuPointClick(pt, interactionWidget, e->pos());
            return true;
        }

    }

    return false;

}
bool InputDtmLandmarksOverlay::mouseReleaseEvent(QImageDisplay::ImageWidget* interactionWidget,
                                                 QMouseEvent *e) {

    if (_previously_pressed == Qt::LeftButton) {
        if (_control_pressed_when_clicked) {


            QPointF pos = interactionWidget->widgetToImageCoordinates(e->pos());

            emit newPointClick(pos);
            return true;
        }
    }

    return false;

}
bool InputDtmLandmarksOverlay::mouseMoveEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *e) {

    if (_currentDataBlock == nullptr) {
        return false;
    }

    if (_previously_pressed == Qt::LeftButton) {
        if (!_control_pressed_when_clicked) {
            if (_previously_pressed == Qt::LeftButton and _activePoint >= 0) {
                DtmLandmark* ilm = _currentDataBlock->getDtmLandmark(_activePoint);
                if (ilm != nullptr) {

                    QPointF pos = interactionWidget->widgetToImageCoordinates(e->pos());
                    pos /= _imageRescale;

                    ilm->setImageCoordinates(pos);
                    requestFullRepainting();
                    return true;
                }
            }
        }
    }

    return false;

}

qint64 InputDtmLandmarksOverlay::pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const {

    if (interactionWidget == nullptr) {
        return -1;
    }

    if (_currentDataBlock == nullptr) {
        return -1;
    }

    QPointF imCoord = interactionWidget->widgetToImageCoordinates(widgetCoordinate);
    imCoord /= _imageRescale;

    return _currentDataBlock->getDtmLandmarkAt(imCoord, (1200./interactionWidget->zoom())/_imageRescale);
}

double InputDtmLandmarksOverlay::imageRescale() const
{
    return _imageRescale;
}

void InputDtmLandmarksOverlay::setImageRescale(double newImageRescale)
{
    if (qFuzzyCompare(_imageRescale, newImageRescale))
        return;
    _imageRescale = newImageRescale;
    emit imageRescaleChanged();
}

} // namespace PikaLTools
