#include "nextandpreviousframeeventfilter.h"

#include <QKeyEvent>

NextAndPreviousFrameEventFilter::NextAndPreviousFrameEventFilter(QObject *parent)
    : QObject{parent}
{

}

bool NextAndPreviousFrameEventFilter::eventFilter(QObject *object, QEvent *event) {

    if (event->type() != QEvent::KeyRelease) {
        return false;
    }

    QKeyEvent* keyEvent = dynamic_cast<QKeyEvent*>(event);

    if (keyEvent->key() == Qt::Key_N) {
        Q_EMIT nextFrameRequested();
        return true;
    } else if (keyEvent->key() == Qt::Key_P) {
        Q_EMIT previousFrameRequested();
        return true;
    }

    return false;

}
