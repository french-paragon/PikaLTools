#ifndef NEXTANDPREVIOUSFRAMEEVENTFILTER_H
#define NEXTANDPREVIOUSFRAMEEVENTFILTER_H

#include <QObject>

class NextAndPreviousFrameEventFilter : public QObject
{
    Q_OBJECT
public:
    explicit NextAndPreviousFrameEventFilter(QObject *parent = nullptr);

    bool eventFilter(QObject *object, QEvent *event) override;

Q_SIGNALS:
    void nextFrameRequested();
    void previousFrameRequested();

};

#endif // NEXTANDPREVIOUSFRAMEEVENTFILTER_H
