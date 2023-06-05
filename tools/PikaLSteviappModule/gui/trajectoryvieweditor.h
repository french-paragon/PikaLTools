#ifndef PIKALTOOLS_TRAJECTORYVIEWEDITOR_H
#define PIKALTOOLS_TRAJECTORYVIEWEDITOR_H

#include <steviapp/gui/editor.h>

#include "LibStevi/geometry/rotations.h"

#include <QMap>

class QSpinBox;

namespace StereoVisionApp {

class OpenGl3DSceneViewWidget;
class OpenGlDrawableSceneGrid;

}

namespace PikaLTools {

class OpenGlDrawableDtm;
class OpenGlDrawableTrajectory;

class BilSequenceAcquisitionData;
class ComparisonTrajectory;
class InputDtm;

class TrajectoryViewEditor : public StereoVisionApp::Editor
{
    Q_OBJECT
public:
    TrajectoryViewEditor(QWidget* parent = nullptr);

    void setTrajectory(BilSequenceAcquisitionData const& bilSequence);
    void clearTrajectory();

    void setComparisonTrajectory(ComparisonTrajectory const& comparisonTrajectory);
    void clearComparisonTrajectory();

    void setDtm(InputDtm *bilSequence);
    void clearDtm();

    void setStartLine(int bilLine);
    void setEndLine(int bilLine);

protected:

    void setDrawableScale(float scale);

    inline float bil2lcfLineId(int bilId) const {
        QList<int> splitLines = _bil2lcfLines.keys();
        int minAbove = bilId;
        int maxBelow = bilId;

        for (int l : splitLines) {

            if (l > bilId) {
                if (minAbove == bilId) {
                    minAbove = l;
                }
            } else if (l < minAbove and l > bilId) {
                minAbove = l;
            }

            if (l < bilId) {
                if (maxBelow == bilId) {
                    maxBelow = l;
                }
            } else if (l > maxBelow and l < bilId) {
                maxBelow = l;
            }
        }

        if (bilId == maxBelow) {
            return _bil2lcfLines.value(maxBelow, 0);
        }

        if (bilId == minAbove) {
            return _bil2lcfLines.value(maxBelow, 0);
        }

        float lcfLBelow = _bil2lcfLines.value(maxBelow, 0);
        float lcfLAbove = _bil2lcfLines.value(minAbove, 0);

        float weight = float(bilId - maxBelow)/float(minAbove - maxBelow);
        return (1-weight)*lcfLBelow + weight*lcfLAbove;
    }

    StereoVisionApp::OpenGl3DSceneViewWidget* _viewScene;

    StereoVisionApp::OpenGlDrawableSceneGrid* _grid;
    OpenGlDrawableDtm* _drawableDtm;
    OpenGlDrawableTrajectory* _drawableTrajectory;
    OpenGlDrawableTrajectory* _comparisonTrajectory;

    QMap<int, int> _bil2lcfLines;

    QSpinBox* _startLineSpinBox;
    QSpinBox* _endLineSpinBox;

    std::vector<double> _baseTrajectoryTimes;
    std::vector<double> _compareTrajectoryTimes;
};

class TrajectoryViewEditorFactory : public StereoVisionApp::EditorFactory
{
    Q_OBJECT
public:

    explicit TrajectoryViewEditorFactory(QObject *parent = nullptr);

    QString TypeDescrName() const override;
    QString itemClassName() const override;
    StereoVisionApp::Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace PikaLTools

#endif // PIKALTOOLS_TRAJECTORYVIEWEDITOR_H
