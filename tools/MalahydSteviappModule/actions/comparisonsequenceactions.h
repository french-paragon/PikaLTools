#ifndef COMPARISONSEQUENCEACTIONS_H
#define COMPARISONSEQUENCEACTIONS_H

#include <QString>

namespace StereoVisionApp{
    class Project;
}

namespace PikaLTools {

class ComparisonTrajectory;

int addComparisonSequence(StereoVisionApp::Project* p, const QString &pFile = "");

bool viewComparisonTrajectory(ComparisonTrajectory* traj);

}

#endif // COMPARISONSEQUENCEACTIONS_H
