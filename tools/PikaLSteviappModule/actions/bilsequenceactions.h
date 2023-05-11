#ifndef BILSEQUENCEACTIONS_H
#define BILSEQUENCEACTIONS_H

#include <QString>

namespace StereoVisionApp{
    class Project;
}

namespace PikaLTools {

int loadBilSequenceFromFolder(StereoVisionApp::Project* p, QString const& folder = "");

} // namespace PikaLTools

#endif // BILSEQUENCEACTIONS_H
