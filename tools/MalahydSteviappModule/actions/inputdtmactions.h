#ifndef INPUTDTMACTIONS_H
#define INPUTDTMACTIONS_H

#include <QString>

namespace StereoVisionApp{
    class Project;
}

namespace PikaLTools {

class InputDtm;

int addInputDtm(StereoVisionApp::Project* p, const QString &pFile = "");

bool viewInputDtm(InputDtm* inputDtm);

bool viewInputDtm2D(InputDtm* inputDtm);

}
#endif // INPUTDTMACTIONS_H
