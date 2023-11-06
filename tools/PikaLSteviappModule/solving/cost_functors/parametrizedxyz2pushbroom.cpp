#include "parametrizedxyz2pushbroom.h"

namespace PikaLTools {

ParametrizedXYZ2PushBroom::ParametrizedXYZ2PushBroom(Eigen::Vector2d const& uv, double const& sensorWidth, const double &timeUnitsPerPixels) :
    _uv(uv),
    _sensorWidth(sensorWidth),
    _timeUnitsPerPixels(timeUnitsPerPixels)
{

}

} // namespace PikaLTools
