#include "parametrizedxyz2pushbroom.h"

namespace PikaLTools {

ParametrizedXYZ2PushBroom::ParametrizedXYZ2PushBroom(Eigen::Vector2d const& uv, double const& sensorWidth, const double &timeUnitsPerPixels) :
    _uv(uv),
    _sensorWidth(sensorWidth),
    _timeUnitsPerPixels(timeUnitsPerPixels)
{

}

ParametrizedInterpolatedXYZ2PushBroom::ParametrizedInterpolatedXYZ2PushBroom(Eigen::Vector2d const& uv,
                                                                             double const& sensorWidth,
                                                                             double const& w1,
                                                                             double const& w2) :
    _w1(w1),
    _w2(w2),
    _uv(uv),
    _sensorWidth(sensorWidth)
{

}

} // namespace PikaLTools
