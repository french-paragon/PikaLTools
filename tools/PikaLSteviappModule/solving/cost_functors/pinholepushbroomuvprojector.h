#ifndef PIKALTOOLS_PINHOLEPUSHBROOMUVPROJECTOR_H
#define PIKALTOOLS_PINHOLEPUSHBROOMUVPROJECTOR_H

#include <Eigen/Core>

namespace PikaLTools {

class PinholePushbroomUVProjector
{
public:
    PinholePushbroomUVProjector(double sensorWidth);

    template<typename T>
    Eigen::Matrix<T,3,1> dirFromUV(T* uv,
                                   T const* const* params) {

        const T* f = params[0];
        const T* pp = params[1];
        const T* as = params[2];
        const T* bs = params[3];

        T s = uv[0]/T(_sensorWidth);
        T s2 = s*s;
        T s3 = s2*s;
        T s4 = s3*s;
        T s5 = s4*s;

        T du = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dv = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;


        Eigen::Matrix<T,3,1> ret;
        ret << uv[1] + dv, uv[0] - pp[0] + du, f[0];

        return ret;
    }

protected:

    double _sensorWidth;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_PINHOLEPUSHBROOMUVPROJECTOR_H
