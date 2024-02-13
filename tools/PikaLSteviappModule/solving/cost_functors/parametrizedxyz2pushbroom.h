#ifndef PIKALTOOLS_PARAMETRIZEDXYZ2PUSHBROOM_H
#define PIKALTOOLS_PARAMETRIZEDXYZ2PUSHBROOM_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <ceres/jet.h>

namespace PikaLTools {

class ParametrizedXYZ2PushBroom
{
public:
    ParametrizedXYZ2PushBroom(Eigen::Vector2d const& uv, double const& sensorWidth, double const& timeUnitsPerPixels);

    template <typename T>
    bool operator()(const T* const lm,
                    const T* const leverArm_r,
                    const T* const leverArm_t,
                    const T* const r, // orientation
                    const T* const w, // angular speed (in axis angle unit per scanline)
                    const T* const t, // position
                    const T* const v, // speed (in world length unit per scanline)
                    const T* const f, // focal length
                    const T* const pp, // principal point (scalar)
                    const T* const as, // distortion coefficients for the x coordinate
                    const T* const bs, // distortion coefficients for the y coordinate
                    T* residual) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << lm[0], lm[1], lm[2];

        double dt = _timeUnitsPerPixels*_uv[1];

        V3T pose_r;
        pose_r << r[0] + w[0]*dt, r[1] + w[1]*dt, r[2] + w[2]*dt; //TODO: check if we want to integrate on the maniforld instead.

        V3T leverarm_r;
        leverarm_r << leverArm_r[0], leverArm_r[1], leverArm_r[2];

        M3T pose_R = StereoVision::Geometry::rodriguezFormula(pose_r);

        V3T pose_t;
        pose_t << t[0] + v[0]*dt, t[1] + v[1]*dt, t[2] + v[2]*dt; //account for the fact that y coordinate in the push broom sensor is time!

        V3T leverarm_t;
        leverarm_t << leverArm_t[0], leverArm_t[1], leverArm_t[2];

        V3T Pbar = StereoVision::Geometry::angleAxisRotate<T>(leverarm_r,pose_R.transpose()*(lm_pos - pose_t)) + leverarm_t;
        if (Pbar[2] < 0.0) {
            return false;
        }

        //projection of the point in homogeneous coordinates in the body frame.
        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        //compute distortion.

        double s = _uv.x()/_sensorWidth;
        double s2 = s*s;
        double s3 = s2*s;
        double s4 = s3*s;
        double s5 = s4*s;

        T dx = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dy = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;

        proj *= f[0];

        proj[0] += pp[0];

        V2T error = proj;
        error[0] -= _uv[0] + dx;
        error[1] -= _uv[1] + dy;

        residual[0] = error[0];
        residual[1] = error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in projection computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector2d _uv;
    double _sensorWidth;
    double _timeUnitsPerPixels;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_PARAMETRIZEDXYZ2PUSHBROOM_H
