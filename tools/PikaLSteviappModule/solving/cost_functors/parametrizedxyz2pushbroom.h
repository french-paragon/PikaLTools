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
            residual[0] = T(std::nan(""));
            residual[1] = T(std::nan(""));
            #ifndef NDEBUG
            std::cout << "Error in ParametrizedXYZ2PushBroom projection computation" << std::endl;
            #endif
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

        T du = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dv = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;

        proj *= f[0];

        proj[1] += pp[0];

        V2T error = proj;
        error[1] -= _uv[0] + du;
        error[0] -= _uv[1] + dv;

        residual[0] = error[0];
        residual[1] = error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in ParametrizedXYZ2PushBroom projection computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector2d _uv;
    double _sensorWidth;
    double _timeUnitsPerPixels;
};

/*!
 * \brief The ParametrizedInterpolatedXYZ2PushBroom class represent a measurement error on a GCP or tie point for a push broom sensor, where the pose is interpolated between two pose nodes.
 */
class ParametrizedInterpolatedXYZ2PushBroom
{
public:
    ParametrizedInterpolatedXYZ2PushBroom(Eigen::Vector2d const& uv,
                                          double const& sensorWidth,
                                          double const& w1,
                                          double const& w2);

    template <typename T>
    bool operator()(const T* const lm,
                    const T* const leverArm_r,
                    const T* const leverArm_t,
                    const T* const r1, // orientation1
                    const T* const t1, // position1
                    const T* const r2, // orientation2
                    const T* const t2, // position2
                    const T* const f, // focal length
                    const T* const pp, // principal point (scalar)
                    const T* const as, // distortion coefficients for the u coordinate
                    const T* const bs, // distortion coefficients for the v coordinate
                    T* residual) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << lm[0], lm[1], lm[2];

        V3T pose_r1;
        pose_r1 << r1[0], r1[1], r1[2];

        V3T pose_r2;
        pose_r2 << r2[0], r2[1], r2[2];

        V3T leverarm_r;
        leverarm_r << leverArm_r[0], leverArm_r[1], leverArm_r[2];

        M3T pose_R1 = StereoVision::Geometry::rodriguezFormula<T>(pose_r1);
        M3T pose_R2 = StereoVision::Geometry::rodriguezFormula<T>(pose_r2);

        M3T pose_RDelta = pose_R1.transpose()*pose_R2;

        V3T pose_RDeltaLog = StereoVision::Geometry::inverseRodriguezFormula<T>(pose_RDelta);
        pose_RDeltaLog *= T(_w2);

        M3T pose_RDeltaInterp = StereoVision::Geometry::rodriguezFormula<T>(pose_RDeltaLog);

        M3T pose_R = pose_R1*pose_RDeltaInterp;

        V3T pose_t1;
        pose_t1 << t1[0], t1[1], t1[2];

        V3T pose_t2;
        pose_t2 << t2[0], t2[1], t2[2];

        V3T pose_t = T(_w1)*pose_t1 + T(_w2)*pose_t2;

        V3T leverarm_t;
        leverarm_t << leverArm_t[0], leverArm_t[1], leverArm_t[2];

        V3T Pbar = StereoVision::Geometry::angleAxisRotate<T>(leverarm_r,pose_R.transpose()*(lm_pos - pose_t)) + leverarm_t;
        if (Pbar[2] < 0.0) {
            residual[0] = T(std::nan(""));
            residual[1] = T(std::nan(""));
            #ifndef NDEBUG
            std::cout << "Error in ParametrizedInterpolatedXYZ2PushBroom projection computation" << std::endl;
            #endif
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

        T du = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dv = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;

        proj *= f[0];

        proj[1] += pp[0];

        V2T error = proj;
        error[0] -= _uv[1] + dv;
        error[1] -= _uv[0] + du;

        residual[0] = error[0];
        residual[1] = error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in ParametrizedInterpolatedXYZ2PushBroom projection computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    double _w1, _w2;

    Eigen::Vector2d _uv;
    double _sensorWidth;
};

class ParametrizedInterpolatedPriorXYZ2PushBroom
{
public:
    ParametrizedInterpolatedPriorXYZ2PushBroom(Eigen::Vector2d const& uv,
                                               Eigen::Vector3d const& xyz,
                                               double const& sensorWidth,
                                               double const& w1,
                                               double const& w2);

    template <typename T>
    bool operator()(const T* const leverArm_r,
                    const T* const leverArm_t,
                    const T* const r1, // orientation1
                    const T* const t1, // position1
                    const T* const r2, // orientation2
                    const T* const t2, // position2
                    const T* const f, // focal length
                    const T* const pp, // principal point (scalar)
                    const T* const as, // distortion coefficients for the u coordinate
                    const T* const bs, // distortion coefficients for the v coordinate
                    T* residual) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T pose_r1;
        pose_r1 << r1[0], r1[1], r1[2];

        V3T pose_r2;
        pose_r2 << r2[0], r2[1], r2[2];

        V3T leverarm_r;
        leverarm_r << leverArm_r[0], leverArm_r[1], leverArm_r[2];

        M3T pose_R1 = StereoVision::Geometry::rodriguezFormula<T>(pose_r1);
        M3T pose_R2 = StereoVision::Geometry::rodriguezFormula<T>(pose_r2);

        M3T pose_RDelta = pose_R1.transpose()*pose_R2;

        V3T pose_RDeltaLog = StereoVision::Geometry::inverseRodriguezFormula<T>(pose_RDelta);
        pose_RDeltaLog *= T(_w2);

        M3T pose_RDeltaInterp = StereoVision::Geometry::rodriguezFormula<T>(pose_RDeltaLog);

        M3T pose_R = pose_R1*pose_RDeltaInterp;

        V3T pose_t1;
        pose_t1 << t1[0], t1[1], t1[2];

        V3T pose_t2;
        pose_t2 << t2[0], t2[1], t2[2];

        V3T pose_t = T(_w1)*pose_t1 + T(_w2)*pose_t2;

        V3T leverarm_t;
        leverarm_t << leverArm_t[0], leverArm_t[1], leverArm_t[2];

        V3T Pbar = StereoVision::Geometry::angleAxisRotate<T>(leverarm_r,pose_R.transpose()*(_xyz.cast<T>() - pose_t)) + leverarm_t;
        if (Pbar[2] < 0.0) {
            residual[0] = T(std::nan(""));
            residual[1] = T(std::nan(""));
            #ifndef NDEBUG
            std::cout << "Error in ParametrizedInterpolatedXYZ2PushBroom projection computation" << std::endl;
            #endif
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

        T du = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dv = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;

        proj *= f[0];

        proj[1] += pp[0];

        V2T error = proj;
        error[0] -= _uv[1] + dv;
        error[1] -= _uv[0] + du;

        residual[0] = error[0];
        residual[1] = error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in ParametrizedInterpolatedXYZ2PushBroom projection computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    double _w1, _w2;

    Eigen::Vector2d _uv;
    Eigen::Vector3d _xyz;
    double _sensorWidth;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_PARAMETRIZEDXYZ2PUSHBROOM_H
