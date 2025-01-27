#ifndef PIKALTOOLS_ORIENTATIONSPEEDCONSISTENCY_H
#define PIKALTOOLS_ORIENTATIONSPEEDCONSISTENCY_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>


namespace PikaLTools {

class OrientationSpeedConsistency
{
public:
    OrientationSpeedConsistency(double dt);

    template <typename T>
    bool operator()(const T* const r1,
                    const T* const w1,
                    const T* const r2,
                    T* residual) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;


        V3T pose_r1;
        pose_r1 << r1[0], r1[1], r1[2];

        V3T pose_w1;
        pose_w1 << w1[0], w1[1], w1[2];

        V3T pose_r2;
        pose_r2 << r2[0], r2[1], r2[2];

        M3T pose_R1 = StereoVision::Geometry::rodriguezFormula<T>(pose_r1);
        M3T pose_W1 = StereoVision::Geometry::rodriguezFormula<T>(pose_w1*_dt);

        V3T composed = StereoVision::Geometry::inverseRodriguezFormula<T>(pose_R1*pose_W1);

        V3T err = composed - pose_r2;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

        return true;

    }

protected:

    double _dt;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_ORIENTATIONSPEEDCONSISTENCY_H
