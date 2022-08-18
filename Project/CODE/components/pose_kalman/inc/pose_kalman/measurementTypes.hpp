#ifndef _pose_kalman_measurementTypes_hpp
#define _pose_kalman_measurementTypes_hpp

#include "pose_kalman/Measurement.hpp"

namespace pose_kalman {

using Odom = Measurement<0, 0, 0, 1, 1, 1>;
using Gyro = Measurement<0, 0, 0, 0, 0, 1>;
using Yaw = Measurement<0, 0, 1, 0, 0, 0>;
using Rect = Measurement<1, 1, 1, 0, 0, 0>;
using SetState = Measurement<1, 1, 1, 1, 1, 1>;

// struct RectNoiseJacobianUpdater;
// using Rect = Measurement<1, 1, 0, 0, 0, 0, RectNoiseJacobianUpdater>;
// struct RectNoiseJacobianUpdater {
//     void operator()(const State& x, Rect::NoiseJacobian& V) const { V.diagonal().setConstant(x.velocityNorm()); }
// };

}  // namespace pose_kalman

#endif  // _pose_kalman_measurementTypes_hpp