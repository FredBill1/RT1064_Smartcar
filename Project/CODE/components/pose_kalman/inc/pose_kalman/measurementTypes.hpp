#ifndef _pose_kalman_measurementTypes_hpp
#define _pose_kalman_measurementTypes_hpp

#include "pose_kalman/Measurement.hpp"

namespace pose_kalman {

using Odom = Measurement<0, 0, 0, 1, 1, 1>;
using Gyro = Measurement<0, 0, 0, 0, 0, 1>;
using SetState = Measurement<1, 1, 1, 1, 1, 1>;

}  // namespace pose_kalman

#endif  // _pose_kalman_measurementTypes_hpp