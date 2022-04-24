#ifndef _pose_kalman_config_hpp
#define _pose_kalman_config_hpp

namespace pose_kalman {
using T = double;
constexpr bool useDynamicProcessNoiseCovariance = true;

enum class MeasurementType : int {
    ODOM,
    GYRO,
    NUM_TYPES,
};

enum class MeasurementSize {
    ODOM = 3,
    GYRO = 1,
};

}  // namespace pose_kalman

#endif  // _pose_kalman_config_hpp