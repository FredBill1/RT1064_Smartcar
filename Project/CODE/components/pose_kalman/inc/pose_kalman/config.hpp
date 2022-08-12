#ifndef _pose_kalman_config_hpp
#define _pose_kalman_config_hpp

#include <cstdint>

namespace pose_kalman {

#define pose_kalman_use_squareroot false

using T = double;
constexpr bool useDynamicProcessNoiseCovariance = true;
constexpr int64_t timeout_us = 1000000;
enum class MeasurementType : int {
    Odom,
    Gyro,
    Yaw,
    Rect,
    SetState,
    NUM_TYPES,
};

constexpr int MeasurementQueueSize(MeasurementType type) {
    switch (type) {
    case MeasurementType::Odom: return 20;
    case MeasurementType::Gyro: return 20;
    case MeasurementType::Yaw: return 20;
    case MeasurementType::Rect: return 1;
    case MeasurementType::SetState: return 1;
    default: return 0;
    }
}

#define MeasurementModelTypes Odom, Gyro, Yaw, Rect, SetState

}  // namespace pose_kalman

#endif  // _pose_kalman_config_hpp