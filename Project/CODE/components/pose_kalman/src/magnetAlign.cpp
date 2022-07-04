#include "pose_kalman/magnetAlign.hpp"

#include <array>
#include <cmath>

#include "magnetConfig.hpp"
#include "pose_kalman/utils.hpp"

namespace pose_kalman {

void magnetAlign(const T pose_xy[2], const imgProc::apriltag::float_t target_xy[2], int magnet_idx, T result_pose[3]) {
    auto [mag_x, mag_y] = magnet::pos[magnet_idx];
    float mag_r = std::sqrt(mag_x * mag_x + mag_y * mag_y);
    float dx = target_xy[0] - pose_xy[0], dy = target_xy[1] - pose_xy[1], r = std::sqrt(dx * dx + dy * dy);
    result_pose[0] = pose_xy[0] + dx * (1 - (mag_r / r));
    result_pose[1] = pose_xy[1] + dy * (1 - (mag_r / r));
    result_pose[2] = wrapAngle(std::atan2(dy, dx) - std::atan2(mag_y, mag_x));
}

}  // namespace pose_kalman