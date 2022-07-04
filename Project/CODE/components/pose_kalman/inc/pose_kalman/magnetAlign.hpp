#ifndef _pose_kalman_magnetAlign_hpp
#define _pose_kalman_magnetAlign_hpp

#include "apriltag/config.hpp"
#include "pose_kalman/config.hpp"
namespace pose_kalman {

void magnetAlign(const T pose_xy[2], const imgProc::apriltag::float_t target_xy[2], int magnet_idx, T result_pose[3]);

}  // namespace pose_kalman

#endif  // _pose_kalman_magnetAlign_hpp