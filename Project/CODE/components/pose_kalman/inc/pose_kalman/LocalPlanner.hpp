#ifndef _pose_kalman_LocalPlanner_hpp
#define _pose_kalman_LocalPlanner_hpp

#include "pose_kalman/config.hpp"

namespace pose_kalman {

class LocalPlanner {
 public:
    struct Params {
        T acc_lim_xy = 1;
        T acc_lim_yaw = 1;
        T vel_lim_xy = 2;
        T vel_lim_yaw = 3;

        T xy_goal_tolerance = 5 * 1e-3;
        T yaw_goal_tolerance = 0.1;
    };
    Params params;
    bool getControlCmd(const T pose[3], const T vel[3], const T target[3], T cmd_vel[3]) const;
};

}  // namespace pose_kalman

#endif  // _pose_kalman_LocalPlanner_hpp