#ifndef _pose_kalman_LocalPlanner_hpp
#define _pose_kalman_LocalPlanner_hpp

#include "MoveBase.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {

class LocalPlanner {
 public:
    struct Params {
        T acc_lim_xy = 0.3;
        T acc_lim_yaw = 1.6;
        T vel_lim_xy = 2;
        T vel_lim_yaw = 6;
    };
    Params params;
    void setParams(const Params& params) { this->params = params; }
    bool getControlCmd(const T pose[3], const T vel[3], const MoveBase::Goal& goal, T cmd_vel[3]) const;
};

}  // namespace pose_kalman

#endif  // _pose_kalman_LocalPlanner_hpp