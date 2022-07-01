#ifndef _localPlannerParam_hpp
#define _localPlannerParam_hpp

#include "pose_kalman/LocalPlanner.hpp"
namespace Param {
constexpr pose_kalman::LocalPlanner::Params localPlannerParam{
    .vel_lim_xy = 2,
    .vel_lim_yaw = 6,
    .acc_lim_xy = 0.8,
    .acc_lim_yaw = 2,
    .dt_ref = 0.3,
};
}
#endif  // _localPlannerParam_hpp