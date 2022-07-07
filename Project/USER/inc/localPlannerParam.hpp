#ifndef _localPlannerParam_hpp
#define _localPlannerParam_hpp

#include "MoveBase.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/utils.hpp"
using pose_kalman::PI, pose_kalman::PI_2;

namespace Param {
constexpr pose_kalman::LocalPlanner::Params localPlannerParam{
    .vel_lim_xy = 2,
    .vel_lim_yaw = 6,
    .acc_lim_xy = 0.8,
    .acc_lim_yaw = 10,
    .dt_ref = 0.35,
};
}

constexpr MoveBase::Goal GOAL_NAVI{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 5 * PI / 180,
    .time_tolerance_us = uint64_t(15e4),
    .reached = false,
};

constexpr MoveBase::Goal GOAL_PICK{
    .xy_tolerance = 5e-3,
    .yaw_tolerance = 3 * PI / 180,
    .time_tolerance_us = uint64_t(10e4),
    .reached = false,
};

constexpr MoveBase::Goal GOAL_CARRY{
    .xy_tolerance = 1e-1,
    .yaw_tolerance = 10 * PI / 180,
    .time_tolerance_us = 0,
    .reached = false,
};

#endif  // _localPlannerParam_hpp