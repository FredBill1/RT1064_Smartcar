#ifndef _localPlannerParam_hpp
#define _localPlannerParam_hpp

#include "MoveBase.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/utils.hpp"
using pose_kalman::PI, pose_kalman::PI_2;

namespace Param {
constexpr pose_kalman::LocalPlanner::Params localPlannerParam{
    .vel_lim_xy = 2.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.0,
    .acc_lim_yaw = 10,
    .dt_ref = 0.33,
};
}

constexpr MoveBase::Goal GOAL_NAVI{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 5 * PI / 180,
    .time_tolerance_us = uint64_t(30e4),
    .reached = false,
};

constexpr MoveBase::Goal GOAL_PICK{
    .xy_tolerance = 10e-3,
    .yaw_tolerance = 4 * PI / 180,
    .time_tolerance_us = uint64_t(15e4),
    .reached = false,
};

constexpr MoveBase::Goal GOAL_CARRY{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .time_tolerance_us = 0,
    .reached = false,
};

#endif  // _localPlannerParam_hpp