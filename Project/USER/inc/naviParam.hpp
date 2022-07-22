#ifndef _localPlannerParam_hpp
#define _localPlannerParam_hpp

#include "MoveBase.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/utils.hpp"
using pose_kalman::PI, pose_kalman::PI_2;

namespace Param {
constexpr pose_kalman::LocalPlanner::Params localPlannerParam{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};
}

//! ÕÒ¿¨Æ¬
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_PARAM{
    .vel_lim_xy = 1.8,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 8,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 5 * PI / 180,
    .xy_near = 0,
    .yaw_near = 0,
    .time_tolerance_us = uint64_t(30e4),
    .reached = false,
    .params = &GOAL_NAVI_PARAM,
};

//! ×Ô×ªÎü¿¨Æ¬
constexpr pose_kalman::LocalPlanner::Params GOAL_PICK_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 14,
    .acc_lim_xy = 0.8,
    .acc_lim_yaw = 12,
    .dt_ref = 0.26,
};

constexpr MoveBase::Goal GOAL_PICK{
    .xy_tolerance = 10e-3,
    .yaw_tolerance = 4 * PI / 180,
    .xy_near = 0,
    .yaw_near = 0,
    .time_tolerance_us = uint64_t(15e4),
    .reached = false,
    .params = &GOAL_PICK_PARAM,
};

//! °áÔË¿¨Æ¬
constexpr pose_kalman::LocalPlanner::Params GOAL_CARRY_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_CARRY{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_CARRY_PARAM,
};

//! »Ø³µ¿â
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_PARAM{
    .vel_lim_xy = 1.8,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_GARAGE{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 2e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_GARAGE_PARAM,
};

#endif  // _localPlannerParam_hpp