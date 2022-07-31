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

//! 去目标点
// 自转
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_TURN_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI_TURN{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 5 * PI / 180,
    .xy_near = 0,
    .yaw_near = 0,
    .time_tolerance_us = uint64_t(30e4),
    .reached = false,
    .params = &GOAL_NAVI_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_MOVE_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI_MOVE{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 5 * PI / 180,
    .xy_near = 0,
    .yaw_near = 0,
    .time_tolerance_us = uint64_t(30e4),
    .reached = false,
    .params = &GOAL_NAVI_MOVE_PARAM,
};

// 调整
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_REFINE_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI_REFINE{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 5 * PI / 180,
    .xy_near = 0,
    .yaw_near = 0,
    .time_tolerance_us = uint64_t(30e4),
    .reached = false,
    .params = &GOAL_NAVI_REFINE_PARAM,
};

//! 搬运卡片
// 旋转
constexpr pose_kalman::LocalPlanner::Params GOAL_CARRY_TURN_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_CARRY_TURN{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_CARRY_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_CARRY_MOVE_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_CARRY_MOVE{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_CARRY_MOVE_PARAM,
};

//! 车库
// 旋转
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_TURN_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_GARAGE_TURN{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_GARAGE_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_MOVE_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_GARAGE_MOVE{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_GARAGE_MOVE_PARAM,
};

// 入库位置
constexpr float garage_position[2]{0.55, 0.55};
constexpr float garage_left_padding = -5e-2;
constexpr int garage_down_delay = 300;
constexpr int garage_stop_delay = 50;

#endif  // _localPlannerParam_hpp