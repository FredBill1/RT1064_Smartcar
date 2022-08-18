#ifndef _localPlannerParam_hpp
#define _localPlannerParam_hpp

#include "MoveBase.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/utils.hpp"
using pose_kalman::PI, pose_kalman::PI_2;

constexpr pose_kalman::T PARAM_DONT_CARE = 1e6;
constexpr int PARAM_DISABLE = 0;
constexpr pose_kalman::T DEG = PI / 180, RPS = PI * 2;

namespace Param {
constexpr pose_kalman::LocalPlanner::Params localPlannerParam{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};
}

//! 出库
// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_OUT_GARAGE_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 1 * RPS,
    .acc_lim_xy = 1.4,
    .acc_lim_yaw = 1 * RPS,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_OUT_GARAGE{
    .xy_tolerance = PARAM_DISABLE,
    .yaw_tolerance = PARAM_DISABLE,
    .xy_near = 5e-2,
    .yaw_near = PARAM_DONT_CARE,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_OUT_GARAGE_PARAM,
};

//! 去目标点
// 自转
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_TURN_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 2 * RPS,
    .acc_lim_xy = 1.4,
    .acc_lim_yaw = 2 * RPS,
    .dt_ref = 0.255,
};

constexpr MoveBase::Goal GOAL_NAVI_TURN{
    .xy_tolerance = PARAM_DISABLE,
    .yaw_tolerance = PARAM_DISABLE,
    .xy_near = PARAM_DONT_CARE,
    .yaw_near = 3 * DEG,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_NAVI_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_MOVE_PARAM{
    .vel_lim_xy = 2.5,
    .vel_lim_yaw = 1 * RPS,
    .acc_lim_xy = 1.4,
    .acc_lim_yaw = 2 * RPS,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI_MOVE{
    .xy_tolerance = PARAM_DISABLE,
    .yaw_tolerance = PARAM_DISABLE,
    .xy_near = 2.5,
    .yaw_near = PARAM_DONT_CARE,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_NAVI_MOVE_PARAM,
};

// 调整
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_REFINE_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 1 * RPS,
    .acc_lim_xy = 1.4,
    .acc_lim_yaw = 2 * RPS,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_NAVI_REFINE{
    .xy_tolerance = 15e-3,
    .yaw_tolerance = 6 * DEG,
    .xy_near = PARAM_DISABLE,
    .yaw_near = PARAM_DISABLE,
    .time_tolerance_us = uint64_t(31e4),
    .reached = false,
    .params = &GOAL_NAVI_REFINE_PARAM,
};

//! 搬运卡片
// 旋转
constexpr pose_kalman::LocalPlanner::Params GOAL_CARRY_TURN_PARAM{
    .vel_lim_xy = 2.5,
    .vel_lim_yaw = 1.5 * RPS,
    .acc_lim_xy = 2,
    .acc_lim_yaw = 1.5 * RPS,
    .dt_ref = 0.255,
};

constexpr MoveBase::Goal GOAL_CARRY_TURN{
    .xy_tolerance = PARAM_DISABLE,
    .yaw_tolerance = PARAM_DISABLE,
    .xy_near = PARAM_DONT_CARE,
    .yaw_near = 6 * DEG,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_CARRY_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_CARRY_MOVE_PARAM{
    .vel_lim_xy = 2.5,
    .vel_lim_yaw = 1 * RPS,
    .acc_lim_xy = 2,
    .acc_lim_yaw = 1 * RPS,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_CARRY_MOVE{
    .xy_tolerance = 5e-2,
    .yaw_tolerance = 6 * DEG,
    .xy_near = 5e-2,
    .yaw_near = PARAM_DONT_CARE,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_CARRY_MOVE_PARAM,
};

//! 车库
// 旋转
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_TURN_PARAM{
    .vel_lim_xy = 2.5,
    .vel_lim_yaw = 2 * RPS,
    .acc_lim_xy = 10,
    .acc_lim_yaw = 2 * RPS,
    .dt_ref = 0.255,
};

constexpr MoveBase::Goal GOAL_GARAGE_TURN{
    .xy_tolerance = PARAM_DISABLE,
    .yaw_tolerance = PARAM_DISABLE,
    .xy_near = PARAM_DONT_CARE,
    .yaw_near = 6 * DEG,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_GARAGE_TURN_PARAM,
};

// 平移
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_MOVE_PARAM{
    .vel_lim_xy = 1.0,
    .vel_lim_yaw = 1 * RPS,
    .acc_lim_xy = 10,
    .acc_lim_yaw = 1 * RPS,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_GARAGE_MOVE{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * DEG,
    .xy_near = 5e-2,
    .yaw_near = PARAM_DONT_CARE,
    .time_tolerance_us = PARAM_DISABLE,
    .reached = false,
    .params = &GOAL_GARAGE_MOVE_PARAM,
};

// 入库位置
constexpr float garage_position[2]{0.70, 0.55};
constexpr float garage_left_move = 5e-2;
constexpr int garage_down_delay = 300;
constexpr int garage_stop_delay = 50;

#endif  // _localPlannerParam_hpp