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

//! 找卡片
constexpr pose_kalman::LocalPlanner::Params GOAL_NAVI_PARAM{
    .vel_lim_xy = 1.6,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
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

//! 自转吸卡片
constexpr pose_kalman::LocalPlanner::Params GOAL_PICK_PARAM{
    .vel_lim_xy = 1.5,
    .vel_lim_yaw = 14,
    .acc_lim_xy = 0.8,
    .acc_lim_yaw = 14,
    .dt_ref = 0.255,
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

//! 搬运卡片
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

//! 最后一轮搬运卡片
constexpr pose_kalman::LocalPlanner::Params GOAL_FINAL_CARRY_PARAM{
    .vel_lim_xy = 1.4,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_FINAL_CARRY{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e-1,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_FINAL_CARRY_PARAM,
};

//! 回车库前
constexpr pose_kalman::LocalPlanner::Params GOAL_BEFORE_GARAGE_PARAM{
    .vel_lim_xy = 1.4,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 1.1,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_BEFORE_GARAGE{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 5e-2,
    .yaw_near = 6 * PI / 180,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_BEFORE_GARAGE_PARAM,
};

//! 按边线找车库
// 直走时的参数
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_XY_PARAM{
    .vel_lim_xy = 1.0,
    .vel_lim_yaw = 7,
    .acc_lim_xy = 10,
    .acc_lim_yaw = 7,
    .dt_ref = 0.33,
};

constexpr MoveBase::Goal GOAL_GARAGE_XY{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 5e-2,
    .yaw_near = 1e6,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_GARAGE_XY_PARAM,
};

// 旋转时的参数
constexpr pose_kalman::LocalPlanner::Params GOAL_GARAGE_YAW_PARAM{
    .vel_lim_xy = 0.0,
    .vel_lim_yaw = 14,
    .acc_lim_xy = 10,
    .acc_lim_yaw = 14,
    .dt_ref = 0.255,
};

constexpr MoveBase::Goal GOAL_GARAGE_YAW{
    .xy_tolerance = 1e-2,
    .yaw_tolerance = 6 * PI / 180,
    .xy_near = 1e6,
    .yaw_near = 6 * PI / 180,
    .time_tolerance_us = 0,
    .reached = false,
    .params = &GOAL_GARAGE_YAW_PARAM,
};

// 入库位置
constexpr float garage_position[2]{0.55, 0.55};
constexpr float garage_left_padding = -5e-2;
constexpr int garage_down_delay = 300;
constexpr int garage_stop_delay = 50;

#endif  // _localPlannerParam_hpp