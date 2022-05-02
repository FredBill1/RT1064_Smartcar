#include "pose_kalman/LocalPlanner.hpp"

#include <rtthread.h>

#include "Eigen/Eigen"
#include "pose_kalman/params.hpp"
#include "pose_kalman/utils.hpp"

namespace pose_kalman {
using namespace Eigen;
using Vector2 = Matrix<T, 2, 1>;
using Matrix2 = Matrix<T, 2, 2>;
static inline void chkmin(T &a, T b) {
    if (a > b) a = b;
}
bool LocalPlanner::getControlCmd(const T pose_[3], const T vel_[3], const T target_[3], T cmd_acc_[3]) const {
    Map<const Vector2> pose_xy(pose_);
    Map<const Vector2> vel_xy(vel_);
    Map<const Vector2> target_xy(target_);
    const T pose_yaw = pose_[2];
    const T vel_yaw = vel_[2];
    const T target_yaw = target_[2];

    Map<Vector2> cmd_acc_xy(cmd_acc_);
    T &cmd_acc_yaw = cmd_acc_[2];

    bool xy_goal_reached = false;
    bool yaw_goal_reached = false;

    cmd_acc_xy = target_xy - pose_xy;
    {  // transform to local coordinate
        const T cy = std::cos(pose_yaw), sy = std::sin(pose_yaw);
        Matrix2 rot{{cy, sy}, {-sy, cy}};
        cmd_acc_xy = rot * cmd_acc_xy;
    }
    T dist_xy = cmd_acc_xy.norm();
    if (dist_xy <= params.xy_goal_tolerance) {
        xy_goal_reached = true;
        cmd_acc_xy.setZero();
    } else {
        T vel_lim_xy = std::sqrt(2 * params.acc_lim_xy * dist_xy);
        chkmin(vel_lim_xy, params.vel_lim_xy);
        cmd_acc_xy *= vel_lim_xy / dist_xy;  // the ideal speed
        cmd_acc_xy -= vel_xy;                // the direction of acceleration
        cmd_acc_xy *= predict_freq;
        if (T acc = cmd_acc_xy.norm(); acc > params.acc_lim_xy) cmd_acc_xy *= params.acc_lim_xy / acc;
    }

    cmd_acc_yaw = wrapAngle(target_yaw - pose_yaw);
    T dist_yaw = std::abs(cmd_acc_yaw);
    if (dist_yaw <= params.yaw_goal_tolerance) {
        yaw_goal_reached = true;
        cmd_acc_yaw = 0;
    } else {
        T vel_lim_yaw = std::sqrt(2 * params.acc_lim_yaw * dist_yaw);
        chkmin(vel_lim_yaw, params.vel_lim_yaw);
        cmd_acc_yaw = cmd_acc_yaw > 0 ? vel_lim_yaw : -vel_lim_yaw;  // the ideal speed
        cmd_acc_yaw = wrapAngle(cmd_acc_yaw - vel_yaw);              // the direction of acceleration
        cmd_acc_yaw *= predict_freq;
        if (std::abs(cmd_acc_yaw) > params.acc_lim_yaw) cmd_acc_yaw = cmd_acc_yaw > 0 ? params.acc_lim_yaw : -params.acc_lim_yaw;
    }

    return xy_goal_reached && yaw_goal_reached;
}

}  // namespace pose_kalman