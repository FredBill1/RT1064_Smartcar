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
constexpr T eps = 1e-4;
bool LocalPlanner::getControlCmd(const T pose_[3], const T vel_[3], const T target_[3], T cmd_vel_[3]) const {
    Map<const Vector2> pose_xy(pose_);
    Map<const Vector2> vel_xy(vel_);
    Map<const Vector2> target_xy(target_);
    const T pose_yaw = pose_[2];
    const T vel_yaw = vel_[2];
    const T target_yaw = target_[2];

    Map<Vector2> cmd_vel_xy(cmd_vel_);
    T &cmd_vel_yaw = cmd_vel_[2];

    bool xy_goal_reached = false;
    bool yaw_goal_reached = false;

    T pose_yaw_simulate = pose_yaw + vel_yaw * (predict_period_s * 0.5);
    T pose_yaw_dif = wrapAngle(target_yaw - pose_yaw_simulate);
    if (abs(pose_yaw_dif) <= params.yaw_goal_tolerance) {
        yaw_goal_reached = true;
        cmd_vel_yaw = 0;
    } else {
        cmd_vel_yaw = std::sqrt(2 * params.acc_lim_yaw * abs(pose_yaw_dif));
        chkmin(cmd_vel_yaw, params.vel_lim_yaw);
        if (pose_yaw_dif < 0) cmd_vel_yaw = -cmd_vel_yaw;
    }
    T vel_yaw_dif = cmd_vel_yaw - vel_yaw;
    T vel_yaw_dif_lim = params.acc_lim_yaw * predict_period_s;
    if (std::abs(vel_yaw_dif) > vel_yaw_dif_lim) {
        cmd_vel_yaw = vel_yaw_dif > 0 ? vel_yaw_dif_lim : -vel_yaw_dif_lim;
        cmd_vel_yaw += vel_yaw;
        yaw_goal_reached = false;
    }
    pose_yaw_simulate = pose_yaw + (cmd_vel_yaw + vel_yaw) * (predict_period_s * 0.25);

    const T cy = std::cos(pose_yaw_simulate), sy = std::sin(pose_yaw_simulate);
    Matrix2 rot{{cy, -sy}, {sy, cy}};
    Vector2 pose_xy_simulate = pose_xy + rot * vel_xy * (predict_period_s * 0.5);
    rot(0, 1) = sy, rot(1, 0) = -sy;
    Vector2 pose_xy_dif = rot * (target_xy - pose_xy_simulate);
    T pose_xy_dif_norm = pose_xy_dif.norm();
    T cmd_vel_xy_norm;
    if (pose_xy_dif_norm <= params.xy_goal_tolerance) {
        xy_goal_reached = true;
        cmd_vel_xy.setZero();
    } else {
        cmd_vel_xy_norm = std::sqrt(2 * params.acc_lim_xy * pose_xy_dif_norm);
        chkmin(cmd_vel_xy_norm, params.vel_lim_xy);
        if (pose_xy_dif_norm > 0) cmd_vel_xy.noalias() = pose_xy_dif * (cmd_vel_xy_norm / pose_xy_dif_norm);
        else
            cmd_vel_xy.setZero();
    }
    T vel_xy_norm = vel_xy.norm();
    T vel_xy_dif_norm = cmd_vel_xy_norm - vel_xy_norm;
    T vel_xy_dif_norm_lim = params.acc_lim_xy * predict_period_s;
    if (std::abs(vel_xy_dif_norm) > vel_xy_dif_norm_lim) {
        T old_cmd_vel_xy_norm = cmd_vel_xy_norm;
        cmd_vel_xy_norm = vel_xy_dif_norm > 0 ? vel_xy_dif_norm_lim : -vel_xy_dif_norm_lim;
        cmd_vel_xy_norm += vel_xy_norm;
        cmd_vel_xy = cmd_vel_xy * (cmd_vel_xy_norm / old_cmd_vel_xy_norm);
        xy_goal_reached = false;
    }

    return xy_goal_reached && yaw_goal_reached;
}

}  // namespace pose_kalman