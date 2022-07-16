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
bool LocalPlanner::getControlCmd(const T pose_[3], const T vel_[3], const MoveBase::Goal &goal, T cmd_vel_[3]) const {
    Map<Vector2> cmd_vel_xy(cmd_vel_);
    T &cmd_vel_yaw = cmd_vel_[2];

    Map<const Vector2> pose_xy(pose_);
    // Vector2 vel_xy = cmd_vel_xy;
    Map<const Vector2> vel_xy(vel_);
    const Vector2 target_xy{goal.x, goal.y};
    const T pose_yaw = pose_[2];
    // const T vel_yaw = cmd_vel_yaw;
    const T vel_yaw = vel_[2];
    const T target_yaw = goal.yaw;

    bool xy_goal_reached = false;
    bool yaw_goal_reached = false;

    T pose_yaw_simulate = pose_yaw + vel_yaw * (params.dt_ref * 0.5);
    T pose_yaw_dif = wrapAngle(target_yaw - pose_yaw_simulate);
    if (abs(pose_yaw_dif) <= goal.yaw_tolerance) yaw_goal_reached = true;

    cmd_vel_yaw = std::sqrt(2 * params.acc_lim_yaw * abs(pose_yaw_dif));
    chkmin(cmd_vel_yaw, params.vel_lim_yaw);
    // if (T vel_xy_norm = vel_xy.norm(); vel_xy_norm > eps) chkmin(cmd_vel_yaw, params.acc_lim_xy / vel_xy_norm);
    if (pose_yaw_dif < 0) cmd_vel_yaw = -cmd_vel_yaw;

    T vel_yaw_dif = cmd_vel_yaw - vel_yaw;
    T vel_yaw_dif_lim = params.acc_lim_yaw * params.dt_ref;
    if (std::abs(vel_yaw_dif) > vel_yaw_dif_lim) {
        cmd_vel_yaw = vel_yaw_dif > 0 ? vel_yaw_dif_lim : -vel_yaw_dif_lim;
        cmd_vel_yaw += vel_yaw;
        yaw_goal_reached = false;
    }

    pose_yaw_simulate = pose_yaw + (cmd_vel_yaw + vel_yaw) * (params.dt_ref * 0.25);

    // 将全局坐标系下的目标点坐标转换为底盘坐标系下的坐标，并得到从当前位置到目标点的位置差向量
    const T cy = std::cos(pose_yaw_simulate), sy = std::sin(pose_yaw_simulate);
    Matrix2 rot{{cy, -sy}, {sy, cy}};
    Vector2 pose_xy_simulate = pose_xy + rot * vel_xy * (params.dt_ref * 0.5);
    rot(0, 1) = sy, rot(1, 0) = -sy;
    Vector2 pose_xy_dif = rot * (target_xy - pose_xy_simulate);
    T pose_xy_dif_norm = pose_xy_dif.norm();

    // 计算与位置差向量同向的目标速度分量
    Vector2 cmd_vel_xy_parallel;
    if (pose_xy_dif_norm <= goal.xy_tolerance) {
        xy_goal_reached = true;
        pose_xy_dif.setZero();
        pose_xy_dif_norm = 0;
        cmd_vel_xy_parallel.setZero();
    } else {
        pose_xy_dif /= pose_xy_dif_norm;
        T cmd_vel_xy_parallel_norm = std::sqrt(2 * params.acc_lim_xy * pose_xy_dif_norm);
        chkmin(cmd_vel_xy_parallel_norm, params.vel_lim_xy);
        cmd_vel_xy_parallel.noalias() = pose_xy_dif * cmd_vel_xy_parallel_norm;
    }

    const T vel_xy_delta_lim = params.acc_lim_xy * params.dt_ref;

    // 将当前底盘速度沿位置差方向正交分解
    Vector2 vel_xy_parallel = pose_xy_dif.dot(vel_xy) * pose_xy_dif;
    Vector2 vel_xy_orthogonal = vel_xy - vel_xy_parallel;

    Vector2 vel_xy_orthogonal_delta = -vel_xy_orthogonal;
    T vel_xy_orthogonal_delta_norm = vel_xy_orthogonal_delta.norm();
    if (vel_xy_orthogonal_delta_norm > vel_xy_delta_lim) {
        xy_goal_reached = false;
        vel_xy_orthogonal_delta *= vel_xy_delta_lim / vel_xy_orthogonal_delta_norm;
        vel_xy_orthogonal_delta_norm = vel_xy_delta_lim;
    }
    vel_xy_orthogonal += vel_xy_orthogonal_delta;

    const T vel_xy_parallel_delta_lim =
        std::sqrt(vel_xy_delta_lim * vel_xy_delta_lim - vel_xy_orthogonal_delta_norm * vel_xy_orthogonal_delta_norm);

    Vector2 vel_xy_parallel_delta = cmd_vel_xy_parallel - vel_xy_parallel;
    T vel_xy_parallel_delta_norm = vel_xy_parallel_delta.norm();
    if (vel_xy_parallel_delta_norm > vel_xy_parallel_delta_lim) {
        xy_goal_reached = false;
        vel_xy_parallel_delta *= vel_xy_parallel_delta_lim / vel_xy_parallel_delta_norm;
        vel_xy_parallel_delta_norm = vel_xy_parallel_delta_lim;
    }
    vel_xy_parallel += vel_xy_parallel_delta;

    cmd_vel_xy = vel_xy_orthogonal + vel_xy_parallel;

    return xy_goal_reached && yaw_goal_reached;
}

}  // namespace pose_kalman