#include "utils/FuncThread.hpp"
//

#include "Eigen/Eigen"
#include "devices.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/NoiseGenerator.hpp"
#include "pose_kalman/params.hpp"
#include "pose_kalman/utils.hpp"
namespace pose_kalman {

static constexpr T xy_sigma2 = 0.01;
static constexpr T yaw_sigma2 = 0.03;
using Vector3 = Eigen::Matrix<T, 3, 1>;
using Matrix2 = Eigen::Matrix<T, 2, 2>;
static void testLocalPlannerEntry() {
    static SerialIO::TxUtil<float, 6, true> pose_tx("pose", 30);
    static LocalPlanner lp;
    static NoiseGenerator xy_noise(xy_sigma2);
    static NoiseGenerator yaw_noise(yaw_sigma2);

    lp.params.acc_lim_xy = 1;
    lp.params.acc_lim_yaw = 1;
    lp.params.vel_lim_xy = 2;
    lp.params.vel_lim_yaw = 3;

    T state[6]{0};

    Eigen::Map<Vector3> pose(state);
    Eigen::Map<Vector3> vel(state + 3);
    MoveBase::Goal goal{10, 3, 3.14, 0.1, 0.1, false};
    Vector3 cmd_vel{0, 0, 0};

    for (;;) {
        if (lp.getControlCmd(pose.data(), vel.data(), goal, cmd_vel.data())) {}
        cmd_vel[0] += xy_noise(), cmd_vel[1] += xy_noise(), cmd_vel[2] += yaw_noise();
        vel += cmd_vel;
        vel *= 0.5;
        // transform to global coordinate
        const T cy = std::cos(pose[2]), sy = std::sin(pose[2]);
        Matrix2 rot{{cy, -sy}, {sy, cy}};
        Vector3 global_vel = vel;
        global_vel.segment(0, 2).noalias() = rot * vel.segment(0, 2);
        pose += global_vel * predict_period_s;
        pose[2] = wrapAngle(pose[2]);
        vel = cmd_vel;
        if (pose_tx.txFinished()) {
            pose_tx.setArr(state);
            wireless.send(pose_tx);
        }
        rt_thread_mdelay(predict_period_ms);
    }
}

}  // namespace pose_kalman

bool testLocalPlannerNode() { return FuncThread(pose_kalman::testLocalPlannerEntry, "testLocalPlanner"); }