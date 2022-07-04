#include "utils/FuncThread.hpp"
//
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "SlaveGlobalVars.hpp"
#include "devices.hpp"
#include "magnetConfig.hpp"
#include "masterConfig.hpp"
#include "pose_kalman/magnetAlign.hpp"

using namespace pose_kalman;

constexpr T rect_dist_sim = 1.5;

static inline void sendSlaveTask(SlaveGlobalVars::State task) {
    static SerialIO::TxFlag<true> task_tx("task_tx", 255);
    if (task_tx.txFinished(-1)) {
        task_tx.set(task);
        slave_uart.send(task_tx);
    }
}

void testRectGrabEntry() {
    int magnet_idx = 0;
    for (auto mag : magnets) mag.set(1);
    for (;;) {
        if (!master_key[0].pressing()) continue;
        sendSlaveTask(SlaveGlobalVars::RECT);
        masterGlobalVars.reset_states();
        MoveBase::State state;
        moveBase.get_state(state);

        T cy = std::cos(state.yaw()), sy = std::sin(state.yaw());
        T rect_x = state.x() + cy * rect_dist_sim, rect_y = state.y() + sy * rect_dist_sim;
        T target_x = state.x() + cy * (rect_dist_sim - art_cam_dist), target_y = state.y() + sy * (rect_dist_sim - art_cam_dist);
        masterGlobalVars.coords_cnt = 1;
        masterGlobalVars.coords[1][0] = rect_x, masterGlobalVars.coords[1][1] = rect_y;
        masterGlobalVars.coord_valid.set(1, true);

        moveBase.send_goal(target_x, target_y, state.yaw());

        masterGlobalVars.send_rects_enabled(true, 1e6);
        moveBase.wait_for_result();
        masterGlobalVars.send_rects_enabled(false);

        moveBase.get_state(state);
        T pose_xy[2]{state.x(), state.y()};
        T target_pos[3];
        magnetAlign(pose_xy, masterGlobalVars.coords[1], magnet_idx, target_pos);

        moveBase.send_goal(target_pos[0], target_pos[1], target_pos[2]);
        moveBase.wait_for_result();
        auto& srv = (magnet_idx & 1 ? srv2 : srv1);
        srv.set(90);
        rt_thread_mdelay(500);
        srv.set(10);

        if (++magnet_idx == magnet::cnt) magnet_idx = 0;
    }
}

bool testRectGrabNode() { return FuncThread(testRectGrabEntry, "testRectGrab", 2048, Thread::lowest_priority); }