#include "utils/FuncThread.hpp"
//
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "SlaveGlobalVars.hpp"
#include "devices.hpp"
#include "magnetConfig.hpp"
#include "masterConfig.hpp"
#include "naviParam.hpp"
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
    for (auto& mag : magnets) mag.set(1);
    MoveBase::Goal goal_navi = GOAL_NAVI;
    MoveBase::Goal goal_pick = GOAL_PICK;
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

        goal_navi.x = target_x, goal_navi.y = target_y, goal_navi.yaw = state.yaw();
        moveBase.send_goal(goal_navi);

        masterGlobalVars.send_rects_enabled(true, 1e6);
        moveBase.wait_for_result();
        masterGlobalVars.send_rects_enabled(false);

        moveBase.get_state(state);
        T pose_xy[2]{state.x(), state.y()};
        T target_pos[3];
        magnetAlign(pose_xy, masterGlobalVars.coords[1], magnet_idx, target_pos);

        goal_pick.x = target_pos[0], goal_pick.y = target_pos[1], goal_pick.yaw = target_pos[2];
        moveBase.send_goal(goal_pick);
        moveBase.wait_for_result();
        auto& srv = (magnet_idx & 1 ? srv_r : srv_l);
        srv.max();
        rt_thread_mdelay(grab_srv_down_delay_ms);
        srv.min();

        if (++magnet_idx == magnet::cnt) magnet_idx = 0;
    }
}

bool testRectGrabNode() { return FuncThread(testRectGrabEntry, "testRectGrab", 2048, Thread::lowest_priority); }