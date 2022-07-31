#ifndef _nodes_masterMainLoop_hpp
#define _nodes_masterMainLoop_hpp

#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "MasterGlobalVars.hpp"
#include "RectConfig.hpp"
#include "SlaveGlobalVars.hpp"
#include "TSP/TSP.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"
#include "pose_kalman/magnetAlign.hpp"
#include "pose_kalman/utils.hpp"
//
#include "fieldParam.hpp"
#include "masterConfig.hpp"
#include "naviParam.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

constexpr float tsp_k = std::min((M / 4) / borderWidth, (N / 4) / borderHeight);

static int& coords_cnt = masterGlobalVars.coords_cnt;
static auto& coords = masterGlobalVars.coords;
static TSP::TSP_Solver tsp;

constexpr int key_cnt = sizeof(master_key) / sizeof(master_key[0]), switch_cnt = sizeof(master_switch) / sizeof(master_switch[0]);
static bool key_pressing[key_cnt];
// static bool switch_on[switch_cnt];
static inline void keyScan() {
    for (int i = 0; i < key_cnt; ++i) key_pressing[i] = master_key[i].pressing();
    // for (int i = 0; i < switch_cnt; ++i) switch_on[i] = master_switch[i].get();
}

// clang-format off
#define Task_t static inline bool
#define RUN_TASK(entry) { if (masterGlobalVars.reset_requested() || !(entry)) return false; }
#define GUARD_COND(cond) { if (!(cond)) return false; }
#define CHECK_RESET_SIGNAL() { if (masterGlobalVars.reset_requested()) return false; }
#define CHECK_KEY_RESET() { keyScan(); if (key_pressing[0]) return false; }
#define WAIT_FOR(cond) {      \
    while (!(cond)) {         \
        CHECK_KEY_RESET();    \
        CHECK_RESET_SIGNAL(); \
    }                         \
}
#define SHOW_STATE(s) ips114_showstr(188, 4, s);
// clang-format on

namespace utils {
static inline void clear_screen() { ips114_clear(WHITE); }
static inline void sendSlaveTask(SlaveGlobalVars::State task, bool blocking = false) {
    static SerialIO::TxFlag<true> task_tx("task_tx", 255);
    if (task_tx.txFinished(blocking ? -1 : 0)) {
        task_tx.set(task);
        slave_uart.send(task_tx);
    }
}
static inline void sendCoords() {
    static SerialIO::TxArr<float, target_coords_maxn * 2, true> a4_tx(32, "a4_tx");
    a4_tx.txFinished(-1);
    a4_tx.setArr(coords[1], coords_cnt * 2);
    wireless.send(a4_tx);
}
static inline void sendArtSnapshotTask() {
    static uint8_t cmd_id = 0xA5;
    static SerialIO::TxXfer art_xfer(&cmd_id, 1, "art_snapshot_tx");
    art_xfer.txFinished(-1);
    art_uart.send(art_xfer);
}
static inline void sendArtBorderTask() {
    static uint8_t cmd_id = 0x5A;
    static SerialIO::TxXfer art_xfer(&cmd_id, 1, "art_border_tx");
    art_xfer.txFinished(-1);
    art_uart.send(art_xfer);
}
static inline float calcDist2(const float a[2], const float b[2]) {
    float dx = a[0] - b[0], dy = a[1] - b[1];
    return dx * dx + dy * dy;
}
static inline float calcDist(const float a[2], const float b[2]) { return calcDist2(a, b); }

// 假定x是atan2算出来的，在[-pi, pi]之间
constexpr pose_kalman::T wrapAngleNear(pose_kalman::T x, pose_kalman::T cur) {
    using pose_kalman::wrapAngle;
    if (std::abs(wrapAngle(x - cur)) > PI_2) x = wrapAngle(x + PI);
    return x;
}
Task_t moveBaseReachedCheck() {
    for (;;) {
        CHECK_KEY_RESET();
        CHECK_RESET_SIGNAL();
        switch (moveBase.wait_for_result(mainloop_timeout)) {
        case MoveBase::GoalEventFlag::disabled: return false;
        case MoveBase::GoalEventFlag::reached: return true;
        case MoveBase::GoalEventFlag::timeout: break;
        }
    }
}
#define WAIT_MOVE_BASE_GOAL_REACHED GUARD_COND(utils::moveBaseReachedCheck());
#define WAIT_MOVE_BASE_GOAL_NEAR WAIT_FOR(moveBase.wait_near(mainloop_timeout));
Task_t waitArtSnapshot() {
    rt_tick_t start_ms = rt_tick_get_millisecond();
    for (;;) {
        CHECK_KEY_RESET();
        CHECK_RESET_SIGNAL();
        if (masterGlobalVars.wait_art_snapshot(mainloop_timeout)) break;
        if (rt_tick_get_millisecond() - start_ms > art_snapshot_timeout_ms) {
            // TODO art拍照超时
            // masterGlobalVars.send_art_snapshot();
            // masterGlobalVars.send_art_result(ResultCatgory::Major(rand() % 3));
        }
    }
    return true;
}

Task_t gotoTarget(pose_kalman::T target_x, pose_kalman::T target_y) {
    MoveBase::State state;
    moveBase.get_state(state);

    pose_kalman::T yaw = std::atan2(target_y - state.y(), target_x - state.x());

    MoveBase::Goal goal = GOAL_NAVI_TURN;
    goal.x = state.x();
    goal.y = state.y();
    goal.yaw = yaw;
    moveBase.send_goal(goal);
    WAIT_MOVE_BASE_GOAL_NEAR;

    goal = GOAL_NAVI_MOVE;
    goal.x = target_x;
    goal.y = target_y;
    goal.yaw = yaw;
    moveBase.send_goal(goal);
    WAIT_MOVE_BASE_GOAL_REACHED;

    return true;
}

}  // namespace utils

namespace CarryOrder {

static ResultCatgory::Major catgory[3];
static pose_kalman::T targets[3][2];
static uint8_t catgory_to_index[3];

static inline void calc() {
    using namespace ResultCatgory;

    int last_target = tsp.hamilton_path[coords_cnt];
    float last_x = coords[last_target][0], last_y = coords[last_target][1];
    float dx = 2 * fieldWidth - last_x, dy = 2 * fieldHeight - last_y;
    float x = last_x + dx * (fieldHeight - last_y) / dy, y = last_y + dy * (fieldWidth - last_x) / dx;
    int idx = x > fieldWidth;
    idx ? (x = 2 * fieldWidth - x) : (y = 2 * fieldHeight - y);

    catgory[idx ^ 1] = Major::fruit;  // 右
    targets[idx ^ 1][0] = fieldWidth + carryExtendPadding;
    targets[idx ^ 1][1] = std::min(y, fieldHeight - carrySidePadding);

    catgory[idx] = Major::vehicle;  // 上
    targets[idx][0] = std::min(x, fieldWidth - carryExtendPadding);
    targets[idx][1] = fieldHeight + carryExtendPadding;

    catgory[2] = Major::animal;  // 左
    targets[2][0] = -carryExtendPadding;
    targets[2][1] = garage_position[1];

    for (int i = 0; i < 2; ++i) catgory_to_index[(int)catgory[i]] = i;
}

}  // namespace CarryOrder

#endif  // _nodes_masterMainLoop_hpp