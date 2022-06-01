#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include <algorithm>
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "RectConfig.hpp"
#include "SlaveGlobalVars.hpp"
#include "TSP/TSP.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

static int& coords_cnt = masterGlobalVars.coords_cnt;
static auto& coords = masterGlobalVars.coords;

static bool navigationStarted = false;
static int currentTarget;
#define debug_ips(s) ips114_showstr(188, 4, s)
static TSP::TSP_Solver tsp;

#define slave_uart uart3

constexpr int mainloop_timeout = 500;

static constexpr float borderWidth = 7, borderHeight = 5;
constexpr float tsp_k = std::min((M / 4) / borderWidth, (N / 4) / borderHeight);

static inline void sendTask(SlaveGlobalVars::State task, bool blocking = false) {
    static SerialIO::TxFlag<true> task_tx("task_tx", 255);
    if (task_tx.txFinished(blocking ? -1 : 0)) {
        task_tx.set(task);
        slave_uart.send(task_tx);
    }
}

static inline void Reset() {
    sendTask(SlaveGlobalVars::RESET);
    MoveBase::State state(systick.get_us(), 0, 0, 0, 0, 0, 0);
    moveBase.send_set_state(state);

    ips114_clear(WHITE);
    navigationStarted = false;
    masterGlobalVars.set_state(MasterGlobalVars::GET_COORDS);
}

static inline void sendCoords() {
    static SerialIO::TxArr<float, (target_coords_maxn + 1) * 2, true> a4_tx(32, "a4_tx");
    a4_tx.txFinished(-1);
    a4_tx.setArr(coords[0], coords_cnt * 2);
    wireless.send(a4_tx);
}

static inline void GetCoords() {
    if (!masterGlobalVars.wait_for_coord_recv(mainloop_timeout)) return;
    sendTask(SlaveGlobalVars::RECT);

    masterGlobalVars.get_coord_recv();
    sendCoords();
    draw_corr(coords, coords_cnt, 7, 5);

    masterGlobalVars.set_state(MasterGlobalVars::SOLVE_TSP);
}

static inline float calcDist(const float a[2], const float b[2]) {
    float dx = a[0] - b[0], dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

static inline void SolveTSP() {
    tsp.N = coords_cnt;
    for (int i = 0; i < coords_cnt; ++i) {
        tsp.dist[i][i] = 0;
        for (int j = i + 1; j < coords_cnt; ++j) tsp.dist[i][j] = tsp.dist[j][i] = calcDist(coords[i], coords[j]);
    }
    tsp.solve();

    {  // 距离起点更近的点作为第一个点
        int u = tsp.hamilton_path[1], v = tsp.hamilton_path[coords_cnt - 1];
        if (calcDist(coords[0], coords[u]) > calcDist(coords[0], coords[v]))
            std::reverse(tsp.hamilton_path + 1, tsp.hamilton_path + coords_cnt);
    }

    for (int i = 1; i < coords_cnt; ++i) {
        int u = tsp.hamilton_path[i - 1], v = tsp.hamilton_path[i];
        drawLine(coords[u][0] * tsp_k, coords[u][1] * tsp_k, coords[v][0] * tsp_k, coords[v][1] * tsp_k,
                 [](int x, int y) { ips114_drawpoint(x, N / 4 - y, ips.Red); });
    }

    masterGlobalVars.set_state(MasterGlobalVars::NAVIGATION);
}

static inline void Navigation() {
    if (!navigationStarted) {
        currentTarget = 0;
        navigationStarted = true;
    } else {
        auto result = moveBase.wait_for_result(mainloop_timeout);
        switch (result) {
        case MoveBase::GoalEventFlag::timeout: return;
        case MoveBase::GoalEventFlag::disabled: masterGlobalVars.set_state(MasterGlobalVars::IDLE); return;
        default: break;
        }
    }
    if (++currentTarget > coords_cnt) {
        masterGlobalVars.set_state(MasterGlobalVars::IDLE);
        return;
    }
    MoveBase::State state;
    moveBase.get_state(state);
    int cur = currentTarget < coords_cnt ? currentTarget : 0;
    cur = tsp.hamilton_path[cur];
    float x = coords[cur][0], y = coords[cur][1], yaw = std::atan2(y - state.y(), x - state.x());
    constexpr float dist = 0.2;
    moveBase.send_goal(x - dist * std::cos(yaw), y - dist * std::sin(yaw), yaw);
}

static inline void Idle() {
    if (master_key[0].pressing()) { masterGlobalVars.set_state(MasterGlobalVars::RESET); }
}

static void masterMainLoopEntry() {
    for (;;) {
        auto state = masterGlobalVars.get_state();
        ips114_showstr(188, 4, masterGlobalVars.state_str(state));
        switch (state) {
        case MasterGlobalVars::IDLE: Idle(); break;
        case MasterGlobalVars::RESET: Reset(); break;
        case MasterGlobalVars::GET_COORDS: GetCoords(); break;
        case MasterGlobalVars::SOLVE_TSP: SolveTSP(); break;
        case MasterGlobalVars::NAVIGATION: Navigation(); break;
        default: masterGlobalVars.set_state(MasterGlobalVars::IDLE); break;
        }
    }
}

bool masterMainLoopNode() { return FuncThread(masterMainLoopEntry, "masterMainLoop", 4096); }