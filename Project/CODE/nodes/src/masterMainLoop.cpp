#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include <algorithm>
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "RectConfig.hpp"
#include "TSP/TSP.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

static int coords_cnt = 1;
static float coords[target_coords_maxn + 1][2];

static bool navigationStarted = false;
static int currentTarget;
#define debug_ips(s) ips114_showstr(188, 4, s)
static TSP::TSP_Solver tsp;

constexpr int mainloop_timeout = 500;

enum class CurrentState {
    IDLE,
    RESET,
    GET_COORDS,
    SOLVE_TSP,
    NAVIGATION,
};

static CurrentState currentState;

static constexpr float borderWidth = 7, borderHeight = 5;
constexpr float tsp_k = std::min((M / 4) / borderWidth, (N / 4) / borderHeight);

static inline void Reset() {
    ips114_clear(WHITE);
    navigationStarted = false;
    // TODO: 让从机也重置
    currentState = CurrentState::GET_COORDS;
}

static inline void sendCoords() {
    static SerialIO::TxArr<float, (target_coords_maxn + 1) * 2, true> a4_tx(32, "a4_tx");
    a4_tx.txFinished(-1);
    a4_tx.setArr(coords[0], coords_cnt * 2);
    wireless.send(a4_tx);
}

static inline void GetCoords() {
    if (!masterGlobalVars.wait_for_coord_recv(mainloop_timeout)) return;

    masterGlobalVars.get_coord_recv(coords_cnt, coords[1]);
    ++coords_cnt;
    sendCoords();
    draw_corr(coords, coords_cnt, 7, 5);

    currentState = CurrentState::SOLVE_TSP;
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

    currentState = CurrentState::NAVIGATION;
}

static inline void Navigation() {
    if (!navigationStarted) {
        currentTarget = 0;
        navigationStarted = true;
    } else {
        auto result = moveBase.wait_for_result(mainloop_timeout);
        switch (result) {
        case MoveBase::GoalEventFlag::timeout: return;
        case MoveBase::GoalEventFlag::disabled: currentState = CurrentState::IDLE; return;
        default: break;
        }
    }
    if (++currentTarget > coords_cnt) {
        currentState = CurrentState::IDLE;
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

static void masterMainLoopEntry() {
    currentState = CurrentState::RESET;
    for (;;) {
        ips114_showint8(188, 4, (int)currentState);
        switch (currentState) {
        case CurrentState::IDLE: break;
        case CurrentState::RESET: Reset(); break;
        case CurrentState::GET_COORDS: GetCoords(); break;
        case CurrentState::SOLVE_TSP: SolveTSP(); break;
        case CurrentState::NAVIGATION: Navigation(); break;
        default: currentState = CurrentState::RESET; break;
        }
    }
}

bool masterMainLoopNode() { return FuncThread(masterMainLoopEntry, "masterMainLoop", 4096); }