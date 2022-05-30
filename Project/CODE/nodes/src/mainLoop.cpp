#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "GlobalVars.hpp"
#include "RectConfig.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

static int coords_cnt = 1;
static float coords[target_coords_maxn + 1][2];

constexpr int mainloop_timeout = 500;

enum class CurrentState {
    RESET,
    GET_COORDS,
};

static CurrentState currentState;

static inline void Reset() {
    // TODO: 让从机也重置
    currentState = CurrentState::GET_COORDS;
}

static inline void sendCoords() {
    static SerialIO::TxArr<float, target_coords_maxn + 1, true> a4_tx(32, "a4_tx");
    a4_tx.txFinished(-1);
    a4_tx.setArr(coords[0], coords_cnt * 2);
    wireless.send(a4_tx);
}

static inline void GetCoords() {
    if (!globalVars.wait_for_coord_recv(mainloop_timeout)) return;

    draw_corr(coords, coords_cnt, 7, 5, 0xffff);  // 清除上次的坐标
    globalVars.get_coord_recv(coords_cnt, coords[1]);
    ++coords_cnt;
    sendCoords();
    draw_corr(coords, coords_cnt, 7, 5);
}

static void mainLoopEntry() {
    for (;;) {
        switch (currentState) {
        case CurrentState::RESET: Reset(); break;
        case CurrentState::GET_COORDS: GetCoords(); break;

        default: currentState = CurrentState::RESET; break;
        }
    }
}

bool mainLoopNode() { return FuncThread(mainLoopEntry, "mainLoop", 4096); }