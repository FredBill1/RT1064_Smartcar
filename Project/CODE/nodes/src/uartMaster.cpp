#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "RectConfig.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

#define slave_uart uart3

static Beep beep;
static uint8_t id;

void recvCoords(SerialIO& uart, Beep& beep) {
    decltype(target_coords_cnt) cnt_tmp;
    decltype(target_coords_corr) coords_tmp;

    if (!uart.getchar(id)) return;
    cnt_tmp = id / 2;
    if (id & 1 || cnt_tmp > target_coords_maxn) return;
    if (!uart.getArr<float, target_coords_maxn * 2>(coords_tmp[0], cnt_tmp * 2)) return;
    beep.set(false);
    masterGlobalVars.send_coord_recv(cnt_tmp, coords_tmp[0]);
}

static inline void recvRect() {
    static SerialIO::TxArr<float, max_rect_cnt * 2, true> rect_tx(33, "rect_tx");
    static float rects[max_rect_cnt][2];

    uint64_t timestamp_us = systick.get_us();
    if (!slave_uart.getchar(id)) return;
    int rect_cnt = id / 2;
    if (id & 1 || rect_cnt > max_rect_cnt) return;
    if (!slave_uart.getArr<float, max_rect_cnt * 2>(rects[0], rect_cnt * 2)) return;
    beep.set(false);
    if (rect_cnt) {
        MoveBase::State state;
        moveBase.get_state(state);
        float pos[3]{(float)state.x(), (float)state.y(), (float)state.yaw()};
        masterGlobalVars.send_rects(pos, rects[0], rect_cnt, timestamp_us);
    }
    if (rect_tx.txFinished()) {
        rect_tx.setArr(rects[0], rect_cnt * 2);
        wireless.send(rect_tx);
    }
}

static void uartMasterEntry() {
    for (;;) {
        slave_uart.waitHeader();
        beep.set(true);
        if (!slave_uart.getchar(id)) continue;
        switch (id) {
        case 32: recvCoords(slave_uart, beep); break;
        case 33: recvRect(); break;
        }
    }
}

bool uartMasterNode() { return FuncThread(uartMasterEntry, "uartMaster", 4096, Thread::uart_priority); }