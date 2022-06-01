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

static inline void recvCoords() {
    decltype(target_coords_cnt) cnt_tmp;
    decltype(target_coords_corr) coords_tmp;

    if (!slave_uart.getchar(id)) return;
    cnt_tmp = id / 2;
    if (id & 1 || cnt_tmp > target_coords_maxn) return;
    if (!slave_uart.getArr<float, target_coords_maxn * 2>(coords_tmp[0], cnt_tmp * 2)) return;
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
        float target[2];
        if (!masterGlobalVars.get_rectTarget(target)) return;
        float sy = std::sin(state.yaw()), cy = std::cos(state.yaw());

        float min_dist = 1e9;
        float res_x, res_y;
        for (int i = 0; i < rect_cnt; ++i) {
            float x = target[0] - rects[i][0] * cy + rects[i][1] * sy;
            float y = target[1] - rects[i][0] * sy - rects[i][1] * cy;
            float dx = x - state.x(), dy = y - state.y();
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                res_x = x, res_y = y;
            }
        }
        pose_kalman::T res[2]{res_x, res_y};
        pose_kalman::kf.enqueMeasurement(pose_kalman::MeasurementType::Rect, res, timestamp_us);
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
        case 32: recvCoords(); break;
        case 33: recvRect(); break;
        }
    }
}

bool uartMasterNode() { return FuncThread(uartMasterEntry, "uartMaster", 4096, 0); }