#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
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
    if (cnt_tmp > target_coords_maxn) return;
    if (!slave_uart.getArr<float, target_coords_maxn * 2>(coords_tmp[0], cnt_tmp * 2)) return;
    beep.set(false);
    masterGlobalVars.send_coord_recv(cnt_tmp, coords_tmp[0]);
}

static inline void recvRect() {
    if (!slave_uart.getchar(id)) return;

    beep.set(false);
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