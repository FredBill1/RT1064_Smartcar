#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"

using imgProc::apriltag::float_t;
using imgProc::edge_detect::target_coords_corr, imgProc::edge_detect::target_coords_cnt;
static uint8_t id;

static bool try_recv(SerialIO& uart) {
    uart.waitHeader();
    beep.set(true);
    if (!uart.getchar(id)) return false;
    if (!uart.getchar(id)) return false;
    target_coords_cnt = id;
    for (int i = 0; i < target_coords_cnt * 2; ++i)
        if (!uart.getData<float>(((float_t*)target_coords_corr)[i])) return false;
    beep.set(false);
    return true;
}

static void testA4ReceiveEntry() {
    for (;;) {
        if (try_recv(uart3)) {
            rt_kprintf("%d\r\n", target_coords_cnt);
            for (int i = 0; i < target_coords_cnt; ++i) {
                for (int j = 0; j < 2; ++j)
                    rt_kprintf("%d.%02d ", int(target_coords_corr[i][j]), int(target_coords_corr[i][j] * 100) % 100);
                rt_kprintf("\r\n");
            }
            rt_kprintf("\r\n");
        }
    }
}

bool testA4ReceiveNode() { return FuncThread(testA4ReceiveEntry, "testA4Receive", 4096, 0); }