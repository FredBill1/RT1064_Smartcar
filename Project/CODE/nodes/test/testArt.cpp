#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "MasterGlobalVars.hpp"
#include "devices.hpp"
#define art_uart uart4

static Beep beep;

static void testArtRecvEntry() {
    static uint8_t id;
    for (;;) {
        art_uart.waitHeader();
        beep.set(1);
        if (!art_uart.getchar(id) || id != 0xF0) continue;
        if (!art_uart.getchar(id)) continue;
        beep.set(0);

        if (id == 0xFF) masterGlobalVars.send_art_snapshot();
        else {
            auto minor = ResultCatgory::id_to_minor(id);
            auto major = ResultCatgory::minor_to_major(minor);
            masterGlobalVars.send_art_result(major);
        }
    }
}

static inline void send_art_snapshot_task() {
    static uint8_t cmd_id = 0xA5;
    static SerialIO::TxXfer art_xfer(&cmd_id, 1, "art_snapshot");
    art_xfer.txFinished(-1);
    art_uart.send(art_xfer);
}

static void testArtMainLoopEntry() {
    for (;;) {
        if (!master_key[3].pressing()) {
            rt_thread_mdelay(200);
            continue;
        }
        masterGlobalVars.send_art_cur_index(0);
        send_art_snapshot_task();
        ips.printf("send art snapshot\n");
        masterGlobalVars.wait_art_snapshot();
        ips.printf("snapshot finished\n");
        ResultCatgory::Major result;
        masterGlobalVars.wait_art_result(result);
        ips.printf("result is %d\n\n", (int)result);
    }
}

bool testArtNode() {
    return FuncThread(testArtRecvEntry, "testArtRecv", 2048, 0) && FuncThread(testArtMainLoopEntry, "testArtMainLoop", 2048, 31);
}