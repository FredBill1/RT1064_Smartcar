#include "utils/FuncThread.hpp"
//

#include "MasterGlobalVars.hpp"
#include "artResult/ResultSender.hpp"
#include "devices.hpp"
#include "masterConfig.hpp"

static Beep beep;
static uint8_t id;

static inline void Classify() {
    static ResultSender resultSender(upload_uart, "art_result");
    if (!art_uart.getchar(id)) return;
    if (id == 0xFF) {
        masterGlobalVars.send_art_snapshot();
    } else if (id < 15) {
        auto minor = ResultCatgory::id_to_minor(id);
        auto major = ResultCatgory::minor_to_major(minor);
        int index;
        if (masterGlobalVars.send_art_result(major, index)) resultSender.send_catgory(minor, index, RT_WAITING_FOREVER);
    } else
        return;
    beep.set(0);
}

static inline void FindBorder() {
    beep.set(0);
    masterGlobalVars.send_art_border();
}

static void uartArtEntry() {
    for (;;) {
        art_uart.waitHeader();
        beep.set(1);
        if (!art_uart.getchar(id)) continue;
        switch (id) {
        case 0xF0: Classify(); break;
        case 0x0F: FindBorder(); break;
        }
    }
}

bool uartArtNode() { return FuncThread(uartArtEntry, "uartArt", 2048, Thread::uart_priority); }