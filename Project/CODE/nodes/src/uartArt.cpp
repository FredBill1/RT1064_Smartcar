#include "utils/FuncThread.hpp"
//

#include "MasterGlobalVars.hpp"
#include "artResult/ResultSender.hpp"
#include "devices.hpp"
#include "masterConfig.hpp"

static Beep beep;
static uint8_t id;

static void uartArtEntry() {
    static ResultSender resultSender(upload_uart, "art_result");
    for (;;) {
        art_uart.waitHeader();
        beep.set(1);
        if (!art_uart.getchar(id) || id != 0xF0) continue;
        if (!art_uart.getchar(id)) continue;
        if (id == 0xFF) {
            masterGlobalVars.send_art_snapshot();
        } else if (id < 15) {
            auto catgory = ResultCatgory::id_to_minor(id);
            if (masterGlobalVars.send_art_result(catgory)) resultSender.send_catgory(catgory, RT_WAITING_FOREVER);
        } else {
            continue;
        }
        beep.set(0);
    }
}

bool uartArtNode() { return FuncThread(uartArtEntry, "uartArt", 2048, Thread::uart_priority); }