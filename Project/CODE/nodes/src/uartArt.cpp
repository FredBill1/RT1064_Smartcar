#include "utils/FuncThread.hpp"
//

#include "MasterGlobalVars.hpp"
#include "devices.hpp"
#include "masterConfig.hpp"

static Beep beep;
static uint8_t id;

void uartArtEntry() {
    for (;;) {
        art_uart.waitHeader();
        beep.set(1);
        if (!art_uart.getchar(id) || id != 0xF0) continue;
        if (!art_uart.getchar(id)) continue;
        if (id == 0xFF) {
            masterGlobalVars.send_art_snapshot();
        } else if (id < 15) {
            masterGlobalVars.send_art_result(id);
        } else
            beep.set(1);
    }
}

bool uartArtNode() { return FuncThread(uartArtEntry, "uartArt", 2048, Thread::uart_priority); }