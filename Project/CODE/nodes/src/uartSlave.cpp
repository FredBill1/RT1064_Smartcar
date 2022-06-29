#include "utils/FuncThread.hpp"
//
#include "SlaveGlobalVars.hpp"
#include "devices.hpp"

#define master_uart uart3
static uint8_t id;

static inline void RecvTask() {
    if (!master_uart.getchar(id)) return;
    slaveGlobalVars.set_state((SlaveGlobalVars::State)id);
}

static void uartSlaveEntry() {
    for (;;) {
        master_uart.waitHeader();
        if (!master_uart.getchar(id)) continue;
        switch (id) {
        case 255: RecvTask(); break;
        }
    }
}

bool uartSlaveNode() { return FuncThread(uartSlaveEntry, "uartSlave", 4096, Thread::uart_priority); }