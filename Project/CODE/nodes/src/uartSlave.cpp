#include "utils/FuncThread.hpp"
//
#include "SlaveGlobalVars.hpp"
#include "devices.hpp"
#include "slaveConfig.hpp"

static uint8_t id;

static inline void ConfigCamera() {
    if (!master_uart.getchar(id)) return;
    float data;
    if (!master_uart.getData<float>(data)) return;
    switch (id) {
    case 0: camera.set_auto_exposure(data), camera.write_config(); break;
    case 1: camera.set_exposure_time_fast(data); break;
    case 2: camera.set_gain(data), camera.write_config(); break;
    }
}

static inline void RecvTask() {
    if (!master_uart.getchar(id)) return;
    slaveGlobalVars.set_state((SlaveGlobalVars::State)id);
}

static void uartSlaveEntry() {
    for (;;) {
        master_uart.waitHeader();
        if (!master_uart.getchar(id)) continue;
        switch (id) {
        case 12: ConfigCamera(); break;
        case 255: RecvTask(); break;
        }
    }
}

bool uartSlaveNode() { return FuncThread(uartSlaveEntry, "uartSlave", 4096, Thread::uart_priority); }