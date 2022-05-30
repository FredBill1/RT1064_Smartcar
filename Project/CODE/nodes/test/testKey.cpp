#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

static void testKeyEntry() {
    for (int i;;) {
        i = 0;
        for (auto& gpio : master_key) ips114_showchar(i++ << 3, 0 << 4, '0' + gpio.get());

        i = 0;
        for (auto& gpio : master_switch) ips114_showchar(i++ << 3, 1 << 4, '0' + gpio.get());

        i = 0;
        for (auto& gpio : slave_key) ips114_showchar(i++ << 3, 2 << 4, '0' + gpio.get());

        i = 0;
        for (auto& gpio : slave_switch) ips114_showchar(i++ << 3, 3 << 4, '0' + gpio.get());

        rt_thread_mdelay(100);
    }
}

bool testKeyNode() { return FuncThread(testKeyEntry, "testKey"); }
