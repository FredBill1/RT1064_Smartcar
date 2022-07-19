#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

static void testMagnetEntry() {
    for (auto &mag : magnets) mag.set(0);
    for (;;) {
        for (int i = 0; i < sizeof(magnets) / sizeof(magnets[0]); ++i) {
            ips.printf("%d\n", i);
            magnets[i].set(1);
            rt_thread_mdelay(1000);
            magnets[i].set(0);
        }
    }
}

bool testMagnetNode() { return FuncThread(testMagnetEntry, "testMagnet"); }
