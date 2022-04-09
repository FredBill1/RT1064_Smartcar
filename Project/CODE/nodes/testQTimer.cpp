#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

static void testQTimerEntry() {
    for (;;) {
        ips.printf("%7d%7d %7d%7d\n", qtimerL1.get(), qtimerL2.get(), qtimerR1.get(), qtimerR2.get());
        rt_thread_mdelay(100);
    }
}

bool testQTimerNode() { return FuncThread(testQTimerEntry, "testQTimer"); }