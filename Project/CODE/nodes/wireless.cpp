#include "Thread.h"
#include "devices.hpp"

inline void SystemReset() { NVIC_SystemReset(); }

static void wirelessThreadEntry(void*) {
    SerialIO::TxUtil<float, 3> data("recvTest");
    for (;;) {
        wireless.waitHeader();
        int arr[3];
        if (wireless.getArr<float, 3>(arr)) {
            if (data.txFinished()) {
                data.setArr(arr);
                wireless.send(data);
            }
        }
    }
}

rtthread::Thread wirelessThread(wirelessThreadEntry, NULL, 2048, 0, 20, "wirelessThread");