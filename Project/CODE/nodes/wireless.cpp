#include "nodes.hpp"
//
#include "devices.hpp"

inline void SystemReset() { NVIC_SystemReset(); }

static void wirelessEntry() {
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

bool wirelessNode() { return FuncThread(wirelessEntry, "wireless", 1024, 0, 20); }