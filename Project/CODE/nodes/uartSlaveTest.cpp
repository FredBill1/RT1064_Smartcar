#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

static void uart3RecvEntry() {
    float t;
    static SerialIO::TxXfer tx("uart3tx");
    char a = '0';
    for (;;) {
        uart3.waitHeader();
        if (!uart3.getData<float>(t)) continue;
        ips.printf("%f\n", t);
        if (tx.txFinished()) {
            if (++a == '9' + 1) a = '0';
            tx.setData(&a, 1);
            uart3.send(tx);
        }
    }
}

bool uartSlaveTest() { return FuncThread(uart3RecvEntry, "uartSlaveTest", 1024, 1); }