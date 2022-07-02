#include "utils/FuncThread.hpp"

//
#include "artResult/ResultSender.hpp"
#include "devices.hpp"
#include "masterConfig.hpp"

static void sendTraverseEntry() {
    static ResultSender resultSender(upload_uart, "art_traverse");
    for (;;) {
        resultSender.send_traverse(false);
        rt_thread_mdelay(100);
    }
}

bool sendTraverseNode() { return FuncThread(sendTraverseEntry, "sendTraverse", 2048, Thread::default_priority); }