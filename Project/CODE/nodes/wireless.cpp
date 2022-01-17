#include "Thread.h"
#include "devices.hpp"

inline void SystemReset() { NVIC_SystemReset(); }

void wirelessThreadEntry(void*) {
    wireless.waitHeader();
    uint8 op = wireless.getchar();
    switch (op) {
    case 0: SystemReset(); break;
    }
}

rtthread::Thread wirelessThread(wirelessThreadEntry, NULL, 2048, 0, 20, "wirelessThread");