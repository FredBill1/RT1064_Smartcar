#include "Beep.hpp"

#include "devices.hpp"
#include "utils/InterruptGuard.hpp"

int Beep::cnt = 0;

void Beep::check_cnt() {
    InterruptGuard guard;
    if (enabled) {
        if (!cnt++) beep_io.set(true);
    } else {
        if (!--cnt) beep_io.set(false);
    }
}

void Beep::set(bool enabled) {
    if (enabled == this->enabled) return;
    this->enabled = enabled;
    check_cnt();
}