#include "Systick.hpp"

extern "C" {
#include "zf_systick.h"
}

void Systick::init() {
    systick_start();
    freq = DELAY_CLK;
}
uint32_t Systick::get() const { return systick_getval(); }
uint64_t Systick::get_ms() const { return COUNT_TO_MSEC(get(), freq); }
uint64_t Systick::get_us() const { return COUNT_TO_USEC(get(), freq); }
uint64_t Systick::get_ns() const { return COUNT_TO_USEC(get(), freq / 1000); }

void Systick::delay(uint32_t tick) const {
    uint32_t start = get();
    while (get() - start < tick)
        ;
}
void Systick::delay_ms(uint32_t ms) const { delay(MSEC_TO_COUNT(ms, freq)); }
void Systick::delay_us(uint32_t us) const { delay(USEC_TO_COUNT(us, freq)); }
void Systick::delay_ns(uint32_t ns) const { delay(USEC_TO_COUNT(ns, freq / 1000)); }