#ifndef _driver_Systick_hpp
#define _driver_Systick_hpp
#include <cstdint>

class Systick {
    uint32_t freq;

 public:
    void init();
    uint32_t get() const;
    uint64_t get_ms() const;
    uint64_t get_us() const;
    uint64_t get_ns() const;
    void delay(uint32_t tick) const;
    void delay_ms(uint32_t ms) const;
    void delay_us(uint32_t us) const;
    void delay_ns(uint32_t ns) const;
};

#endif  // _driver_Systick_hpp