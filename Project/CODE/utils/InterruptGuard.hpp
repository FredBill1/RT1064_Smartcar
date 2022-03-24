#ifndef _InterruptGuard_hpp
#define _InterruptGuard_hpp

#include <rthw.h>

class InterruptGuard {
    rt_base_t level;

 public:
    InterruptGuard() { level = rt_hw_interrupt_disable(); }
    ~InterruptGuard() { rt_hw_interrupt_enable(level); }
};

#endif  // _InterruptGuard_hpp