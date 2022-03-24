#ifndef _FakeAtomic_hpp
#define _FakeAtomic_hpp

#include <rthw.h>

template <typename T> class FakeAtomicLoader {
    T _tmp;
    bool _changed = false;

 public:
    void store(const T& data) {
        rt_base_t level = rt_hw_interrupt_disable();
        _tmp = data, _changed = true;
        rt_hw_interrupt_enable(level);
    }
    bool load(T& data) {
        bool res = _changed;
        rt_base_t level = rt_hw_interrupt_disable();
        if (_changed) data = _tmp, _changed = false;
        rt_hw_interrupt_enable(level);
        return res;
    }
};

#endif  // _FakeAtomic_hpp