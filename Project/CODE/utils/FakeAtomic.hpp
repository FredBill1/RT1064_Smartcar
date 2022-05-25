#ifndef _FakeAtomic_hpp
#define _FakeAtomic_hpp

#include <rthw.h>

#include <utility>

template <typename T> class FakeAtomicLoader {
    T _tmp;
    bool _changed = false;

 public:
    void store(const T& data) {
        rt_base_t level = rt_hw_interrupt_disable();
        _tmp = data, _changed = true;
        rt_hw_interrupt_enable(level);
    }
    template <typename... Args> void emplace(Args&&... args) {
        rt_base_t level = rt_hw_interrupt_disable();
        _tmp = T(std::forward<Args>(args)...), _changed = true;
        rt_hw_interrupt_enable(level);
    }
    bool load(T& data) {
        rt_base_t level = rt_hw_interrupt_disable();
        bool res = _changed;
        if (_changed) data = _tmp, _changed = false;
        rt_hw_interrupt_enable(level);
        return res;
    }
};

#endif  // _FakeAtomic_hpp