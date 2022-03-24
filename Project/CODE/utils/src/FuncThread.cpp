#include "utils/FuncThread.hpp"

bool FuncThread(std::function<void()> func, const char* name, rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick) {
    class FuncWrapper {
        const std::function<void()> _func;
        rt_thread_t _thread;
        static void funcWrapper(void* self) { ((FuncWrapper*)self)->func(); }

     public:
        FuncWrapper(std::function<void()> func) : _func(func) {}
        bool start(const char* name, rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick) {
            _thread = rt_thread_create(name, funcWrapper, this, stack_size, priority, tick);
            if (_thread) {
                if (rt_thread_startup(_thread) == RT_EOK) return true;
                rt_thread_delete(_thread);
            }
            delete this;
            return false;
        }
        void func() {
            _func();
            rt_thread_delete(_thread);
            delete this;
        }
    };
    FuncWrapper* wrapper = new FuncWrapper(func);
    return wrapper->start(name, stack_size, priority, tick);
}

bool FuncTimer(std::function<void()> func, const char* name, rt_uint32_t period_tick) {
    class FuncWrapper {
        const std::function<void()> _func;
        rt_timer_t _timer;
        static void funcWrapper(void* self) { ((FuncWrapper*)self)->func(); }

     public:
        FuncWrapper(std::function<void()> func) : _func(func) {}
        bool start(const char* name, rt_uint32_t period_tick) {
            _timer = rt_timer_create(name, funcWrapper, this, period_tick, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
            if (_timer) {
                if (rt_timer_start(_timer) == RT_EOK) return true;
                rt_timer_delete(_timer);
            }
            delete this;
            return false;
        }
        void func() { _func(); }
    };
    FuncWrapper* wrapper = new FuncWrapper(func);
    return wrapper->start(name, period_tick);
}
