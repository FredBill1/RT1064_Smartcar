#ifndef _Semaphore_hpp
#define _Semaphore_hpp

#include <rtthread.h>

class Semaphore {
    rt_semaphore mID;
    bool initialized = false;

 public:
    Semaphore() = default;
    // flag可以取 RT_IPC_FLAG_FIFO(先来后到, 非实时) 或 RT_IPC_FLAG_PRIO(实时)
    Semaphore(const char *name, int32_t count, rt_uint8_t flag = RT_IPC_FLAG_PRIO) { init(name, count, flag); }
    void init(const char *name, int32_t count, rt_uint8_t flag = RT_IPC_FLAG_PRIO) {
        rt_sem_init(&mID, name, count, flag);
        initialized = true;
    }
    ~Semaphore() {
        if (initialized) rt_sem_detach(&mID);
    }
    // 小于0永久等待, 等于0立即返回, 大于0等待指定时间
    bool wait(int32_t timeout_ms = -1) {
        rt_int32_t tick = (timeout_ms < 0) ? -1 : rt_tick_from_millisecond(timeout_ms);
        return rt_sem_take(&mID, tick) == RT_EOK;
    }
    void release(void) { rt_sem_release(&mID); }
};

#endif  //_Semaphore_hpp