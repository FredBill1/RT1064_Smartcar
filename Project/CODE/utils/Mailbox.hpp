#ifndef _Mailbox_hpp
#define _Mailbox_hpp

#include <rtthread.h>

class Mailbox {
    bool _initialized = false;
    rt_mailbox_t _mb;

 public:
    Mailbox() = default;
    Mailbox(Mailbox &&) = delete;
    Mailbox(const char *name, rt_size_t size, rt_uint8_t flag = RT_IPC_FLAG_FIFO) { init(name, size, flag); }
    void init(const char *name, rt_size_t size, rt_uint8_t flag = RT_IPC_FLAG_FIFO) {
        _mb = rt_mb_create(name, size, flag);
        _initialized = true;
    }
    ~Mailbox() {
        if (_initialized) rt_mb_delete(_mb);
    }
    bool put(rt_ubase_t data, rt_int32_t timeout = RT_WAITING_FOREVER) { return rt_mb_send_wait(_mb, data, timeout) == RT_EOK; }
    bool put_nowait(rt_ubase_t data) { return put(data, 0); }

    bool get(rt_ubase_t &data, rt_int32_t timeout = RT_WAITING_FOREVER) { return rt_mb_recv(_mb, &data, timeout) == RT_EOK; }
    bool get_nowait(rt_ubase_t &data) { return get(data, 0); }
    rt_ubase_t get() {
        rt_ubase_t data;
        get(data);
        return data;
    }
    void instant_put(rt_ubase_t data) {
        rt_ubase_t tmp;
        while (!put_nowait(data)) get_nowait(tmp);
    }
};

#endif  //_Mailbox_hpp