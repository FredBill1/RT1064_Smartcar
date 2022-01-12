#ifndef _MQueue_hpp
#define _MQueue_hpp

// 感觉RTT官方库里的实现有点问题，所以自己写一个；大部分代码应该差不多
#include <rtthread.h>

template <typename T, int N> class MQueue {
 private:
    rt_messagequeue mID;
    char mPool[(sizeof(void*) + RT_ALIGN(sizeof(T), RT_ALIGN_SIZE)) * N];

 public:
    MQueue() { rt_mq_init(&mID, "mq", mPool, sizeof(T), sizeof(mPool), RT_IPC_FLAG_FIFO); };
    ~MQueue() { rt_mq_detach(&mID); };

    bool pushback(const T& data, int32_t millisec = 0) {
        return rt_mq_send_wait(&mID, &data, sizeof(data), millisec) != RT_EOK;
    };
    bool pushfront(const T& data) { return rt_mq_urgent(&mID, &data, sizeof(data)) != RT_EOK; };

    bool popfront(T& data, int32_t millisec = -1) {
        return rt_mq_recv(&mID, &data, sizeof(data), (millisec < 0) ? -1 : rt_tick_from_millisecond(millisec)) !=
               RT_EOK;
    };

    void put(const T& data) {
        while (pushback(data)) {
            T tmp;
            popfront(tmp, 0);
        }
    };
    void get(T& data) { popfront(data); };
};

template <typename T> class SensorDataBuf {
 private:
    MQueue<T, 1> mq;

 public:
    void put(const T& data) {
        while (mq.pushback(data)) {
            T tmp;
            mq.popfront(tmp, 0);
        }
    }
    void get(T& data) { mq.popfront(data); }
};

#endif  // _MQueue_hpp