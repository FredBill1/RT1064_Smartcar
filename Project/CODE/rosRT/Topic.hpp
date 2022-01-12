#ifndef _rosRT_Topic_hpp
#define _rosRT_Topic_hpp

#include <functional>
#include <vector>

#include "Thread.h"
#include "utils/MQueue.hpp"
namespace rosRT {
template <typename data_t, int queue_sz> class Topic {
 private:
    MQueue<data_t, queue_sz> mq;
    std::vector<std::function<void(const data_t &)>> _CB;
    rtthread::Thread _CBThread;
    static void _CBThreadFunc(void *param) {
        Topic<data_t, queue_sz> *p = (Topic<data_t, queue_sz> *)param;
        data_t data;
        for (;;) {
            p->mq.get(data);
            for (auto &cb : p->_CB) { cb(data); }
        }
    }

 public:
    Topic(const char *name = "topic", rt_uint32_t stack_size = 2048,
          rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20)
        : _CBThread(&Topic<data_t, queue_sz>::_CBThreadFunc, this, stack_size, priority, tick, name) {
        _CBThread.start();
    }
    void subscribe(std::function<void(data_t)> cb) { _CB.push_back(cb); }
    void publish(const data_t &data) { mq.put(data); }
};
};  // namespace rosRT

#endif  // _rosRT_Topic_hpp