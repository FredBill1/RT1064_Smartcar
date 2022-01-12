#ifndef _rosRT_Topic_hpp
#define _rosRT_Topic_hpp

#include <functional>
#include <vector>

#include "Thread.h"
#include "utils/MQueue.hpp"
namespace rosRT {

class Topic : private MQueue {
 public:
    using Callback_t = std::function<void(const void *)>;
    Topic(size_t data_sz, size_t data_cnt, rt_uint32_t stack_size = 2048,
          rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20, const char *name = "topic")
        : MQueue(data_sz, data_cnt, name), _CBThread(&Topic::_CBThreadFunc, this, stack_size, priority, tick, name) {
        _CBThread.start();
    }
    void subscribe(Callback_t cb) { _CB.push_back(cb); }
    void publish(const void *data) { put(data); }
    using MQueue::data_size;

 private:
    Topic() = delete;
    std::vector<Callback_t> _CB;
    rtthread::Thread _CBThread;
    static void _CBThreadFunc(void *self) {
        Topic *p = (Topic *)self;
        for (;;) {
            p->get(p->tmp);
            for (auto &cb : p->_CB) { cb(p->tmp); }
        }
    }
};

};  // namespace rosRT

#endif  // _rosRT_Topic_hpp