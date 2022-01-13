#ifndef _rosRT_Topic_hpp
#define _rosRT_Topic_hpp

#include <functional>
#include <vector>

#include "Mutex.h"
#include "Thread.h"
#include "utils/MQueue.hpp"
namespace rosRT {

class TopicBase : protected MQueue {
 public:
    const char *const _topic;
    TopicBase(const char *topic, size_t data_sz, size_t queue_sz) : _topic(topic), MQueue(data_sz, queue_sz, topic) {}
    using MQueue::data_size;
};

class Subscriber : public TopicBase {
    friend class Publisher;
    using Callback_t = std::function<void(const void *)>;
    Callback_t _callback;
    rtthread::Thread _WorkerThread;
    static void _Worker(void *self);

 public:
    Subscriber(const char *topic, size_t data_sz, size_t queue_sz, Callback_t callback, rt_uint32_t stack_size = 2048,
               rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20);
    template <typename T>
    static Subscriber create(const char *topic, size_t queue_sz, std::function<void(const T &)> callback,
                             rt_uint32_t stack_size = 2048, rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3,
                             rt_uint32_t tick = 20) {
        return {topic,      sizeof(T), queue_sz, [callback](const void *data) { callback(*(const T *)data); },
                stack_size, priority,  tick};
    }
    virtual ~Subscriber();
};

class Publisher : public TopicBase {
    friend class TopicManager;
    std::vector<Subscriber *> _subs;
    rtthread::Thread _WorkerThread;
    static void _Worker(void *self);

 protected:
    void addSub(Subscriber &sub);
    void rmSub(Subscriber &sub);

 public:
    Publisher(const char *topic, size_t data_sz, size_t queue_sz, rt_uint32_t stack_size = 2048,
              rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20);
    template <typename T>
    static Publisher create(const char *topic, size_t queue_sz, rt_uint32_t stack_size = 2048,
                            rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20) {
        return {topic, sizeof(T), queue_sz, stack_size, priority, tick};
    }
    virtual ~Publisher();
    void publish(const void *data);
};

class TopicManager {
 private:
    friend class Publisher;
    friend class Subscriber;
    using Pub_Sub_t = std::pair<std::vector<Publisher *>, std::vector<Subscriber *>>;
    std::vector<std::pair<const char *, Pub_Sub_t>> _topics;
    rtthread::Mutex _mutex;
    TopicManager() {}
    Pub_Sub_t &findTopic(const char *topic);

 protected:
    void Register(Publisher &pub);
    void Register(Subscriber &sub);
    void UnRegister(Publisher &pub);
    void UnRegister(Subscriber &sub);

 public:
    static TopicManager &get();
};
};  // namespace rosRT

#endif  // _rosRT_Topic_hpp