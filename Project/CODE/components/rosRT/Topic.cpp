#include "Topic.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace rosRT {

static uint8_t TopicManager_buf[sizeof(TopicManager)];

TopicManager &TopicManager::get() {
    static bool _initialized = false;
    if (!_initialized) {
        _initialized = true;
        new (TopicManager_buf) TopicManager();
    }
    return *reinterpret_cast<TopicManager *>(TopicManager_buf);
}

void Subscriber::_Worker(void *self) {
    Subscriber *p = (Subscriber *)self;
    for (;;) p->get(p->tmp), p->_callback(p->tmp);
}

Subscriber::Subscriber(const char *topic, size_t data_sz, size_t queue_sz, Callback_t callback, rt_uint32_t stack_size,
                       rt_uint8_t priority, rt_uint32_t tick)
    : _callback(callback),
      TopicBase(topic, data_sz, queue_sz),
      _WorkerThread(&Subscriber::_Worker, this, stack_size, priority, tick, topic) {
    TopicManager::get().Register(*this);
    _WorkerThread.start();
}
Subscriber::~Subscriber() { TopicManager::get().UnRegister(*this); }
void Publisher::_Worker(void *self) {
    Publisher *p = (Publisher *)self;
    for (;;) {
        p->get(p->tmp);
        for (Subscriber *s : p->_subs) s->put(p->tmp);
    }
}

void Publisher::addSub(Subscriber &sub) { _subs.push_back(&sub); }
void Publisher::rmSub(Subscriber &sub) { _subs.erase(std::find(_subs.begin(), _subs.end(), &sub)); }
Publisher::Publisher(const char *topic, size_t data_sz, size_t queue_sz, rt_uint32_t stack_size, rt_uint8_t priority,
                     rt_uint32_t tick)
    : TopicBase(topic, data_sz, queue_sz), _WorkerThread(&Publisher::_Worker, this, stack_size, priority, tick, topic) {
    TopicManager::get().Register(*this);
    _WorkerThread.start();
}
Publisher::~Publisher() { TopicManager::get().UnRegister(*this); }
void Publisher::publish(const void *data) { put(data); }

TopicManager::Pub_Sub_t &TopicManager::findTopic(const char *topic) {
    for (auto &[_name, p] : _topics)
        if (strcmp(_name, topic) == 0) return p;
    _topics.emplace_back(topic, Pub_Sub_t());
    return _topics.back().second;
}

void TopicManager::Register(Publisher &pub) {
    _mutex.lock();
    auto &[_pubs, _subs] = findTopic(pub._topic);
    _pubs.push_back(&pub);
    for (auto s : _subs) pub.addSub(*s);
    _mutex.unlock();
}
void TopicManager::Register(Subscriber &sub) {
    _mutex.lock();
    auto &[_pubs, _subs] = findTopic(sub._topic);
    _subs.push_back(&sub);
    for (auto p : _pubs) p->addSub(sub);
    _mutex.unlock();
}
void TopicManager::UnRegister(Publisher &pub) {
    _mutex.lock();
    auto &[_pubs, _subs] = findTopic(pub._topic);
    auto p = std::find(_pubs.begin(), _pubs.end(), &pub);
    if (p != _pubs.end()) _pubs.erase(p);
    _mutex.unlock();
}
void TopicManager::UnRegister(Subscriber &sub) {
    _mutex.lock();
    auto &[_pubs, _subs] = findTopic(sub._topic);
    auto s = std::find(_subs.begin(), _subs.end(), &sub);
    if (s != _subs.end()) {
        _subs.erase(s);
        for (auto p : _pubs) p->rmSub(sub);
    }
    _mutex.unlock();
}

};  // namespace rosRT