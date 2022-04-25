#include "utils/PeekQueue.hpp"

#include <rthw.h>
#include <rtthread.h>

PeekQueue::PeekQueue(int size, int data_size) { init(size, data_size); }
void PeekQueue::init(int size, int data_size) {
    Q.init(size, data_size);
    buf = (char*)rt_malloc(RT_ALIGN(data_size, RT_ALIGN_SIZE));
}
PeekQueue::~PeekQueue() { rt_free(buf); }
bool PeekQueue::peek() {
    if (valid) return true;
    rt_base_t level = rt_hw_interrupt_disable();
    if (Q.empty()) {
        rt_hw_interrupt_enable(level);
        return false;
    }
    rt_memcpy(buf, Q.front(), Q.data_size());
    Q.pop_front();
    rt_hw_interrupt_enable(level);
    return valid = true;
}
void PeekQueue::pop() { valid = false; }
void PeekQueue::push(const void* data) {
    rt_base_t level = rt_hw_interrupt_disable();
    Q.push_back(data);
    rt_hw_interrupt_enable(level);
}
void PeekQueue::clear() {
    rt_base_t level = rt_hw_interrupt_disable();
    Q.clear();
    rt_hw_interrupt_enable(level);
}
void* PeekQueue::front() { return buf; }