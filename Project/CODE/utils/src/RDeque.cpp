#include "utils/RDeque.hpp"

#include <rtthread.h>

#define memcpy rt_memcpy

void RDeque::forward(char*& p) {
    p += data_sz_aligned;
    if (p == buf_end) p = buf;
}
void RDeque::backward(char*& p) {
    if (p == buf) p = buf_end - data_sz_aligned;
    else
        p -= data_sz_aligned;
}
RDeque::RDeque(int size, int data_size) { init(size, data_size); }
RDeque::~RDeque() { delete[] buf; }
void RDeque::init(int size, int data_size) {
    data_sz = data_size;
    data_sz_aligned = RT_ALIGN(data_size, RT_ALIGN_SIZE);
    sz = size;
    int buf_sz = data_sz_aligned * size;
    l = r = buf = (char*)rt_malloc(buf_sz);
    buf_end = buf + buf_sz;
}
void RDeque::push_back(const void* data) {
    if (cnt != 0) forward(r);
    memcpy(r, data, data_sz);
    if (cnt == sz) forward(l);
    else
        ++cnt;
}
void RDeque::pop_back() {
    if (--cnt) backward(r);
}
void* RDeque::back() { return r; }
void RDeque::push_front(const void* data) {
    if (cnt != 0) backward(l);
    memcpy(l, data, data_sz);
    if (cnt == sz) backward(r);
    else
        ++cnt;
}
void RDeque::pop_front() {
    if (--cnt) forward(l);
}
void* RDeque::front() { return l; }
bool RDeque::full() const { return cnt == sz; }
bool RDeque::empty() const { return cnt == 0; }
void RDeque::clear() {
    l = r = buf;
    cnt = 0;
}