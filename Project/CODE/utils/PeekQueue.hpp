#ifndef _utils_PeekQueue_hpp
#define _utils_PeekQueue_hpp

#include "utils/RDeque.hpp"

class PeekQueue {
    RDeque Q;
    char* buf = nullptr;
    bool valid = false;

 public:
    PeekQueue() = default;
    PeekQueue(int size, int data_size);
    ~PeekQueue();
    void init(int size, int data_size);
    bool peek();
    void push(const void* data);
    void clear();
    void pop();
    void* front();
};

#endif  // _utils_PeekQueue_hpp