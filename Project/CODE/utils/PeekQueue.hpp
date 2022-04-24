#ifndef _utils_PeekQueue_hpp
#define _utils_PeekQueue_hpp

#include "utils/RDeque.hpp"

class PeekQueue {
    RDeque Q;
    char* buf = nullptr;

 public:
    PeekQueue() = default;
    PeekQueue(int size, int data_size);
    ~PeekQueue();
    void init(int size, int data_size);
    bool get();
    void push(void* data);
    void* front();
    template <typename T> T& front() { return *(T*)front(); }
};

#endif  // _utils_PeekQueue_hpp