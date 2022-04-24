#ifndef _utils_RDeque_hpp
#define _utils_RDeque_hpp

class RDeque {
    int cnt = 0;
    int sz = 0, data_sz = 0, data_sz_aligned = 0;
    char *buf = nullptr, *buf_end = nullptr, *l = nullptr, *r = nullptr;
    RDeque(RDeque const&) = delete;
    void forward(char*& p);
    void backward(char*& p);

 public:
    RDeque() = default;
    RDeque(int size, int data_size);
    ~RDeque();
    void init(int size, int data_size);
    void push_back(const void* data);
    template <typename T> void push_back(const T& data) { push_back(&data); }
    void pop_back();
    void* back();
    template <typename T> T& back() { return *(T*)back(); }
    void push_front(const void* data);
    template <typename T> void push_front(const T& data) { push_front(&data); }
    void pop_front();
    void* front();
    template <typename T> T& front() { return *(T*)front(); }
    bool empty() const;
    bool full() const;
    void clear();
    int size() const { return cnt; }
};

#endif  // _utils_RDeque_hpp
