#ifndef _StaticBuffer_hpp
#define _StaticBuffer_hpp

#include <cstddef>
#include <cstdint>

namespace imgProc {
namespace apriltag {

class StaticBuffer {
    uint8_t* const data;
    const int32_t N;
    int32_t i;
    bool _overflow;

 public:
    StaticBuffer(void* buf, int32_t size);
    void reset();
    void* allocate(int32_t size);
    void pop(int32_t size);
    bool overflow() const { return _overflow; }
    int32_t usage() const { return i; }
};

template <typename T> struct StaticAllocator {
    using value_type = T;
    StaticBuffer& data;
    StaticAllocator(StaticBuffer& buf) : data(buf) {}
    template <class U> constexpr StaticAllocator(const StaticAllocator<U>& other) noexcept : data(other.data) {}
    T* allocate(std::size_t n) { return (T*)data.allocate(n * sizeof(T)); }
    void deallocate(T*, std::size_t) noexcept {}
};

extern StaticBuffer staticBuffer;

}  // namespace apriltag
}  // namespace imgProc

#endif  //_StaticBuffer_hpp