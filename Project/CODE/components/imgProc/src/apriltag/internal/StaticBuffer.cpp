#include "apriltag/internal/StaticBuffer.hpp"

#include "apriltag/config.hpp"
#include "common/common.h"

namespace imgProc {
namespace apriltag {

constexpr size_t alignment = 16;
template <typename T> static inline constexpr T get_aligned_size(T size) { return (size + alignment - 1) & ~(alignment - 1); }

StaticBuffer::StaticBuffer(void* buf, int32_t size) : data((uint8_t*)buf), N(size) { reset(); }
void StaticBuffer::reset() { i = 0, _overflow = false; }
void* StaticBuffer::allocate(int32_t size) {
    int32_t aligned_size = get_aligned_size(size);
    if (i + aligned_size > N) i = 0, _overflow = true;
    void* res = data + i;
    i += aligned_size;
    return res;
}
void StaticBuffer::pop(int32_t size) {
    if (_overflow) return;
    size = get_aligned_size(size);
    i -= size;
}

AT_SDRAM_SECTION_ALIGN(static uint8_t buffer[STATICBUFFER_SIZE], 64);
StaticBuffer staticBuffer(buffer, sizeof(buffer));

}  // namespace apriltag
}  // namespace imgProc