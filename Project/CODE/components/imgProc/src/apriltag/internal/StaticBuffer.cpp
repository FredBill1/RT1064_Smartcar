#include "apriltag/internal/StaticBuffer.hpp"

#include "apriltag/config.hpp"
#include "common/common.h"

namespace imgProc {
namespace apriltag {

StaticBuffer::StaticBuffer(void* buf, int32_t size) : data((uint8_t*)buf), N(size) { reset(); }
void StaticBuffer::reset() { i = 0, _overflow = false; }
void* StaticBuffer::allocate(int32_t size) {
    if (i + size > N) i = 0, _overflow = true;
    void* res = data + i;
    i += size;
    return res;
}
void StaticBuffer::pop(int32_t size) {
    if (_overflow) return;
    i -= size;
}

AT_SDRAM_SECTION_ALIGN(static uint8_t buffer[STATICBUFFER_SIZE], 64);
StaticBuffer staticBuffer(buffer, sizeof(buffer));

}  // namespace apriltag
}  // namespace imgProc