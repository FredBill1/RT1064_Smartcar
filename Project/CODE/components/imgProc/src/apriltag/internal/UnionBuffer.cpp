#include "apriltag/internal/UnionBuffer.hpp"

#include "common/common.h"

namespace imgProc {
namespace apriltag {
AT_SDRAM_SECTION_ALIGN(UnionBuffer unionBuffer, 64);
AT_SDRAM_NONCACHE_SECTION_ALIGN(QuadImg_t threshim, 64);

}  // namespace apriltag
}  // namespace imgProc