#include "apriltag/internal/StaticBuffer.hpp"

#include "common/common.h"

namespace imgProc {
namespace apriltag {

AT_SDRAM_SECTION_ALIGN(UnionBuffer unionBuffer, 64);

}
}  // namespace imgProc