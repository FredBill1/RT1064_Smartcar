#include "apriltag/internal/globalVariables.hpp"

#include "common/common.h"

namespace imgProc {
namespace apriltag {
AT_SDRAM_SECTION_ALIGN(Unionfind_t uf, 64);
AT_SDRAM_SECTION_ALIGN(QuadImg_t threshim, 64);
AT_DTCM_SECTION_ALIGN(uint8_t hashmapbuf[sizeof(Hashmap)], 64);
AT_DTCM_SECTION_ALIGN(threshold_buf thresholdbuf, 64);

}  // namespace apriltag
}  // namespace imgProc