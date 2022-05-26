#include "show_image.hpp"

#include <cstdint>
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

namespace show_image {

void koishi() {
    static constexpr uint32_t data[]{
#include "koishi.txt"
    };
    ips114_set_region(0, 0, IPS114_X_MAX - 1, IPS114_Y_MAX - 1);
    for (const uint16_t *p = (const uint16_t *)data, *e = (const uint16_t *)data + sizeof(data) / 2; p != e; ++p)
        ips114_writedata_16bit(*p);
}

}  // namespace show_image