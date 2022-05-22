#include "edge_detect/show_edge.hpp"

#include "apriltag/config.hpp"
#include "apriltag/internal/utility.hpp"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

namespace imgProc {
namespace edge_detect {

using namespace imgProc::apriltag;

static inline bool have_edge(uint8_t* img, int i, int j) {
    rep(x, j, j + 4) rep(y, i, i + 4) if (img[y * M + x] == 255) return true;
    return false;
}

static constexpr uint16_t get_color(uint8_t m) {
    switch (m) {
    case 2: return RED;
    case 3: return GREEN;
    case 4: return BLUE;
    default: return 0;
    }
}

void show_edge(uint8_t* img) {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    rep(i, 0, N / 4) rep(j, 0, M / 4) {
        uint16_t color = get_color(img[(i * M + j) * 4]);
        if (color) ips114_writedata_16bit(color);
        else
            ips114_writedata_16bit(have_edge(img, i * 4, j * 4) ? 0x0000 : 0xffff);
    }
}

}  // namespace edge_detect
}  // namespace imgProc