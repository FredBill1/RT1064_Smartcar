#ifndef _apriltag_apriltag_hpp
#define _apriltag_apriltag_hpp

#include <cstdint>

namespace imgProc {
namespace apriltag {

struct apriltag_family {
    // How many codes are there in this tag family?
    const uint32_t ncodes;

    // The codes in the family.
    const uint64_t *const codes;

    const int width_at_border;
    const int total_width;
    const bool reversed_border;

    // The bit locations.
    const uint32_t nbits;
    const uint32_t *const bit_x;
    const uint32_t *const bit_y;

    // minimum hamming distance between any two codes. (e.g. 36h11 => 11)
    const uint32_t h;

    // a human-readable name, e.g., "tag36h11"
    const char *const name;

    // some detector implementations may preprocess codes in order to
    // accelerate decoding.  They put their data here. (Do not use the
    // same apriltag_family instance in more than one implementation)
    // void *impl;
};

struct quad {
    float p[4][2];  // corners

    bool reversed_border;

    // H: tag coordinates ([-1,1] at the black corners) to pixels
    // Hinv: pixels to tag
    // matd_t *H, *Hinv;
};

struct pt {
    // Note: these represent 2*actual value.
    uint16_t x, y;
    int16_t gx, gy;
    float slope;
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_apriltag_hpp