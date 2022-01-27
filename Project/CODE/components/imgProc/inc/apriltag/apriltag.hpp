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
    void *impl;

    void init(int maxhamming);
};

struct quad {
    float p[4][2];  // corners

    bool reversed_border;

    // H: tag coordinates ([-1,1] at the black corners) to pixels
    // Hinv: pixels to tag
    // matd_t *H, *Hinv;
    double H[3][3], Hinv[3][3];
};

struct pt {
    // Note: these represent 2*actual value.
    uint16_t x, y;
    int16_t gx, gy;
    float slope;
};

struct apriltag_detection {
    // a pointer for convenience. not freed by apriltag_detection_destroy.
    const apriltag_family *family;

    // The decoded ID of the tag
    int id;

    // How many error bits were corrected? Note: accepting large numbers of
    // corrected errors leads to greatly increased false positive rates.
    // NOTE: As of this implementation, the detector cannot detect tags with
    // a hamming distance greater than 2.
    int hamming;

    // A measure of the quality of the binary decoding process: the
    // average difference between the intensity of a data bit versus
    // the decision threshold. Higher numbers roughly indicate better
    // decodes. This is a reasonable measure of detection accuracy
    // only for very small tags-- not effective for larger tags (where
    // we could have sampled anywhere within a bit cell and still
    // gotten a good detection.)
    float decision_margin;

    // The 3x3 homography matrix describing the projection from an
    // "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
    // -1)) to pixels in the image. This matrix will be freed by
    // apriltag_detection_destroy.
    double H[3][3];

    // The center of the detection in image pixel coordinates.
    double c[2];

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    double p[4][2];
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_apriltag_hpp