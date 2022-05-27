#ifndef _apriltag_apriltag_hpp
#define _apriltag_apriltag_hpp

#include <cstdint>
#include <forward_list>

#include "BinaryImg.hpp"
#include "apriltag/apriltag.hpp"
#include "apriltag/config.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/Unionfind.hpp"

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

    void init(int maxhamming, bool static_allocate = false);
};

struct quad {
    float p[4][2];  // corners

    bool reversed_border;

    // H: tag coordinates ([-1,1] at the black corners) to pixels
    // Hinv: pixels to tag
    // matd_t *H, *Hinv;
    float_t H[3][3];
    // float_t Hinv[3][3];
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
    float_t H[3][3];

    // The center of the detection in image pixel coordinates.
    float_t c[2];

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    float_t p[4][2];
};

struct rect {
    float_t p[4][2];
    uint32_t magnitude;
};

using QuadImg_t = imgProc::QuadImg<N / quad_decimate, M / quad_decimate>;
using Unionfind_t = Unionfind<int32_t, N * M / (quad_decimate * quad_decimate)>;

using ID_t = uint32_t;

using List_pt_alloc_t = StaticAllocator<pt>;
using List_pt_t = std::forward_list<pt, List_pt_alloc_t>;

using clusters_alloc_t = StaticAllocator<List_pt_t *>;
using clusters_t = std::forward_list<List_pt_t *, clusters_alloc_t>;

using quads_alloc_t = StaticAllocator<quad>;
using quads_t = std::forward_list<quad, quads_alloc_t>;

using detections_alloc_t = StaticAllocator<apriltag_detection *>;
using detections_t = std::forward_list<apriltag_detection *, detections_alloc_t>;

using rects_alloc_t = StaticAllocator<rect *>;
using rects_t = std::forward_list<rect *, rects_alloc_t>;

enum class apriltag_detect_visualize_flag { None, threshim, unionfind, clusters, quads, decode };

detections_t &apriltag_detect(apriltag_family &tf, uint8_t *img,
                              apriltag_detect_visualize_flag visualize_flag = apriltag_detect_visualize_flag::None);

rects_t &find_rects(uint8_t *img, float min_magnitude,
                    apriltag_detect_visualize_flag visualize_flag = apriltag_detect_visualize_flag::None);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_apriltag_hpp