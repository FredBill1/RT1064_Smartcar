#include "apriltag/apriltag.hpp"

#include "apriltag/internal/Quick_decode.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/decode_quad.hpp"
#include "apriltag/internal/fit_quad.hpp"
#include "apriltag/internal/reconcile_detections.hpp"
#include "apriltag/internal/segmentation.hpp"
#include "apriltag/internal/threshold.hpp"
#include "apriltag/visualization.hpp"

extern "C" {
#include "common.h"
}

namespace imgProc {
namespace apriltag {

void apriltag_family::init(int maxhamming, bool static_allocate) { quick_decode::init(*this, maxhamming, static_allocate); }

detections_t &apriltag_detect(apriltag_family &tf, uint8_t *img) {
    AT_SDRAM_SECTION_ALIGN(static QuadImg_t threshim, 64);

    staticBuffer.reset();

    // show_grayscale(img);
    threshold(img, threshim);
    // show_threshim(threshim);

    unionfind_connected(threshim);
    // show_unionfind();

    auto &clusters = *gradient_clusters(threshim);
    // show_clusters(clusters);

    auto &quads = *fit_quads(clusters, tf, img, true);
    // show_clusters(clusters);
    // show_quads(quads);

    auto &detections = *decode_quads(tf, img, quads);
    reconcile_detections(detections);

    return detections;
}

}  // namespace apriltag
}  // namespace imgProc