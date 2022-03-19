#include "apriltag/apriltag.hpp"

#include "apriltag/internal/Quick_decode.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/decode_quad.hpp"
#include "apriltag/internal/fit_quad.hpp"
#include "apriltag/internal/globalVariables.hpp"
#include "apriltag/internal/reconcile_detections.hpp"
#include "apriltag/internal/segmentation.hpp"
#include "apriltag/internal/threshold.hpp"
#include "apriltag/visualization.hpp"
extern "C" {
#include "common.h"
}

#define apriltag_benchmark 0

#if (apriltag_benchmark)
#include <rtthread.h>
#endif

namespace imgProc {
namespace apriltag {

void apriltag_family::init(int maxhamming, bool static_allocate) { quick_decode::init(*this, maxhamming, static_allocate); }

detections_t &apriltag_detect(apriltag_family &tf, uint8_t *img, apriltag_detect_visualize_flag visualize_flag) {
    staticBuffer.reset();
#if (apriltag_benchmark)
    int32_t t0 = rt_tick_get();
#endif
    // show_grayscale(img);
    threshold(img, threshim);
    if (visualize_flag == apriltag_detect_visualize_flag::threshim) show_threshim(threshim);

#if (apriltag_benchmark)
    int32_t t1 = rt_tick_get();
#endif

    unionfind_connected(threshim);
    if (visualize_flag == apriltag_detect_visualize_flag::unionfind) show_unionfind();

#if (apriltag_benchmark)
    int32_t t2 = rt_tick_get();
#endif

    auto &clusters = *gradient_clusters(threshim);
    if (visualize_flag == apriltag_detect_visualize_flag::clusters) show_clustersImg(img, clusters);

#if (apriltag_benchmark)
    int32_t t3 = rt_tick_get();
#endif

    auto &quads = *fit_quads(
        clusters, tf, img,
        visualize_flag == apriltag_detect_visualize_flag::quads || visualize_flag == apriltag_detect_visualize_flag::decode);
    if (visualize_flag == apriltag_detect_visualize_flag::quads || visualize_flag == apriltag_detect_visualize_flag::decode) {
        show_clustersImg(img, clusters);
        show_quadsImg(img, quads);
    }

#if (apriltag_benchmark)
    int32_t t4 = rt_tick_get();
#endif
    auto &detections = *decode_quads(tf, img, quads, visualize_flag == apriltag_detect_visualize_flag::decode);
    reconcile_detections(detections);

#if (apriltag_benchmark)
    int32_t t5 = rt_tick_get();
    rt_kprintf("%d %d %d %d %d\r\n", t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4);
#endif

    // 按检测的可信度降序排序
    // detections.sort([](const apriltag_detection *a, const apriltag_detection *b) {
    //     if (a->hamming != b->hamming) return a->hamming < b->hamming;
    //     return a->decision_margin > b->decision_margin;
    // });

    return detections;
}

}  // namespace apriltag
}  // namespace imgProc