#include "apriltag/internal/reconcile_detections.hpp"

#include <algorithm>
#include <utility>

#include "apriltag/internal/StaticBuffer.hpp"

namespace imgProc {
namespace apriltag {

// a function that check if the two quads are intersecting

inline bool is_overlap(const apriltag_detection& det0, const apriltag_detection& det1) {
    if (det0.id != det1.id || det0.family != det1.family) return false;
    // TODO: 检查是否有重叠，但这个其实不需要检查，因为场地里没有重复出现的同id标签
    return true;
}

inline bool prefer(const apriltag_detection& det0, const apriltag_detection& det1) {
    if (det0.hamming != det1.hamming) return det0.hamming < det1.hamming;
    return det0.decision_margin > det1.decision_margin;
}

void reconcile_detections(detections_t& detections) {
    for (auto front = detections.begin(); front != detections.end(); ++front) {
        for (decltype(front) it = front, cur; it != detections.end(); ++it) {
            if (cur = it; ++cur == detections.end()) break;
            if (is_overlap(**cur, **front)) {
                if (prefer(**cur, **front)) std::swap(*cur, *front);
                detections.erase_after(it);
            }
        }
    }
}

}  // namespace apriltag
}  // namespace imgProc