#ifndef _imgProc_apriltag_internal_g2d_hpp
#define _imgProc_apriltag_internal_g2d_hpp

#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

bool line_intersection(const float_t p1[2], const float_t p2[2], const float_t p3[2], const float_t p4[2], float_t res[2]);

bool point_in_line_segment(const float_t p1[2], const float_t p2[2], const float_t p[2]);

bool line_segment_intersection(const float_t p1[2], const float_t p2[2], const float_t p3[2], const float_t p4[2],
                               float_t res[2]);

bool polygon_contains_point(const float_t poly[][2], int N, const float_t q[2]);

float_t vector_cos(float_t x1, float_t y1, float_t x2, float_t y2);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _imgProc_apriltag_internal_g2d_hpp