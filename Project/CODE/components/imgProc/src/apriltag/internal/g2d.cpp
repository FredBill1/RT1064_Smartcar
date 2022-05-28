#include "apriltag/internal/g2d.hpp"

#include <algorithm>

#include "apriltag/fmath.hpp"

namespace imgProc {
namespace apriltag {

// https://en.wikipedia.org/wiki/Line¨Cline_intersection
// p1.....p4
// ..\.../..
// ...res...
// ../...\..
// p3.....p2
bool line_intersection(const float_t p1[2], const float_t p2[2], const float_t p3[2], const float_t p4[2], float_t res[2]) {
    float_t x1x2 = p1[0] - p2[0], x3x4 = p3[0] - p4[0], y1y2 = p1[1] - p2[1], y3y4 = p3[1] - p4[1];
    float_t D = (x1x2) * (y3y4) - (y1y2) * (x3x4);
    if (fabs(D) < 1e-8f) return false;
    float_t x1y2y1x2 = p1[0] * p2[1] - p1[1] * p2[0], x3y4y3x4 = p3[0] * p4[1] - p3[1] * p4[0];
    res[0] = (x1y2y1x2 * x3x4 - x1x2 * x3y4y3x4) / D;
    res[1] = (x1y2y1x2 * y3y4 - y1y2 * x3y4y3x4) / D;
    return true;
}

bool point_in_line_segment(const float_t p1[2], const float_t p2[2], const float_t p[2]) {
    auto [xl, xh] = std::minmax(p1[0], p2[0]);
    auto [yl, yh] = std::minmax(p1[1], p2[1]);
    return xl <= p[0] && p[0] <= xh && yl <= p[1] && p[1] <= yh;
}

bool line_segment_intersection(const float_t p1[2], const float_t p2[2], const float_t p3[2], const float_t p4[2],
                               float_t res[2]) {
    return line_intersection(p1, p2, p3, p4, res) && point_in_line_segment(p1, p2, res) && point_in_line_segment(p3, p4, res);
}

constexpr int polygon_contains_point_quadrant(const float_t p[2], const float_t q[2]) {
    // p[0] < q[0]       p[1] < q[1]    quadrant
    //     0                 0              0
    //     0                 1              3
    //     1                 0              1
    //     1                 1              2
    if (p[0] < q[0]) return (p[1] < q[1]) ? 2 : 1;
    else
        return (p[1] < q[1]) ? 3 : 0;
}

// https://github.com/AprilRobotics/apriltag/blob/60071c0ed2d3a0c3685854e849ee75475f50fd83/common/g2d.c#L323
bool polygon_contains_point(const float_t poly[][2], int N, const float_t q[2]) {
    // use winding. If the point is inside the polygon, we'll wrap
    // around it (accumulating 6.28 radians). If we're outside the
    // polygon, we'll accumulate zero.
    int last_quadrant = polygon_contains_point_quadrant(poly[N - 1], q);
    int quad_acc = 0;
    for (int i = 0; i < N; ++i) {
        const float_t* const p = poly[i];
        int quadrant = polygon_contains_point_quadrant(p, q);
        int dquadrant = quadrant - last_quadrant;
        // encourage a jump table by mapping to small positive integers.
        switch (dquadrant) {
        case -3:
        case 1: quad_acc++; break;
        case -1:
        case 3: quad_acc--; break;
        case 0: break;
        case -2:
        case 2: {
            // get the previous point.
            const float_t* const p0 = poly[(i + N - 1) % N];

            // Consider the points p0 and p (the points around the
            // polygon that we are tracing) and the query point q.
            //
            // If we've moved diagonally across quadrants, we want
            // to measure whether we have rotated +PI radians or
            // -PI radians. We can test this by computing the dot
            // product of vector (p0-q) with the vector
            // perpendicular to vector (p-q)
            float_t nx = p[1] - q[1];
            float_t ny = -p[0] + q[0];

            float_t dot = nx * (p0[0] - q[0]) + ny * (p0[1] - q[1]);
            if (dot < 0) quad_acc -= 2;
            else
                quad_acc += 2;
            break;
        }
        }
        last_quadrant = quadrant;
    }
    return (quad_acc >= 2) || (quad_acc <= -2);
}

float_t vector_cos(float_t x1, float_t y1, float_t x2, float_t y2) {
    return (x1 * x2 + y1 * y2) / (sqrtf(x1 * x1 + y1 * y1) * sqrtf(x2 * x2 + y2 * y2));
}

}  // namespace apriltag
}  // namespace imgProc