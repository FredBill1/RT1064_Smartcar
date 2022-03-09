#include "apriltag/internal/fit_quad.hpp"

#include <rtthread.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "apriltag/config.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/utility.hpp"
#include "apriltag/visualization.hpp"

namespace imgProc {
namespace apriltag {

struct line_fit_pt {
    float_t Mx, My;
    float_t Mxx, Myy, Mxy;
    float_t W;  // total weight
};

inline line_fit_pt* compute_lfps(int_fast32_t sz, const List_pt_t& cluster, const uint8_t* im) {
    line_fit_pt* lfps = (line_fit_pt*)staticBuffer.allocate(sizeof(line_fit_pt) * sz);
    int_fast32_t i = -1;
    for (const auto& p : cluster) {
        if (++i > 0) rt_memcpy(lfps + i, lfps + (i - 1), sizeof(line_fit_pt));
        else
            rt_memset(lfps, 0, sizeof(line_fit_pt));
        float_t delta = 0.5, x = p.x * .5 + delta, y = p.y * .5 + delta, W = 1;
        int_fast32_t ix = x, iy = y;
        if (ix > 0 && ix + 1 < M / quad_decimate && iy > 0 && iy + 1 < N / quad_decimate) {
            int_fast32_t grad_x = (int_fast32_t)im[(iy * M + ix + 1) * quad_decimate] - im[(iy * M + ix - 1) * quad_decimate],
                         grad_y = (int_fast32_t)im[((iy + 1) * M + ix) * quad_decimate] - im[((iy - 1) * M + ix) * quad_decimate];
            W = sqrtf(grad_x * grad_x + grad_y * grad_y) + 1;
        }
        float_t fx = x, fy = y;
        lfps[i].Mx += W * fx;
        lfps[i].My += W * fy;
        lfps[i].Mxx += W * fx * fx;
        lfps[i].Mxy += W * fx * fy;
        lfps[i].Myy += W * fy * fy;
        lfps[i].W += W;
    }
    return lfps;
}

static void fit_line(line_fit_pt* lfps, int_fast32_t sz, int_fast32_t i0, int_fast32_t i1, float_t* lineparm, float_t* err,
                     float_t* mse) {
    float_t Mx, My, Mxx, Myy, Mxy, W;
    int_fast32_t N;
    if (i0 < i1) {
        N = i1 - i0 + 1, Mx = lfps[i1].Mx, My = lfps[i1].My, Mxx = lfps[i1].Mxx, Mxy = lfps[i1].Mxy, Myy = lfps[i1].Myy,
        W = lfps[i1].W;
        if (i0 > 0)
            Mx -= lfps[i0 - 1].Mx, My -= lfps[i0 - 1].My, Mxx -= lfps[i0 - 1].Mxx, Mxy -= lfps[i0 - 1].Mxy,
                Myy -= lfps[i0 - 1].Myy, W -= lfps[i0 - 1].W;
    } else {
        Mx = lfps[sz - 1].Mx - lfps[i0 - 1].Mx, My = lfps[sz - 1].My - lfps[i0 - 1].My, Mxx = lfps[sz - 1].Mxx - lfps[i0 - 1].Mxx,
        Mxy = lfps[sz - 1].Mxy - lfps[i0 - 1].Mxy, Myy = lfps[sz - 1].Myy - lfps[i0 - 1].Myy, W = lfps[sz - 1].W - lfps[i0 - 1].W;
        Mx += lfps[i1].Mx, My += lfps[i1].My, Mxx += lfps[i1].Mxx, Mxy += lfps[i1].Mxy, Myy += lfps[i1].Myy, W += lfps[i1].W;
        N = sz - i0 + i1 + 1;
    }
    float_t Ex = Mx / W, Ey = My / W, Cxx = Mxx / W - Ex * Ex, Cxy = Mxy / W - Ex * Ey, Cyy = Myy / W - Ey * Ey,
            eig_small = 0.5 * (Cxx + Cyy - sqrtf((Cxx - Cyy) * (Cxx - Cyy) + 4 * Cxy * Cxy));
    if (lineparm) {
        lineparm[0] = Ex;
        lineparm[1] = Ey;
        float_t eig = 0.5 * (Cxx + Cyy + sqrtf((Cxx - Cyy) * (Cxx - Cyy) + 4 * Cxy * Cxy)), nx1 = Cxx - eig, ny1 = Cxy,
                M1 = nx1 * nx1 + ny1 * ny1, nx2 = Cxy, ny2 = Cyy - eig, M2 = nx2 * nx2 + ny2 * ny2, nx, ny, M;
        if (M1 > M2) nx = nx1, ny = ny1, M = M1;
        else
            nx = nx2, ny = ny2, M = M2;
        float_t length = sqrtf(M);
        lineparm[2] = nx / length, lineparm[3] = ny / length;
    }
    if (err) *err = N * eig_small;
    if (mse) *mse = eig_small;
}

inline bool quad_segment_maxima(int_fast32_t sz, List_pt_t& cluster, line_fit_pt* lfps, int indices[4]) {
    int_fast32_t ksz = min(20, sz / 12);
    if (ksz < 2) return false;

    int_fast32_t* maxima = (int_fast32_t*)staticBuffer.allocate(sizeof(int_fast32_t) * sz);
    float_t* maxima_errs = (float_t*)staticBuffer.allocate(sizeof(float_t) * sz);
    int_fast32_t nmaxima = 0;

    float_t* errs = (float_t*)staticBuffer.allocate(sizeof(float_t) * sz);
    rep(i, 0, sz) fit_line(lfps, sz, (i + sz - ksz) % sz, (i + ksz) % sz, NULL, &errs[i], NULL);
    {
        float_t* y = (float_t*)staticBuffer.allocate(sizeof(float_t) * sz);
        // constexpr int_fast32_t fsz = 17;  // cutoff = 0.05, sigma = 3
        // constexpr float f[fsz]{0.02856549993F, 0.06572853029F, 0.1353352815F, 0.2493522018F,  0.4111122787F, 0.6065306664F,
        //                        0.800737381F,   0.9459594488F,  1.0F,          0.9459594488F,  0.800737381F,  0.6065306664F,
        //                        0.4111122787F,  0.2493522018F,  0.1353352815F, 0.06572853029F, 0.02856549993F};
        constexpr int_fast32_t fsz = 7;  // cutoff = 0.05, sigma = 1
        constexpr float f[fsz]{0.01110899635F, 0.1353352815F, 0.6065306664F, 1.0F, 0.6065306664F, 0.1353352815F, 0.01110899635F};
        rep(iy, 0, sz) {
            float_t acc = 0;
#if (0)
            rep(i, 0, fsz) acc += errs[(iy + i - fsz / 2 + sz) % sz] * f[i];
#else
            int index = (iy - fsz / 2 + sz) % sz;
            rep(i, 0, fsz) {
                acc += errs[index] * f[i];
                if (++index >= sz) index = 0;
            }
#endif
            y[iy] = acc;
        }
        rt_memcpy(errs, y, sizeof(float_t) * sz);
        staticBuffer.pop(sizeof(float_t) * sz);  // float_t y[sz];
    }

    rep(i, 0, sz) if (errs[i] > errs[(i + 1) % sz] && errs[i] > errs[(i + sz - 1) % sz]) {
        maxima[nmaxima] = i;
        maxima_errs[nmaxima] = errs[i];
        ++nmaxima;
    }
    staticBuffer.pop(sizeof(float_t) * sz);  // float_t errs[sz];
    if (nmaxima < 4) {
        staticBuffer.pop(sizeof(float_t) * sz);       // float_t maxima_errs[sz];
        staticBuffer.pop(sizeof(int_fast32_t) * sz);  // int_fast32_t maxima[sz]
        return false;
    }

    if (nmaxima > max_nmaxima) {
        float_t* maxima_errs_copy = (float_t*)staticBuffer.allocate(sizeof(float_t) * (max_nmaxima + 1));
        std::partial_sort_copy(maxima_errs, maxima_errs + nmaxima, maxima_errs_copy, maxima_errs_copy + (max_nmaxima + 1),
                               [](float_t a, float_t b) { return a > b; });
        float_t maxima_thresh = maxima_errs_copy[max_nmaxima];
        staticBuffer.pop(sizeof(float_t) * (max_nmaxima + 1));  // float_t maxima_errs_copy[max_nmaxima + 1];
        int_fast32_t out = 0;
        rep(in, 0, nmaxima) if (maxima_errs[in] > maxima_thresh) maxima[out++] = maxima[in];
        nmaxima = out;
    }
    staticBuffer.pop(sizeof(float_t) * sz);  // float_t maxima_errs[sz];

    int best_indices[4];
    float_t best_error = HUGE_VALF;
    float_t err01, err12, err23, err30, mse01, mse12, mse23, mse30, params01[4], params12[4], params23[4], params30[4];
    constexpr float_t max_dot = cos_critical_rad;
    for (int m0 = 0; m0 < nmaxima - 3; m0++) {
        int i0 = maxima[m0];
        for (int m1 = m0 + 1; m1 < nmaxima - 2; m1++) {
            int i1 = maxima[m1];
            fit_line(lfps, sz, i0, i1, params01, &err01, &mse01);
            if (mse01 > max_line_fit_mse) continue;
            for (int m2 = m1 + 1; m2 < nmaxima - 1; m2++) {
                int i2 = maxima[m2];
                fit_line(lfps, sz, i1, i2, params12, &err12, &mse12);
                if (mse12 > max_line_fit_mse) continue;
                float_t dot = params01[2] * params12[2] + params01[3] * params12[3];
                if (fabs(dot) > max_dot) continue;
                for (int m3 = m2 + 1; m3 < nmaxima; m3++) {
                    int i3 = maxima[m3];
                    fit_line(lfps, sz, i2, i3, params23, &err23, &mse23);
                    if (mse23 > max_line_fit_mse) continue;
                    fit_line(lfps, sz, i3, i0, params30, &err30, &mse30);
                    if (mse30 > max_line_fit_mse) continue;
                    float_t err = err01 + err12 + err23 + err30;
                    if (err < best_error)
                        best_error = err, best_indices[0] = i0, best_indices[1] = i1, best_indices[2] = i2, best_indices[3] = i3;
                }
            }
        }
    }
    staticBuffer.pop(sizeof(int_fast32_t) * sz);  // int_fast32_t maxima[sz]
    if (best_error == HUGE_VALF) return false;
    if (best_error / sz > max_line_fit_mse) return false;
    rt_memcpy(indices, best_indices, sizeof(int_fast32_t) * 4);
    return true;
}

bool fit_quad(List_pt_t& cluster, apriltag_family& tf, quad& quad, uint8_t* im) {
    int_fast32_t sz = std::distance(cluster.begin(), cluster.end());
    if (sz < 24) return false;
    int_fast32_t xmax = 0, xmin = std::numeric_limits<int_fast32_t>::max(), ymax = 0, ymin = xmin;
    for (auto& p : cluster) chkmax(xmax, p.x), chkmin(xmin, p.x), chkmax(ymax, p.y), chkmin(ymin, p.y);
    if ((xmax - xmin) * (ymax - ymin) < max(tf.width_at_border / quad_decimate, 3)) return false;
    float cx = (xmin + xmax) * 0.5 + 0.05118, cy = (ymin + ymax) * 0.5 + -0.028581;
    float dot = 0;
    for (auto& p : cluster) {
        constexpr float quadrants[2][2] = {{-1 * (2 << 15), 0}, {2 * (2 << 15), 2 << 15}};
        float dx = p.x - cx, dy = p.y - cy;
        dot += dx * p.gx + dy * p.gy;
        float quadrant = quadrants[dy > 0][dx > 0];
        if (dy < 0) dy = -dy, dx = -dx;
        if (dx < 0) {
            float tmp = dx;
            dx = dy, dy = -tmp;
        }
        p.slope = quadrant + dy / dx;
    }

    quad.reversed_border = dot < 0;
    if (!tf.reversed_border && quad.reversed_border) return false;

    cluster.sort([](const pt& a, const pt& b) { return a.slope < b.slope; });
    cluster.unique([](const pt& a, const pt& b) { return a.x == b.x && a.y == b.y; });
    sz = std::distance(cluster.begin(), cluster.end());
    if (sz < 24) return false;
    line_fit_pt* lfps = compute_lfps(sz, cluster, im);

    int indices[4];
    if (!quad_segment_maxima(sz, cluster, lfps, indices)) return false;

    float_t lines[4][4];
    bool res = false;
    rep(i, 0, 4) {
        int_fast32_t i0 = indices[i], i1 = indices[(i + 1) & 3];
        float_t err;
        fit_line(lfps, sz, i0, i1, lines[i], NULL, &err);
        if (err > max_line_fit_mse) {
            res = false;
            goto finish;
        }
    }

    rep(i, 0, 4) {
        float_t A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3], A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2],
                B0 = -lines[i][0] + lines[(i + 1) & 3][0], B1 = -lines[i][1] + lines[(i + 1) & 3][1], det = A00 * A11 - A10 * A01,
                W00 = A11 / det, W01 = -A01 / det;
        if (fabs(det) < 0.001) {
            res = false;
            goto finish;
        }
        float_t L0 = W00 * B0 + W01 * B1;
        quad.p[i][0] = lines[i][0] + L0 * A00, quad.p[i][1] = lines[i][1] + L0 * A10;
        res = true;
    }

    {  // reject quads that are too small
        float_t area = 0;
        float_t length[3], p;
        for (int i = 0; i < 3; i++) {
            int idxa = i;            // 0, 1, 2,
            int idxb = (i + 1) % 3;  // 1, 2, 0
            length[i] = sqrtf(sq(quad.p[idxb][0] - quad.p[idxa][0]) + sq(quad.p[idxb][1] - quad.p[idxa][1]));
        }
        p = (length[0] + length[1] + length[2]) / 2;
        area += sqrtf(p * (p - length[0]) * (p - length[1]) * (p - length[2]));
        for (int i = 0; i < 3; i++) {
            int idxs[] = {2, 3, 0, 2};
            int idxa = idxs[i];
            int idxb = idxs[i + 1];
            length[i] = sqrtf(sq(quad.p[idxb][0] - quad.p[idxa][0]) + sq(quad.p[idxb][1] - quad.p[idxa][1]));
        }
        p = (length[0] + length[1] + length[2]) / 2;
        area += sqrtf(p * (p - length[0]) * (p - length[1]) * (p - length[2]));
        if (area < 0.95 * sq(max(tf.width_at_border / quad_decimate, 3))) {
            res = false;
            goto finish;
        }
    }
    {  // reject quads whose cumulative angle change isn't equal to 2PI
        for (int i = 0; i < 4; i++) {
            int i0 = i, i1 = (i + 1) & 3, i2 = (i + 2) & 3;
            float_t dx1 = quad.p[i1][0] - quad.p[i0][0], dy1 = quad.p[i1][1] - quad.p[i0][1], dx2 = quad.p[i2][0] - quad.p[i1][0],
                    dy2 = quad.p[i2][1] - quad.p[i1][1];
            float_t cos_dtheta = (dx1 * dx2 + dy1 * dy2) / sqrtf((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
            if ((cos_dtheta > cos_critical_rad || cos_dtheta < -cos_critical_rad) || dx1 * dy2 < dy1 * dx2) {
                res = false;
                goto finish;
            }
        }
    }

finish:
    staticBuffer.pop(sizeof(line_fit_pt) * sz);  // line_fit_pt lfps[sz];
    return res;
}

quads_t* fit_quads(clusters_t& clusters, apriltag_family& tf, uint8_t* im, bool clear) {
    quads_t* quads = new (staticBuffer.allocate(sizeof(quads_t))) quads_t(quads_alloc_t{staticBuffer});
    quad quad;
    for (auto& cluster : clusters) {
        if (fit_quad(*cluster, tf, quad, im)) {
            if (quad_decimate > 1) rep(i, 0, 4) rep(j, 0, 2) quad.p[i][j] = (quad.p[i][j] - 0.5) * quad_decimate + 0.5;
            quads->push_front(quad);

        } else if (clear)
            cluster->clear();
    }
    return quads;
}

}  // namespace apriltag
}  // namespace imgProc