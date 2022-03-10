#include "apriltag/internal/decode_quad.hpp"

#include <rtthread.h>

#include "Eigen/Eigen"
#include "apriltag/config.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/Graymodel.hpp"
#include "apriltag/internal/Quick_decode.hpp"
#include "apriltag/internal/homography.hpp"
#include "apriltag/internal/utility.hpp"
#include "apriltag/visualization.hpp"
namespace imgProc {
namespace apriltag {

inline void refine_edges(uint8_t *im_orig, quad *quad) {
    float_t lines[4][4];
    for (int edge = 0; edge < 4; edge++) {
        int a = edge, b = (edge + 1) & 3;
        float_t nx = quad->p[b][1] - quad->p[a][1], ny = -quad->p[b][0] + quad->p[a][0], mag = sqrtf(nx * nx + ny * ny);
        nx /= mag, ny /= mag;
        if (quad->reversed_border) nx = -nx, ny = -ny;
        int nsamples = max(16, mag / 8);
        float_t Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;
        for (int s = 0; s < nsamples; s++) {
            float_t alpha = (1.0 + s) / (nsamples + 1), x0 = alpha * quad->p[a][0] + (1 - alpha) * quad->p[b][0],
                    y0 = alpha * quad->p[a][1] + (1 - alpha) * quad->p[b][1], Mn = 0, Mcount = 0, range = quad_decimate + 1;
            for (float_t n = -range; n <= range; n += 0.25) {
                float_t grange = 1;
                int x1 = x0 + (n + grange) * nx, y1 = y0 + (n + grange) * ny;
                if (x1 < 0 || x1 >= M || y1 < 0 || y1 >= N) continue;
                int x2 = x0 + (n - grange) * nx, y2 = y0 + (n - grange) * ny;
                if (x2 < 0 || x2 >= M || y2 < 0 || y2 >= N) continue;
                int g1 = im_orig[y1 * M + x1], g2 = im_orig[y2 * M + x2];
                if (g1 < g2) continue;
                float_t weight = (g2 - g1) * (g2 - g1);
                Mn += weight * n, Mcount += weight;
            }
            if (Mcount == 0) continue;
            float_t n0 = Mn / Mcount, bestx = x0 + n0 * nx, besty = y0 + n0 * ny;
            Mx += bestx, My += besty, Mxx += bestx * bestx, Mxy += bestx * besty, Myy += besty * besty, N++;
        }
        float_t Ex = Mx / N, Ey = My / N, Cxx = Mxx / N - Ex * Ex, Cxy = Mxy / N - Ex * Ey, Cyy = Myy / N - Ey * Ey,
                normal_theta = .5 * atan2f(-2 * Cxy, (Cyy - Cxx));
        nx = cosf(normal_theta), ny = sinf(normal_theta), lines[edge][0] = Ex, lines[edge][1] = Ey, lines[edge][2] = nx,
        lines[edge][3] = ny;
    }
    for (int i = 0; i < 4; i++) {
        float_t A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3], A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2],
                B0 = -lines[i][0] + lines[(i + 1) & 3][0], B1 = -lines[i][1] + lines[(i + 1) & 3][1], det = A00 * A11 - A10 * A01;
        if (fabs(det) > 0.001) {
            float_t W00 = A11 / det, W01 = -A01 / det, L0 = W00 * B0 + W01 * B1;
            quad->p[i][0] = lines[i][0] + L0 * A00, quad->p[i][1] = lines[i][1] + L0 * A10;
        } else {  // this is a bad sign. We'll just keep the corner we had.
        }
    }
}

inline void quad_update_homographies(quad &quad) {
    float_t corr_arr[4][4];

    for (int i = 0; i < 4; i++) {
        corr_arr[i][0] = (i == 0 || i == 3) ? -1 : 1;
        corr_arr[i][1] = (i == 0 || i == 1) ? -1 : 1;
        corr_arr[i][2] = quad.p[i][0];
        corr_arr[i][3] = quad.p[i][1];
    }

    homography_compute2(quad.H, corr_arr);
    // rt_memcpy(quad.Hinv[0], quad.H[0], sizeof(quad.H));
    // Eigen::Map<Eigen::Matrix<float_t, 3, 3>> Hinv(quad.Hinv[0]);
    // Hinv = Hinv.inverse();
}

inline float_t value_for_pixel(uint8_t *im, float_t px, float_t py) {
    int x1 = floorf(px - 0.5);
    int x2 = ceilf(px - 0.5);
    float_t x = px - 0.5 - x1;
    int y1 = floorf(py - 0.5);
    int y2 = ceilf(py - 0.5);
    float_t y = py - 0.5 - y1;
    if (x1 < 0 || x2 >= M || y1 < 0 || y2 >= N) return -1;
    return im[y1 * M + x1] * (1 - x) * (1 - y) + im[y1 * M + x2] * x * (1 - y) + im[y2 * M + x1] * (1 - x) * y +
           im[y2 * M + x2] * x * y;
}

inline void sharpen(float_t *values, int size) {
    float_t *sharpened = (float_t *)staticBuffer.allocate(sizeof(float_t) * sq(size));
    float_t kernel[9] = {0, -1, 0, -1, 4, -1, 0, -1, 0};
    rep(y, 0, size) rep(x, 0, size) {
        sharpened[y * size + x] = 0;
        rep(i, 0, 3) rep(j, 0, 3) {
            if ((y + i - 1) < 0 || (y + i - 1) > size - 1 || (x + j - 1) < 0 || (x + j - 1) > size - 1) continue;
            sharpened[y * size + x] += values[(y + i - 1) * size + (x + j - 1)] * kernel[i * 3 + j];
        }
    }
    rep(y, 0, size) rep(x, 0, size) values[y * size + x] = values[y * size + x] + decode_sharpening * sharpened[y * size + x];
    staticBuffer.pop(sizeof(float_t) * sq(size));  // float_t sharpened[sq(size)];
}

inline uint64_t rotate90(uint64_t w, int numBits) {
    int p = numBits;
    uint64_t l = 0;
    if (numBits % 4 == 1) {
        p = numBits - 1;
        l = 1;
    }
    w = ((w >> l) << (p / 4 + l)) | (w >> (3 * p / 4 + l) << l) | (w & l);
    w &= ((uint64_t(1) << numBits) - 1);
    return w;
}

float decode_quad(const apriltag_family &family, uint8_t *im, quad &quad, quick_decode_entry &entry) {
    const float patterns[][5]{
        // left white column
        -0.5, 0.5, 0, 1, 1,
        // left black column
        0.5, 0.5, 0, 1, 0,
        // right white column
        family.width_at_border + 0.5f, .5, 0, 1, 1,
        // right black column
        family.width_at_border - 0.5f, .5, 0, 1, 0,
        // top white row
        0.5, -0.5, 1, 0, 1,
        // top black row
        0.5, 0.5, 1, 0, 0,
        // bottom white row
        0.5, family.width_at_border + 0.5f, 1, 0, 1,
        // bottom black row
        0.5, family.width_at_border - 0.5f, 1, 0, 0
        // XXX float_t-counts the corners.
    };
    graymodel whitemodel, blackmodel;
    for (auto pattern : patterns) {
        const bool is_white = pattern[4];
        rep(i, 0, family.width_at_border) {
            float_t tagx01 = (pattern[0] + i * pattern[2]) / (family.width_at_border);
            float_t tagy01 = (pattern[1] + i * pattern[3]) / (family.width_at_border);
            float_t tagx = 2 * (tagx01 - 0.5);
            float_t tagy = 2 * (tagy01 - 0.5);
            float_t px, py;
            homography_project(quad.H, tagx, tagy, &px, &py);
            int ix = px;
            int iy = py;
            if (ix < 0 || iy < 0 || ix >= M || iy >= N) continue;
            int v = im[iy * M + ix];
            if (is_white) whitemodel.add(tagx, tagy, v);
            else
                blackmodel.add(tagx, tagy, v);
        }
    }
    if (family.width_at_border > 1) {
        whitemodel.solve();
        blackmodel.solve();
    } else {
        whitemodel.solve();
        blackmodel.C[0] = 0, blackmodel.C[1] = 0, blackmodel.C[2] = blackmodel.B[2] / 4;
    }

    if ((whitemodel.interpolate(0, 0) - blackmodel.interpolate(0, 0) < 0) != family.reversed_border) return false;

    float black_score = 0, white_score = 0;
    float black_score_count = 1, white_score_count = 1;

    float_t *values = (float_t *)staticBuffer.allocate(sizeof(float_t) * sq(family.total_width));
    rt_memset(values, 0, sizeof(float_t) * sq(family.total_width));

    int min_coord = (family.width_at_border - family.total_width) / 2;
    for (int i = 0; i < family.nbits; i++) {
        int bity = family.bit_y[i];
        int bitx = family.bit_x[i];

        float_t tagx01 = (bitx + 0.5) / (family.width_at_border);
        float_t tagy01 = (bity + 0.5) / (family.width_at_border);

        // scale to [-1, 1]
        float_t tagx = 2 * (tagx01 - 0.5);
        float_t tagy = 2 * (tagy01 - 0.5);

        float_t px, py;
        homography_project(quad.H, tagx, tagy, &px, &py);

        float_t v = value_for_pixel(im, px, py);

        if (v == -1) continue;

        float_t thresh = (blackmodel.interpolate(tagx, tagy) + whitemodel.interpolate(tagx, tagy)) / 2.0;
        values[family.total_width * (bity - min_coord) + bitx - min_coord] = v - thresh;
    }

    sharpen(values, family.total_width);

    uint64_t rcode = 0;
    for (int i = 0; i < family.nbits; i++) {
        int bity = family.bit_y[i];
        int bitx = family.bit_x[i];
        rcode = (rcode << 1);
        float_t v = values[(bity - min_coord) * family.total_width + bitx - min_coord];

        if (v > 0) {
            white_score += v;
            white_score_count++;
            rcode |= 1;
        } else {
            black_score -= v;
            black_score_count++;
        }
    }

    quick_decode::codeword(family, rcode, entry);
    staticBuffer.pop(sizeof(float_t) * sq(family.total_width));  // float_t values[sq(family.total_width)];.
    return min(white_score / white_score_count, black_score / black_score_count);
}

detections_t *decode_quads(const apriltag_family &family, uint8_t *im, quads_t &quads, bool debug) {
    detections_t *detections = new (staticBuffer.allocate(sizeof(detections_t))) detections_t(detections_alloc_t{staticBuffer});

    for (auto &quad : quads) {
        refine_edges(im, &quad);
        quad_update_homographies(quad);

        quick_decode_entry entry;
        float decision_margin = decode_quad(family, im, quad, entry);
        if (debug) {
            plotInt(im, quad.p[0][1], quad.p[0][0], decision_margin, 3, true);
            plotInt(im, quad.p[1][1], quad.p[1][0], entry.hamming, 3, true);
        }
        if (decision_margin >= min_decision_margin && entry.hamming < 255) {  //
            detections->push_front(new (staticBuffer.allocate(sizeof(apriltag_detection))) apriltag_detection);
            auto &det = *detections->front();
            det.family = &family;
            det.id = entry.id;
            det.hamming = entry.hamming;
            det.decision_margin = decision_margin;
            if (debug) plotInt(im, quad.p[2][1], quad.p[2][0], det.id, 2, true);

#if (0)
            float_t theta = entry.rotation * (EIGEN_PI / 2.0), c = cosf(theta), s = sinf(theta);
#else
            float_t c, s;
            switch (entry.rotation) {
            case 0: c = 1, s = 0; break;
            case 1: c = 0, s = 1; break;
            case 2: c = -1, s = 0; break;
            default: c = 0, s = -1; break;
            }
#endif
            Eigen::Matrix<float_t, 3, 3> R{{c, -s, 0}, {s, c, 0}, {0, 0, 1}};
            Eigen::Map<Eigen::Matrix<float_t, 3, 3>>(det.H[0]).noalias() =
                R * Eigen::Map<Eigen::Matrix<float_t, 3, 3>>(quad.H[0]);
            homography_project(det.H, 0, 0, &det.c[0], &det.c[1]);

            for (int i = 0; i < 4; i++) {
                int tcx = (i == 0 || i == 3) ? -1 : 1;
                int tcy = (i == 0 || i == 1) ? -1 : 1;
                homography_project(det.H, tcx, tcy, &det.p[i][0], &det.p[i][1]);
            }
        }
    }

    return detections;
}

}  // namespace apriltag
}  // namespace imgProc