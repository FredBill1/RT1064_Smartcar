#include "apriltag/internal/Quick_decode.hpp"

#include <rtthread.h>

#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

void quick_decode::add(uint64_t code, int id, int hamming) {
    uint32_t bucket = code % nentries;
    while (entries[bucket].rcode != UINT64_MAX) bucket = (bucket + 1) % nentries;
    entries[bucket].rcode = code;
    entries[bucket].id = id;
    entries[bucket].hamming = hamming;
}

void quick_decode::init(apriltag_family& family, int maxhamming, bool static_allocate) {
    quick_decode* qd =
        (quick_decode*)(static_allocate ? (staticBuffer.allocate(sizeof(quick_decode))) : rt_malloc(sizeof(quick_decode)));

    rt_memset(qd, 0, sizeof(quick_decode));
    int capacity = family.ncodes;
    int nbits = family.nbits;
    if (maxhamming >= 1) capacity += family.ncodes * nbits;
    if (maxhamming >= 2) capacity += family.ncodes * nbits * (nbits - 1);
    if (maxhamming >= 3) capacity += family.ncodes * nbits * (nbits - 1) * (nbits - 2);
    qd->nentries = capacity * 3;

    rep(i, 0, qd->nentries) qd->entries[i].rcode = UINT64_MAX;
    rep(i, 0, family.ncodes) {
        uint64_t code = family.codes[i];
        qd->add(code, i, 0);
        if (maxhamming >= 1) rep(j, 0, nbits) qd->add(code ^ (uint64_t(1) << j), i, 1);
        if (maxhamming >= 2) rep(j, 0, nbits) rep(k, 0, j) qd->add(code ^ (uint64_t(1) << j) ^ (uint64_t(1) << k), i, 2);
        if (maxhamming >= 3)
            rep(j, 0, nbits) rep(k, 0, j) rep(m, 0, k)
                qd->add(code ^ (uint64_t(1) << j) ^ (uint64_t(1) << k) ^ (uint64_t(1) << m), i, 3);
    }
    family.impl = qd;
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

void quick_decode::codeword(const apriltag_family& tf, uint64_t rcode, quick_decode_entry& entry) {
    quick_decode* qd = (quick_decode*)tf.impl;
    rep(ridx, 0, 4) {
        for (int bucket = rcode % qd->nentries; qd->entries[bucket].rcode != UINT64_MAX; bucket = (bucket + 1) % qd->nentries) {
            if (qd->entries[bucket].rcode == rcode) {
                entry = qd->entries[bucket];
                entry.rotation = ridx;
                return;
            }
        }
        rcode = rotate90(rcode, tf.nbits);
    }
    entry.rcode = 0;
    entry.id = 65535;
    entry.hamming = 255;
    entry.rotation = 0;
}

}  // namespace apriltag
}  // namespace imgProc