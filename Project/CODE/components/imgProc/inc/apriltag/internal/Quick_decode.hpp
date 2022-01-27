#ifndef _apriltag_Quick_decode_hpp
#define _apriltag_Quick_decode_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

struct quick_decode_entry {
    uint64_t rcode;    // the queried code
    uint16_t id;       // the tag ID (a small integer)
    uint8_t hamming;   // how many errors corrected?
    uint8_t rotation;  // number of rotations [0, 3]
};

struct quick_decode {
    int nentries;
    struct quick_decode_entry* entries;
    quick_decode() = delete;
    static void init(apriltag_family& family, int maxhamming, bool static_allocate = false);
    static void codeword(const apriltag_family& tf, uint64_t rcode, quick_decode_entry& entry);

 private:
    void add(uint64_t code, int id, int hamming);
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_Quick_decode_hpp