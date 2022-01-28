#ifndef _apriltag_Hashmap_hpp
#define _apriltag_Hashmap_hpp

#include <cstdint>
#include <forward_list>
#include <functional>
#include <utility>

#include "apriltag/apriltag.hpp"
#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {
class Hashmap {
 public:
    using key_type = ID_t;
    using mapped_type = List_pt_t*;
    using value_type = std::pair<key_type, mapped_type>;
    using allocator_type = StaticAllocator<value_type>;
    using bucket_type = std::forward_list<value_type, allocator_type>;
    uint8_t _bucketbuf[HASH_BUCKET_CNT * sizeof(bucket_type)];
    bucket_type* const buckets = (bucket_type*)_bucketbuf;

 private:
    Hashmap(StaticBuffer& buf);
    bucket_type& findBucket(key_type id);

 public:
    static Hashmap& create(void* pos, StaticBuffer& buf);
    mapped_type& operator[](key_type id);
    void for_each(std::function<void(mapped_type&)> func);
};
}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_Hashmap_hpp