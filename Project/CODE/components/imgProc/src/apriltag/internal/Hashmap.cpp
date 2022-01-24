#include "apriltag/internal/Hashmap.hpp"

#include "apriltag/internal/StaticBuffer.hpp"

namespace imgProc {
namespace apriltag {

Hashmap::Hashmap(StaticBuffer& buf) {
    for (int_fast32_t i = 0; i < HASH_BUCKET_CNT; ++i) new (buckets + i) bucket_type(allocator_type{buf});
}

Hashmap::bucket_type& Hashmap::findBucket(key_type id) { return buckets[id % HASH_BUCKET_CNT]; }

Hashmap& Hashmap::create(void* pos, StaticBuffer& buf) { return *(new (pos) Hashmap(buf)); }

Hashmap::mapped_type& Hashmap::operator[](key_type id) {
    bucket_type& bucket = findBucket(id);
    for (auto& [key, value] : bucket)
        if (key == id) return value;
    bucket.emplace_front(id, nullptr);
    return bucket.front().second;
}

void Hashmap::for_each(std::function<void(mapped_type&)> func) {
    for (int_fast32_t i = 0; i < HASH_BUCKET_CNT; ++i)
        for (auto& [k, v] : buckets[i]) func(v);
}

}  // namespace apriltag
}  // namespace imgProc