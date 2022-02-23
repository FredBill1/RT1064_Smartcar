#ifndef _ClassPlaceHolder_hpp
#define _ClassPlaceHolder_hpp

#include <cstdint>
#include <utility>

namespace utils {
template <typename T> class ClassPlaceHolder {
 private:
    T* ptr = nullptr;
    alignas(T) uint8_t buf[sizeof(T)];

 public:
    T& instance() { return *reinterpret_cast<T*>(ptr); }
    template <typename... Args> void initialize(Args&&... args) {
        if (ptr) destroy();
        ptr = new ((void*)buf) T(std::forward<Args>(args)...);
    }
    void destroy() {
        if (ptr) {
            ptr->~T();
            ptr = nullptr;
        }
    }
    ~ClassPlaceHolder() { destroy(); }
};
}  // namespace utils

#endif  //_ClassPlaceHolder_hpp