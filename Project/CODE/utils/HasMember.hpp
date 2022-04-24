#ifndef _utils_HasMember_hpp
#define _utils_HasMember_hpp

// https://stackoverflow.com/a/1007175

#define HAS_MEMBER_PREDEFINE(member)                                                   \
    template <typename T> struct HAS_MEMBER_UTIL_##member {                            \
        struct Fallback {                                                              \
            int member;                                                                \
        };                                                                             \
        struct Derived : private T, Fallback {};                                       \
        template <typename C, C> struct ChT;                                           \
        template <typename C> static char (&f(ChT<int Fallback::*, &C::member> *))[1]; \
        template <typename C> static char (&f(...))[2];                                \
        static constexpr bool value = sizeof(f<Derived>(0)) == 2;                      \
    };

#define HAS_MEMBER(cls, member) (HAS_MEMBER_UTIL_##member<cls>::value)

#endif  // _utils_HasMember_hpp