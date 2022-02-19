#ifndef _geometry_msgs_hpp
#define _geometry_msgs_hpp

#include "std_msgs.hpp"

namespace rosRT {
namespace msgs {

struct Quaternion {
    float_t x;
    float_t y;
    float_t z;
    float_t w;
};

struct QuaternionStamped {
    Header header;
    Quaternion quaternion;
};

struct Vector3 {
    float_t x;
    float_t y;
    float_t z;
};

struct Vector3Stamped {
    Header header;
    Vector3 vector;
};

struct Transform {
    Vector3 translation;
    Quaternion rotation;
};

struct TransformStamped {
    Header header;
    Transform transform;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct TwistStamped {
    Header header;
    Twist twist;
};

}  // namespace msgs
}  // namespace rosRT

#endif  // _geometry_msgs_hpp