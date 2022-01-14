#ifndef _sensor_msgs_hpp
#define _sensor_msgs_hpp

#include "geometry_msgs.hpp"

namespace rosRT {
namespace msgs {

struct VectBias {
    Vector3 vect;
    Vector3 bias;
};

struct Imu {
    Header header;
    Quaternion orientation;
    float64 orientation_covariance[9];
    Vector3 angular_velocity;
    float64 angular_velocity_covariance[9];
    Vector3 linear_acceleration;
    float64 linear_acceleration_covariance[9];
};

}  // namespace msgs
}  // namespace rosRT

#endif  // _sensor_msgs_hpp