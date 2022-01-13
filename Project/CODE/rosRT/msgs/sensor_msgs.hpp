#ifndef _sensor_msgs_hpp
#define _sensor_msgs_hpp

#include "geometry_msgs.hpp"

namespace rosRT {
namespace msgs {

struct Imu {
    Header header;
    Quaternion orientation;
    float orientation_covariance[9];
    Vector3 angular_velocity;
    float angular_velocity_covariance[9];
    Vector3 linear_acceleration;
    float linear_acceleration_covariance[9];
};

}  // namespace msgs
}  // namespace rosRT

#endif  // _sensor_msgs_hpp