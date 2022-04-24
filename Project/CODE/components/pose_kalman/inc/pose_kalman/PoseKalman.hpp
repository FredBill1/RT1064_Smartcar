#ifndef _pose_kalman_PoseKalman_hpp
#define _pose_kalman_PoseKalman_hpp

#include <cstdint>

#include "pose_kalman/config.hpp"

namespace pose_kalman {

class PoseKalman {
    struct Impl;
    Impl* pimpl;

 public:
    PoseKalman();
    ~PoseKalman();
    void setEnabled(bool enable);
    void setState(const T stateData[], uint64_t timestamp_us = 0);
    void setSystemCovariance(const T systemCovariance[]);
    void setMeasurementCovariance(MeasurementType measurementType, const T measurementCovariance[]);
    void enqueMeasurement(MeasurementType measurementType, const T measurementData[], uint64_t timestamp_us);
    void update(uint64_t timestamp_us);
};

}  // namespace pose_kalman

#endif  // _pose_kalman_PoseKalman_hpp