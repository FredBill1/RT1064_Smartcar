#ifndef _pose_kalman_NoiseGenerator_hpp
#define _pose_kalman_NoiseGenerator_hpp

#include <random>

#include "pose_kalman/config.hpp"
namespace pose_kalman {
class NoiseGenerator {
    static std::default_random_engine generator;
    std::normal_distribution<T> distribution;

 public:
    NoiseGenerator(T sigma2 = 1) : distribution(0, sigma2) {}
    T operator()() { return distribution(generator); }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_NoiseGenerator_hpp