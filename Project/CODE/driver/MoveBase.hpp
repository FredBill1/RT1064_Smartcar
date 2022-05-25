#ifndef _driver_MoveBase_hpp
#define _driver_MoveBase_hpp

#include <rtthread.h>

#include "pose_kalman/config.hpp"
#include "utils/FakeAtomic.hpp"

class MoveBase {
 public:
    struct Goal {
        pose_kalman::T x, y, yaw;
        pose_kalman::T xy_tolerance, yaw_tolerance;
        bool reached;
    };
    struct State {
        uint64_t timestamp_us;
        pose_kalman::T state[6];
        State() = default;
        State(uint64_t timestamp_us, const pose_kalman::T* state);
        State(uint64_t timestamp_us, pose_kalman::T x = 0, pose_kalman::T y = 0, pose_kalman::T yaw = 0, pose_kalman::T vX = 0,
              pose_kalman::T vY = 0, pose_kalman::T vYaw = 0);
    };

 private:
    bool _enabled = false;
    bool _new_goal = false;
    Goal _goal;
    FakeAtomicLoader<bool> _enabledLoader;
    FakeAtomicLoader<Goal> _goalLoader;
    FakeAtomicLoader<State> _stateLoader;
    rt_event _reachedEvent;

 public:
    MoveBase();
    void set_enabled(bool enabled);
    bool get_enabled();
    void send_goal(pose_kalman::T x, pose_kalman::T y, pose_kalman::T yaw, pose_kalman::T xy_tolerance = 1e-2,
                   pose_kalman::T yaw_tolerance = (5 * 3.14 / 180));
    const Goal& get_goal();
    void set_reached(bool reached = true);
    void send_reached(bool reached = true);
    void send_state(const State& state);
    bool get_state(State& new_state);
    bool wait_for_result();
    bool get_reached();
    bool new_goal();
};

#endif  // _driver_MoveBase_hpp