#ifndef _driver_MoveBase_hpp
#define _driver_MoveBase_hpp

#include <rtthread.h>

#include "pose_kalman/config.hpp"
#include "utils/FakeAtomic.hpp"

class MoveBase {
 public:
    struct Goal {
        pose_kalman::T x, y, yaw;
        bool reached;
    };

 private:
    bool _enabled = false;
    Goal _goal;
    FakeAtomicLoader<bool> _enabledLoader;
    FakeAtomicLoader<Goal> _goalLoader;
    rt_event _reachedEvent;

 public:
    MoveBase();
    void set_enabled(bool enabled);
    bool get_enabled();
    void send_goal(float x, float y, float yaw);
    const Goal& get_goal();
    bool wait_for_result();
    bool get_reached();
};

#endif  // _driver_MoveBase_hpp