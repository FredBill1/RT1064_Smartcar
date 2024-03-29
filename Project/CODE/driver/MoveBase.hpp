#ifndef _driver_MoveBase_hpp
#define _driver_MoveBase_hpp

#include <rtthread.h>

#include "pose_kalman/config.hpp"
#include "utils/FakeAtomic.hpp"

class MoveBase {
 public:
    static constexpr pose_kalman::T default_xy_tolerance = 1e-2;
    static constexpr pose_kalman::T default_yaw_tolerance = (5 * 3.14 / 180);
    static constexpr pose_kalman::T default_xy_near = 0;
    static constexpr pose_kalman::T default_yaw_near = 0;
    static constexpr uint64_t default_time_tolerance_us = uint64_t(15e4);
    struct Goal {
        pose_kalman::T x, y, yaw;
        pose_kalman::T xy_tolerance;
        pose_kalman::T yaw_tolerance;
        pose_kalman::T xy_near;
        pose_kalman::T yaw_near;
        uint64_t time_tolerance_us;
        bool reached;
        const void* params;
        static constexpr Goal getDefault() {
            return {
                .xy_tolerance = default_xy_tolerance,
                .yaw_tolerance = default_yaw_tolerance,
                .xy_near = default_xy_near,
                .yaw_near = default_yaw_near,
                .time_tolerance_us = default_time_tolerance_us,
                .reached = false,
                .params = nullptr,
            };
        }
    };
    struct State {
        uint64_t timestamp_us;
        State() = default;
        pose_kalman::T state[6];
        State(uint64_t timestamp_us, const pose_kalman::T* state);
        State(uint64_t timestamp_us, pose_kalman::T x = 0, pose_kalman::T y = 0, pose_kalman::T yaw = 0, pose_kalman::T vX = 0,
              pose_kalman::T vY = 0, pose_kalman::T vYaw = 0);
        pose_kalman::T& x() { return state[0]; }
        pose_kalman::T& y() { return state[1]; }
        pose_kalman::T& yaw() { return state[2]; }
        pose_kalman::T& vX() { return state[3]; }
        pose_kalman::T& vY() { return state[4]; }
        pose_kalman::T& vYaw() { return state[5]; }
        pose_kalman::T x() const { return state[0]; }
        pose_kalman::T y() const { return state[1]; }
        pose_kalman::T yaw() const { return state[2]; }
        pose_kalman::T vX() const { return state[3]; }
        pose_kalman::T vY() const { return state[4]; }
        pose_kalman::T vYaw() const { return state[5]; }
    };

 private:
    bool _enabled = false;
    bool _new_goal = false;
    Goal _goal;
    FakeAtomicLoader<bool> _enabledLoader;
    FakeAtomicLoader<Goal> _goalLoader;
    FakeAtomicLoader<State> _stateLoader;
    FakeAtomicLoader<pose_kalman::T> _yawLoader;
    rt_event _reachedEvent;
    rt_event _nearEvent;
    State _state;

 public:
    enum class GoalEventFlag : rt_uint32_t {
        reached = 1 << 0,   // 到达目标
        disabled = 1 << 1,  // 设置禁用
        timeout = 1 << 2,   // 超时
    };
    MoveBase();
    void set_enabled(bool enabled);
    bool get_enabled();
    void send_goal(const Goal& goal);
    const Goal& get_goal();
    void set_reached(bool reached = true);
    void send_reached(bool reached = true);
    void send_near();
    bool wait_near(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_set_state(const State& state);
    bool get_set_state(State& new_state);
    void send_state(uint64_t timestamp_us, const pose_kalman::T* state);
    void send_state(const State& state);
    void get_state(State& state) const;
    bool get_yaw(pose_kalman::T& new_yaw);
    GoalEventFlag wait_for_result(rt_int32_t timeout = RT_WAITING_FOREVER);
    bool get_reached();
    bool new_goal();
};

#endif  // _driver_MoveBase_hpp