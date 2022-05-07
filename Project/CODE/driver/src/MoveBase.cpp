#include "MoveBase.hpp"

enum class GoalEventFlag : rt_uint32_t {
    reached = 1 << 0,   // 到达目标
    disabled = 1 << 1,  // 设置禁用
};

MoveBase::MoveBase() {
    _goal.reached = true;
    rt_event_init(&_reachedEvent, "GoalReached", RT_IPC_FLAG_PRIO);
}

void MoveBase::set_enabled(bool enabled) {
    _enabledLoader.store(enabled);
    if (!enabled) rt_event_send(&_reachedEvent, (rt_uint32_t)GoalEventFlag::disabled);
}

bool MoveBase::get_enabled() {
    _enabledLoader.load(_enabled);
    return _enabled;
}

void MoveBase::send_goal(pose_kalman::T x, pose_kalman::T y, pose_kalman::T yaw, pose_kalman::T xy_tolerance,
                         pose_kalman::T yaw_tolerance) {
    _goalLoader.store({x, y, yaw, xy_tolerance, yaw_tolerance, false});
}

const MoveBase::Goal& MoveBase::get_goal() {
    if (_goalLoader.load(_goal)) _new_goal = true;
    return _goal;
}

bool MoveBase::get_reached() { return get_goal().reached; }
void MoveBase::set_reached(bool reached) {
    _goal.reached = reached;
    if (reached) rt_event_send(&_reachedEvent, (rt_uint32_t)GoalEventFlag::reached);
}

void MoveBase::send_reached(bool reached) {
    auto goal = _goal;
    if (reached != goal.reached) {
        goal.reached = reached;
        _goalLoader.store(goal);
    }
}

bool MoveBase::wait_for_result() {
    if (!get_enabled()) return false;
    rt_uint32_t res;
    rt_event_recv(&_reachedEvent, (rt_uint32_t)GoalEventFlag::reached | (rt_uint32_t)GoalEventFlag::disabled,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &res);
    return res == (rt_uint32_t)GoalEventFlag::reached;
}

bool MoveBase::new_goal() {
    bool res = _new_goal;
    _new_goal = false;
    return res;
}