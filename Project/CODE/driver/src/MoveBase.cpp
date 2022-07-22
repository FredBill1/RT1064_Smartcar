#include "MoveBase.hpp"

#include "utils/InterruptGuard.hpp"

MoveBase::MoveBase() {
    _goal.reached = true;
    rt_event_init(&_reachedEvent, "GoalReached", RT_IPC_FLAG_PRIO);
    rt_event_init(&_nearEvent, "GoalNear", RT_IPC_FLAG_PRIO);
}

void MoveBase::set_enabled(bool enabled) {
    _enabledLoader.store(enabled);
    if (enabled) {
        rt_event_recv(&_reachedEvent, (rt_uint32_t)GoalEventFlag::disabled, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO,
                      RT_NULL);
        rt_event_recv(&_nearEvent, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    } else {
        {
            InterruptGuard guard;
            _goal.reached = true;
        }
        rt_event_send(&_reachedEvent, (rt_uint32_t)GoalEventFlag::disabled);
    }
}

bool MoveBase::get_enabled() {
    _enabledLoader.load(_enabled);
    return _enabled;
}

void MoveBase::send_goal(const Goal& goal) {
    rt_event_recv(&_nearEvent, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    _goalLoader.store(goal);
}

const MoveBase::Goal& MoveBase::get_goal() {
    if (_goalLoader.load(_goal)) _new_goal = true;
    return _goal;
}

bool MoveBase::get_reached() { return get_goal().reached; }
void MoveBase::set_reached(bool reached) {
    get_goal();
    if (_new_goal) return;
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

void MoveBase::send_near() {
    get_goal();
    if (_new_goal) return;
    rt_event_send(&_nearEvent, 1);
}
bool MoveBase::wait_near(rt_int32_t timeout) {
    return rt_event_recv(&_nearEvent, 1, RT_EVENT_FLAG_OR, timeout, RT_NULL) == RT_EOK;
}

MoveBase::State::State(uint64_t timestamp_us, const pose_kalman::T* state) {
    this->timestamp_us = timestamp_us;
    for (int i = 0; i < 6; i++) this->state[i] = state[i];
}

MoveBase::State::State(uint64_t timestamp_us, pose_kalman::T x, pose_kalman::T y, pose_kalman::T yaw, pose_kalman::T vX,
                       pose_kalman::T vY, pose_kalman::T vYaw) {
    this->timestamp_us = timestamp_us;
    state[0] = x, state[1] = y, state[2] = yaw, state[3] = vX, state[4] = vY, state[5] = vYaw;
}

void MoveBase::send_set_state(const State& state) {
    _yawLoader.store(state.state[2]);
    _stateLoader.store(state);
    send_state(state);
}

bool MoveBase::get_set_state(State& new_state) { return _stateLoader.load(new_state); }
bool MoveBase::get_yaw(pose_kalman::T& new_yaw) { return _yawLoader.load(new_yaw); }

void MoveBase::send_state(const State& state) {
    ScheduleGuard guard;
    _state = state;
}
void MoveBase::send_state(uint64_t timestamp_us, const pose_kalman::T* state) {
    ScheduleGuard guard;
    _state = State(timestamp_us, state);
}
void MoveBase::get_state(State& state) const {
    ScheduleGuard guard;
    state = _state;
}

MoveBase::GoalEventFlag MoveBase::wait_for_result(rt_int32_t timeout) {
    if (!get_enabled()) return GoalEventFlag::disabled;
    rt_uint32_t res;
    if (rt_event_recv(&_reachedEvent, (rt_uint32_t)GoalEventFlag::reached | (rt_uint32_t)GoalEventFlag::disabled,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &res) != RT_EOK)
        return GoalEventFlag::timeout;
    return res == (rt_uint32_t)GoalEventFlag::reached ? GoalEventFlag::reached : GoalEventFlag::disabled;
}

bool MoveBase::new_goal() {
    bool res = _new_goal;
    _new_goal = false;
    return res;
}