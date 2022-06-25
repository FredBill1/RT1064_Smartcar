#include "MasterGlobalVars.hpp"

#include <algorithm>

#include "Systick.hpp"
#include "edge_detect/A4Detect.hpp"
#include "utils/InterruptGuard.hpp"
using namespace imgProc::edge_detect;

MasterGlobalVars::MasterGlobalVars() {
    rt_event_init(&coord_recv_event, "coord_recv_event", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_snapshot_event, "art_snapshot_event", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_result_event, "art_result_event", RT_IPC_FLAG_PRIO);
}

void MasterGlobalVars::signal_reset() {
    InterruptGuard guard;
    reset_flag = true;
}

bool MasterGlobalVars::reset_requested() {
    InterruptGuard guard;
    bool flag = reset_flag;
    reset_flag = false;
    return flag;
}

bool MasterGlobalVars::wait_for_coord_recv(rt_int32_t timeout) {
    return rt_event_recv(&coord_recv_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}

void MasterGlobalVars::send_coord_recv(int cnt, const float* coords) {
    {
        ScheduleGuard guard;
        target_coords_cnt = cnt;
        rt_memcpy(target_coords_corr, coords, sizeof(target_coords_corr[0]) * cnt);
    }
    rt_event_send(&coord_recv_event, 1);
}

void MasterGlobalVars::get_coord_recv() {
    ScheduleGuard guard;
    coords_cnt = target_coords_cnt;
    rt_memcpy(coords + 1, target_coords_corr, sizeof(target_coords_corr[0]) * coords_cnt);
}

bool MasterGlobalVars::get_rectTarget(float target[3], uint64_t timestamp_us) {
    InterruptGuard guard;
    if (!_rectTargetEnabled) return false;
    if (!_rectStarted) {
        if (systick.get_diff_us(_rectStartTimestamp_us, timestamp_us) >= _rectCooldown_us) _rectStarted = true;
        else
            return false;
    }

    std::copy(_rectTarget, _rectTarget + 3, target);
    return true;
}
void MasterGlobalVars::send_rectTarget(bool enabled, uint64_t timestamp_us, int64_t cooldown_us, const float target[3]) {
    InterruptGuard guard;
    _rectTargetEnabled = enabled;
    if (!enabled) return;

    _rectStarted = false;
    _rectStartTimestamp_us = timestamp_us;
    _rectCooldown_us = cooldown_us;
    std::copy(target, target + 3, _rectTarget);
}

bool MasterGlobalVars::wait_art_snapshot(int index, rt_int32_t timeout) {
    if (index) {
        InterruptGuard guard;
        _art_cur_index = index;
    }
    rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);  // 清除接收标志位
    return rt_event_recv(&art_snapshot_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}
void MasterGlobalVars::send_art_snapshot() { rt_event_send(&art_snapshot_event, 1); }
bool MasterGlobalVars::wait_art_result(uint8_t& result, rt_int32_t timeout) {
    if (rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) != RT_EOK) return false;
    {
        InterruptGuard guard;
        result = _art_result;
    }
    return true;
}
void MasterGlobalVars::send_art_result(uint8_t result) {
    bool need_result;
    {
        InterruptGuard guard;
        need_result = _art_cur_index;
        if (need_result) {
            _art_result = result;
            art_results[_art_cur_index] = result;
            _art_cur_index = 0;
        }
    }
    if (need_result) rt_event_send(&art_result_event, 1);
}

MasterGlobalVars masterGlobalVars;