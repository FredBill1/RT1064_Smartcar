#include "MasterGlobalVars.hpp"

#include <algorithm>

#include "Systick.hpp"
#include "edge_detect/A4Detect.hpp"
#include "utils/InterruptGuard.hpp"
using namespace imgProc::edge_detect;

MasterGlobalVars::MasterGlobalVars() {
    rt_event_init(&coord_recv_event, "coord_recv", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_snapshot_event, "art_snapshot", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_result_event, "art_result", RT_IPC_FLAG_PRIO);
    rt_event_init(&drop_rect_event, "drop_rect", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_border_event, "art_border", RT_IPC_FLAG_PRIO);
}

void MasterGlobalVars::reset_states() {
    rt_event_recv(&coord_recv_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    rt_event_recv(&art_snapshot_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    rt_event_recv(&drop_rect_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    rt_event_recv(&art_border_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
    {
        InterruptGuard guard;
        reset_flag = false;
        _rectEnabled = false;
        _art_need_result = false;
        for (auto& res : art_results) res = ResultCatgory::Major::None;
    }
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
        coord_valid.set();
    }
    rt_event_send(&coord_recv_event, 1);
}

void MasterGlobalVars::get_coord_recv() {
    ScheduleGuard guard;
    coords_cnt = target_coords_cnt;
    rt_memcpy(coords + 1, target_coords_corr, sizeof(target_coords_corr[0]) * coords_cnt);
}

void MasterGlobalVars::send_rects_enabled(bool enabled, float maxDistErrorSquared) {
    InterruptGuard guard;
    _rectEnabled = enabled;
    if (!enabled) return;
    _rectMaxDistErrorSquared = maxDistErrorSquared;
    _rectCnt = 0;
}

void MasterGlobalVars::send_rects(const float state[3], const float* rects, int cnt, uint64_t timestamp_us) {
    InterruptGuard guard;
    if (!_rectEnabled) return;
    _rectCnt = cnt;
    _rectTimestamp_us = timestamp_us;
    rt_memcpy(_rectRecvingState, state, sizeof(_rectRecvingState[0]) * 3);
    rt_memcpy(_rectCoords[0], rects, sizeof(_rectCoords[0]) * cnt);
}

void MasterGlobalVars::get_rects(float state[3], float* rects, int& cnt, float& maxDistErrorSquared, uint64_t& timestamp_us) {
    InterruptGuard guard;
    if (!_rectEnabled || _rectCnt == 0) {
        cnt = 0;
        return;
    }
    cnt = _rectCnt;
    rt_memcpy(state, _rectRecvingState, sizeof(_rectRecvingState[0]) * 3);
    rt_memcpy(rects, _rectCoords[0], sizeof(_rectCoords[0]) * cnt);
    maxDistErrorSquared = _rectMaxDistErrorSquared;
    timestamp_us = _rectTimestamp_us;
    _rectCnt = 0;
}

void MasterGlobalVars::send_art_cur_index(int index) {
    {
        InterruptGuard guard;
        _art_cur_index = index;
        _art_need_result = true;
    }
    rt_event_recv(&art_snapshot_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);  // 清除snapshot事件
    rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);  // 清除art接收事件
}

bool MasterGlobalVars::wait_art_snapshot(rt_int32_t timeout) {
    return rt_event_recv(&art_snapshot_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}
void MasterGlobalVars::send_art_snapshot() { rt_event_send(&art_snapshot_event, 1); }

bool MasterGlobalVars::wait_art_result(rt_int32_t timeout) {
    return rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}

bool MasterGlobalVars::send_art_result(ResultCatgory::Major result) {
    bool need_result;
    {
        InterruptGuard guard;
        need_result = _art_need_result;
        if (need_result) {
            art_results[_art_cur_index] = result;
            _art_need_result = false;
        }
    }
    if (need_result) rt_event_send(&art_result_event, 1);
    return need_result;
}
ResultCatgory::Major MasterGlobalVars::get_art_result() const {
    InterruptGuard guard;
    return art_results[_art_cur_index];
}

void MasterGlobalVars::send_drop_rect() { rt_event_send(&drop_rect_event, 1); }

bool MasterGlobalVars::wait_drop_rect(rt_int32_t timeout) {
    return rt_event_recv(&drop_rect_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}

void MasterGlobalVars::send_art_border() { rt_event_send(&art_border_event, 1); }
bool MasterGlobalVars::wait_art_border(rt_int32_t timeout) {
    return rt_event_recv(&art_border_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}
void MasterGlobalVars::clear_art_border() {
    rt_event_recv(&art_border_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, RT_NULL);
}

void MasterGlobalVars::send_upload_xy(const uint8_t xy[2]) {
    InterruptGuard guard;
    _upload_xy[0] = xy[0], _upload_xy[1] = xy[1];
}

void MasterGlobalVars::get_upload_xy(uint8_t xy[2]) const {
    InterruptGuard guard;
    xy[0] = _upload_xy[0], xy[1] = _upload_xy[1];
}

MasterGlobalVars masterGlobalVars;