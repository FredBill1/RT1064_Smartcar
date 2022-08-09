#include "artResult/ResultSender.hpp"

#include "MasterGlobalVars.hpp"
#include "fieldParam.hpp"

ResultSender::ResultSender(SerialIO& serial, const char* name) : serial(serial), xfer(buf, sizeof(buf), name) { buf[17] = '\n'; }

void ResultSender::apply_time() {
    rt_tick_t ms = rt_tick_get_millisecond();
    buf[6] = ms % 10 + '0', ms /= 10;
    buf[5] = ms % 10 + '0', ms /= 10;
    buf[4] = ms % 10 + '0', ms /= 10;
    buf[2] = ms % 10 + '0', ms /= 10;
    buf[1] = ms % 10 + '0', ms /= 10;
    buf[0] = ms % 10 + '0';
}

void ResultSender::apply_xy(int x, int y) {
    buf[9] = x % 10 + '0', x /= 10;
    buf[8] = x % 10 + '0';

    buf[12] = y % 10 + '0', y /= 10;
    buf[11] = y % 10 + '0';
}

bool ResultSender::send_catgory(ResultCatgory::Minor catgory, int index, int32_t timeout_ms) {
    if (!xfer.txFinished(timeout_ms)) return false;
    apply_time();
    apply_xy(int(masterGlobalVars.coords[index][0] / squareSize) + 1, int(masterGlobalVars.coords[index][1] / squareSize) + 1);

    using namespace ResultCatgory;
    buf[16] = minor_to_index(catgory) + '0';
    buf[14] = major_to_index(minor_to_major(catgory)) + '0';

    serial.send(xfer);
    return true;
}

bool ResultSender::send_traverse(bool carrying, int32_t timeout_ms) {
    if (!xfer.txFinished(timeout_ms)) return false;
    apply_time();
    {
        uint8_t xy[2];
        masterGlobalVars.get_upload_xy(xy);
        apply_xy(xy[0], xy[1]);
    }

    buf[14] = carrying ? '9' : '0';
    // buf[16] = '0';  // ²»ÐèÒª

    serial.send(xfer);
    return true;
}