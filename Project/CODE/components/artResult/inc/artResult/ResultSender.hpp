#ifndef _artResult_ResultSender_hpp
#define _artResult_ResultSender_hpp

#include "SerialIO.hpp"
#include "artResult/ResultCatgory.hpp"

class ResultSender {
    uint8_t buf[18] = "000.000 00 00 0 0";
    SerialIO::TxXfer xfer;
    SerialIO& serial;

 public:
    ResultSender(SerialIO& serial, const char* name) : serial(serial), xfer(buf, sizeof(buf), name) { buf[17] = '\n'; }
    bool send(int x, int y, ResultCatgory::Minor catgory, int32_t timeout_ms = RT_WAITING_FOREVER) {
        if (!xfer.txFinished(timeout_ms)) return false;
        rt_tick_t ms = rt_tick_get_millisecond();
        buf[6] = ms % 10 + '0', ms /= 10;
        buf[5] = ms % 10 + '0', ms /= 10;
        buf[4] = ms % 10 + '0', ms /= 10;
        buf[2] = ms % 10 + '0', ms /= 10;
        buf[1] = ms % 10 + '0', ms /= 10;
        buf[0] = ms % 10 + '0';

        buf[9] = x % 10 + '0', x /= 10;
        buf[8] = x % 10 + '0';

        buf[11] = y % 10 + '0', y /= 10;
        buf[10] = y % 10 + '0';

        using namespace ResultCatgory;
        buf[16] = minor_to_index(catgory) + '0';
        buf[14] = major_to_index(minor_to_major(catgory)) + '0';

        serial.send(xfer);
        return true;
    }
};

#endif  // _artResult_ResultSender_hpp