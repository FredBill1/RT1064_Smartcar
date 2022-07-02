#ifndef _artResult_ResultSender_hpp
#define _artResult_ResultSender_hpp

#include "SerialIO.hpp"
#include "artResult/ResultCatgory.hpp"

class ResultSender {
    uint8_t buf[18] = "000.000 00 00 0 0";
    SerialIO::TxXfer xfer;
    SerialIO& serial;
    void apply_time();
    void apply_xy();

 public:
    ResultSender(SerialIO& serial, const char* name);
    bool send_catgory(ResultCatgory::Minor catgory, int32_t timeout_ms = RT_WAITING_FOREVER);
    bool send_traverse(bool carrying, int32_t timeout_ms = RT_WAITING_FOREVER);
};

#endif  // _artResult_ResultSender_hpp