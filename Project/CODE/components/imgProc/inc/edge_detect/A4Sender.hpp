#ifndef _imgProc_edge_detect_A4Sender_hpp
#define _imgProc_edge_detect_A4Sender_hpp

#include "SerialIO.hpp"
#include "edge_detect/A4Detect.hpp"

namespace imgProc {
namespace edge_detect {

class A4Sender {
    uint8_t buf[SerialIO::HeaderSize + 2 + SerialIO::TxWirter<float>::getSize(target_coords_maxn * 2)];
    SerialIO::TxXfer xfer;

 public:
    A4Sender(uint8_t id, const char* name = "a4_tx") : xfer(buf, sizeof(buf), name) {
        SerialIO::applyHeader(buf);
        buf[SerialIO::HeaderSize] = id;
    }

    bool send_to(SerialIO& uart, int32_t timeout_ms = 0) {
        if (!xfer.txFinished(timeout_ms)) return false;
        SerialIO::TxWirter<float> writer(buf + SerialIO::HeaderSize + 2);
        buf[SerialIO::HeaderSize + 1] = target_coords_cnt * 2;
        xfer.setSize(SerialIO::HeaderSize + 2 + writer.getSize(target_coords_cnt * 2));
        writer.setArr((float_t*)target_coords_corr, target_coords_cnt * 2);
        uart.send(xfer);
        return true;
    }
};

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_A4Sender_hpp