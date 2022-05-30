#ifndef _imgProc_apriltag_RectSender_hpp
#define _imgProc_apriltag_RectSender_hpp

#include "RectConfig.hpp"
#include "SerialIO.hpp"
#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

class RectSender {
    uint8_t buf[SerialIO::HeaderSize + 2 + SerialIO::TxWirter<float>::getSize(max_rect_cnt * 2)];
    SerialIO::TxXfer xfer;

 public:
    RectSender(uint8_t id, const char* name = "rect_tx") : xfer(buf, sizeof(buf), name) {
        SerialIO::applyHeader(buf);
        buf[SerialIO::HeaderSize] = id;
    }

    bool send_to(rects_t& rects, SerialIO& uart, int32_t timeout_ms = 0) {
        if (!xfer.txFinished(timeout_ms)) return false;
        SerialIO::TxWirter<float> writer(buf + SerialIO::HeaderSize + 2);
        int i = 0;
        for (auto& r : rects) {
            if (i >= max_rect_cnt) break;
            writer.set(i * 2, r->c_proj[0]);
            writer.set(i * 2 + 1, r->c_proj[1]);
            ++i;
        }
        buf[SerialIO::HeaderSize + 1] = i * 2;
        xfer.setSize(SerialIO::HeaderSize + 2 + writer.getSize(i * 2));
        uart.send(xfer);
        return true;
    }
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _imgProc_apriltag_RectSender_hpp