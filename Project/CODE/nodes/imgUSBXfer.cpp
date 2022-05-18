#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "zf_usb_cdc.h"
}
#include "apriltag/visualization.hpp"
#include "devices.hpp"

static void imgUSBXferEntry() {
    using namespace imgProc::apriltag;
    static uint8_t buf[4]{0x00, 0xff, 0x80, 0x7f};
    for (;;) {
        uint8_t* img = mt9v03x_csi_image_take();
        show_grayscale(img);
        if (!slave_key[0].get()) {
            usb_cdc_send_buff(buf, 4);
            usb_cdc_send_buff(img, N * M);
        }
        mt9v03x_csi_image_release();
    }
}

bool imgUSBXferNode() { return FuncThread(imgUSBXferEntry, "imgUSBXfer"); }
