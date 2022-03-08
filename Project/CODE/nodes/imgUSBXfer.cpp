#include "Thread.h"

extern "C" {
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "zf_gpio.h"
#include "zf_usb_cdc.h"
}

#include "apriltag/visualization.hpp"

static void imgUSBXferEntry(void*) {
    using namespace imgProc::apriltag;
    static uint8_t buf[4]{0x00, 0xff, 0x80, 0x7f};
    for (;;) {
        uint8_t* img = mt9v03x_csi_image_take();
        show_grayscale(img);
        if (!gpio_get(C4)) {
            usb_cdc_send_buff(buf, 4);
            usb_cdc_send_buff(img, N * M);
        }
        mt9v03x_csi_image_release();
    }
}

// rtthread::Thread imgUSBXferThread(imgUSBXferEntry, NULL, 512, 1, 1000, "imgUSBXfer");
