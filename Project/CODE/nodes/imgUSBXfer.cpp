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
    AT_SDRAM_NONCACHE_SECTION_ALIGN(static uint8_t buf[N * M + 4], 64);
    buf[0] = 0x00, buf[1] = 0xff, buf[2] = 0x80, buf[3] = 0x7f;
    for (;;) {
        uint8_t* img = mt9v03x_csi_image_take();
        show_grayscale(img);
        if (!gpio_get(C4)) {
            rt_memcpy(buf + 4, img, N * M + 4);
            usb_cdc_send_buff(buf, N * M + 4);
        }
        mt9v03x_csi_image_release();
    }
}

rtthread::Thread imgUSBXferThread(imgUSBXferEntry, NULL, 512, 1, 1000, "imgUSBXfer");
