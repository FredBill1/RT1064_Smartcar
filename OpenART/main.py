import pyb
import sensor, image, time, math
import os, tf
import gc
import SEEKFREE_IPS114_SPI as ips114
from machine import UART
from pyb import LED


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

ips114.ips_init()
uart = UART(1, baudrate=115200)
# led = LED(4)
# led.off()

net = "model.tflite"
labels = [line.rstrip("\n") for line in open("/sd/labels.txt")] + ["none"]

NET_SIZE = 96
BORDER_WIDTH = (NET_SIZE + 9) // 10
CROP_SIZE = NET_SIZE + BORDER_WIDTH * 2

clock = time.clock()

H = sensor.height()
W = sensor.width()
HEADER = b"\x00\xFF\x80\x7F\xF0"

do_classify = False
result_index = -1

model = tf.load(net, load_to_fb=True)

while True:
    clock.tick()
    do_classify = b"\xA5" in uart.read() or do_classify
    img = sensor.snapshot()
    rects = img.find_rects(threshold=-10, quality=150)  # 代表二值化阈值和四边形最小周长
    if rects:
        rect = rects[0]
        if do_classify:
            uart.write((HEADER + b"\xFF") * 2)  # 通知主板拍到矩形, 可以开始搬运

        corners = rect.corners()
        img.rotation_corr(x_translation=CROP_SIZE, y_translation=CROP_SIZE, corners=corners)

        if do_classify:
            # model = tf.load(net, load_to_fb=True)
            obj = model.classify(img, roi=(BORDER_WIDTH, BORDER_WIDTH, NET_SIZE, NET_SIZE), scale_mul=0.9)[0]
            # tf.free_from_fb()

            res = obj.output()
            m = max(res)
            result_index = res.index(m)
            uart.write((HEADER + result_index.to_bytes(1, "little")) * 2)  # 发送识别结果
            print("%s: %f" % (labels[result_index], m))
            do_classify = False

    img.scale(x_scale=0.5, y_scale=0.5)  # 用于显示到屏幕
    img.draw_string(0, CROP_SIZE >> 1, ("*" if do_classify else " ") + labels[result_index], (255, 0, 0), 2)
    ips114.ips_display(img, 160, 120)

    # print(clock.fps())
