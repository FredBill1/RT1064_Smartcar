from machine import SPI, Pin
import time, sensor, image

# IO接线方法
# 屏         openart
# GND       ---> GND
# VCC       ---> 3.3V
# SCL       ---> B0(SCLK)
# SDA/MISO  ---> B1(MISO)
# RESET     ---> B12
# DC        ---> B13
# CS        ---> B3
# BL        ---> B16 (背光控制)


cs = Pin(("B3", 3))  # 引脚定义 IPS CS引脚接B3
rst = Pin(("C8", 32 + 8))  # 引脚定义 IPS RES引脚接B12
dc = Pin(("C9", 32 + 9))  # 引脚定义 IPS DC引脚接B13
bl = Pin(("B16", 16))  # 引脚定义 IPS BL引脚接B16
spi = SPI(30)  # 创建对象 SPI3总线上的第0个设备

# 定义显示方向
# 0 竖屏模式
# 1 竖屏模式  旋转180
# 2 横屏模式
# 3 横屏模式  旋转180
#  修改参数后请复位OpenART模块
IPS_DISPLAY_DIR = 3

# 初始化屏幕像素
X_MAX_PIXEL = 0
Y_MAX_PIXEL = 0
if 0 == IPS_DISPLAY_DIR:
    X_MAX_PIXEL = 135  # 定义屏幕宽度
    Y_MAX_PIXEL = 240  # 定义屏幕高度
elif 1 == IPS_DISPLAY_DIR:
    X_MAX_PIXEL = 135  # 定义屏幕宽度
    Y_MAX_PIXEL = 240  # 定义屏幕高度
elif 2 == IPS_DISPLAY_DIR:
    X_MAX_PIXEL = 240  # 定义屏幕宽度
    Y_MAX_PIXEL = 135  # 定义屏幕高度
elif 3 == IPS_DISPLAY_DIR:
    X_MAX_PIXEL = 240  # 定义屏幕宽度
    Y_MAX_PIXEL = 135  # 定义屏幕高度


# ips初始化
def ips_init():
    dc.init(Pin.OUT_PP, Pin.PULL_NONE)  # 引脚初始化，方向：输出 无上拉
    rst.init(Pin.OUT_PP, Pin.PULL_NONE)  # 引脚初始化，方向：输出 无上拉
    cs.init(Pin.OUT_PP, Pin.PULL_NONE)  # 引脚初始化，方向：输出 无上拉
    bl.init(Pin.OUT_PP, Pin.PULL_NONE)  # 引脚初始化，方向：输出 无上拉 背光控制

    spi.init(48000000, 0, 0, 8, SPI.LSB)  # 初始化 波特率30000000，极性0，相位0，传输数据长度8位，从高位开始传输数据

    bl.value(1)
    rst.value(0)
    time.sleep(100)
    rst.value(1)
    time.sleep(100)
    write_command(0x11)
    time.sleep(100)

    if 0 == IPS_DISPLAY_DIR:
        write_command(0x36, 0x00)
    elif 1 == IPS_DISPLAY_DIR:
        write_command(0x36, 0xC0)
    elif 2 == IPS_DISPLAY_DIR:
        write_command(0x36, 0x70)
    elif 3 == IPS_DISPLAY_DIR:
        write_command(0x36, 0xA0)

    write_command(0x3A, 0x05)
    write_command(0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33)
    write_command(0xB7, 0x35)
    write_command(0xBB, 0x37)
    write_command(0xC0, 0x2C)
    write_command(0xC2, 0x01)
    write_command(0xC3, 0x12)
    write_command(0xC6, 0x0F)
    write_command(0xD0, 0xA4, 0xA1)
    write_command(0xE0, 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23)
    write_command(0xE1, 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23)
    write_command(0x21)
    write_command(0x11)
    time.sleep(100)
    write_command(0x29)
    ips_clear()


# 写命令
def write_command_byte(c):
    c = c.to_bytes(1, "little")
    cs.value(0)
    dc.value(0)
    spi.write(c)
    cs.value(1)


# 写8位数据
def write_data_byte(c):
    c = c.to_bytes(1, "little")
    cs.value(0)
    dc.value(1)
    spi.write(c)
    cs.value(1)


# 写16位数据
def write_data_16bit(dat):
    write_data_byte(dat >> 8)
    write_data_byte(dat & 0xFF)


# 写命令
def write_command(c, *data):
    write_command_byte(c)
    if data:
        for d in data:
            if d > 255:
                write_data_byte(d >> 8)
                write_data_byte(d & 0xFF)
            else:
                write_data_byte(d)


# 框选坐标
def ips_set_region(x_start, y_start, x_end, y_end):
    if 0 == IPS_DISPLAY_DIR:
        write_command(0x2A)
        write_data_16bit(x_start + 52)
        write_data_16bit(x_end + 52)
        write_command(0x2B)
        write_data_16bit(y_start + 40)
        write_data_16bit(y_end + 40)
    elif 1 == IPS_DISPLAY_DIR:
        write_command(0x2A)
        write_data_16bit(x_start + 53)
        write_data_16bit(x_end + 53)
        write_command(0x2B)
        write_data_16bit(y_start + 40)
        write_data_16bit(y_end + 40)
    elif 2 == IPS_DISPLAY_DIR:
        write_command(0x2A)
        write_data_16bit(x_start + 40)
        write_data_16bit(x_end + 40)
        write_command(0x2B)
        write_data_16bit(y_start + 53)
        write_data_16bit(y_end + 53)
    elif 3 == IPS_DISPLAY_DIR:
        write_command(0x2A)
        write_data_16bit(x_start + 40)
        write_data_16bit(x_end + 40)
        write_command(0x2B)
        write_data_16bit(y_start + 52)
        write_data_16bit(y_end + 52)
    write_command(0x2C)


# 清屏
def ips_clear():
    ips_set_region(0, 0, X_MAX_PIXEL - 1, Y_MAX_PIXEL - 1)
    cs.value(0)
    dc.value(1)
    spi.write(b"\xFF" * (X_MAX_PIXEL * Y_MAX_PIXEL * 2))
    cs.value(1)


# 显示图像
def ips_display(image, sizeX, szeY):
    ips_set_region(0, 0, sizeX - 1, szeY - 1)
    cs.value(0)
    dc.value(1)
    spi.write(image)  # 先发第N行的第I个数据的高八位
    cs.value(1)
