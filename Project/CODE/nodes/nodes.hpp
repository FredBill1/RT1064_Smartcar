#ifndef _nodes_hpp
#define _nodes_hpp

#include "Thread.h"

// 无线串口接收进程, 用于遥控
extern rtthread::Thread wirelessThread;

// apriltag检测, 需要初始化mt9v03x
extern rtthread::Thread apriltagDetectThread;

// 上位机传图, 需要初始化usb_cdc和mt9v03x
extern rtthread::Thread imgUSBXferThread;

// 用按键和ips屏幕测试电机
extern rtthread::Thread testMotorThread;

#endif  // _nodes_hpp