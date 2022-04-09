#ifndef _nodes_hpp
#define _nodes_hpp

// 无线串口接收进程, 用于遥控
bool wirelessNode();

// apriltag检测, 需要初始化mt9v03x
bool apriltagDetectNode();

// 上位机传图, 需要初始化usb_cdc和mt9v03x
bool imgUSBXferNode();

// 使用rtthread timer驱动的周期性电机转速控制
bool motorControlNode();

// 用按键和ips屏幕测试电机
bool testMotorNode();

// 测试qtimer
bool testQTimerNode();

// 结合上位机测试电机PWM, 需要禁用wirelessNode和motorControlNode
bool testMotorPwmNode();

#endif  // _nodes_hpp